import board
import digitalio
import wifi
import socketpool
import asyncio
import asyncio.funcs
import json
import arucodict
import displayio
import busio
import pwmio
import supervisor
import time

from adafruit_st7789 import ST7789
from adafruit_motor import motor as Motor

WIFI_SSID = "SDES"
WIFI_PASSWORD = "password"

UDP_PORT = 1234

class Main:
    async def init_hardware(self):
        print("Initializing hardware.")

        backlight = pwmio.PWMOut(board.IO6, frequency = 500000)
        backlight.duty_cycle = int(0.2 * 65535)
                
        mctl_sleep = digitalio.DigitalInOut(board.IO5) # could replace with pull up resistor
        mctl_sleep.direction = digitalio.Direction.OUTPUT
        mctl_sleep.value = True

        motor_a_in1 = pwmio.PWMOut(board.IO1, frequency = 50)
        motor_a_in2 = pwmio.PWMOut(board.IO2, frequency = 50)

        motor_b_in1 = pwmio.PWMOut(board.IO3, frequency = 50)
        motor_b_in2 = pwmio.PWMOut(board.IO4, frequency = 50)
        
        self.motor_a = Motor.DCMotor(motor_a_in1, motor_a_in2)
        self.motor_b = Motor.DCMotor(motor_b_in1, motor_b_in2)

        displayio.release_displays()
        spi = busio.SPI(clock = board.IO7, MOSI = board.IO9)
        while not spi.try_lock():
            pass        
        spi.configure(baudrate = 24000000) # Configure SPI for 24MHz
        spi.unlock()

        display_bus = displayio.FourWire(spi, command = board.IO43, chip_select = board.IO44) # check if we don't need reset and check if we can go without command pin as well

        self.display = ST7789(display_bus, width = 280, height = 240, rowstart = 20, rotation = 90)

        print("Hardware initialization complete.")

    async def configure_wifi(self):
        print(f"Connecting to WiFi...\nNetwork: {WIFI_SSID}\nPassword: {WIFI_PASSWORD}")
        
        wifi.radio.connect(WIFI_SSID, WIFI_PASSWORD)
        pool = socketpool.SocketPool(wifi.radio)
        
        udp_host = str(wifi.radio.ipv4_address)

        self.sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
        self.sock.bind((udp_host, UDP_PORT))
        self.sock.setblocking(False)
        
        print(f"Connected. Current IP: {udp_host}")

    async def display_tag(self):
        id = -1

        try:
            id = int(str(wifi.radio.ipv4_address).split('.')[3])
        except:
            print('Failed to parse IP address.')
            pass
    
        if id < 0 or id > 249:
            print(f'IP address: {id} out of range.')
            return
        
        data = arucodict.aruco_dict[id]
        binary_data = ''

        for d in data:
            binary_data = binary_data + f'{d:>08b}'

        tag_size_px = 32

        display_group = displayio.Group()

        white_palette = displayio.Palette(1)
        white_palette[0] = 0xFFFFFF 

        black_palette = displayio.Palette(1)
        black_palette[0] = 0x000000 

        display_background = displayio.Bitmap(280, 240, 1)        
        display_group.append(displayio.TileGrid(display_background, pixel_shader = white_palette, x = 0, y = 0))

        tag_start_x = (280 - 6 * tag_size_px) // 2
        tag_start_y = (240 - 6 * tag_size_px) // 2

        tag_background = displayio.Bitmap(6 * tag_size_px, 6 * tag_size_px, 1)
        tag_background_sprite = displayio.TileGrid(tag_background, pixel_shader = black_palette, x = tag_start_x, y = tag_start_y)
        display_group.append(tag_background_sprite)

        tag_tile = displayio.Bitmap(tag_size_px, tag_size_px, 1)

        x_start = tag_start_x + tag_size_px
        x = x_start
        y = tag_start_y + tag_size_px

        for b in binary_data:
            if b == '1':
                tag_tile_sprite = displayio.TileGrid(tag_tile, pixel_shader = white_palette, x = x, y = y)
                display_group.append(tag_tile_sprite)
            x = x + tag_size_px
            if x == x_start + 4 * tag_size_px:
                x = x_start
                y = y + tag_size_px

        self.display.show(display_group)

    async def receive_task(self):
        if self.sock is None:
            print("Socket not initialized.")
            return

        while True:
            udp_buffer = bytearray(512)
            await asyncio.sleep(0.005)
            try:
                size, _ = self.sock.recvfrom_into(udp_buffer)
                if size > 0:
                    packet = json.loads(udp_buffer)
                    self.motor_a.throttle = float(packet['left_speed'])
                    self.motor_b.throttle = float(packet['right_speed'])
            except:
                pass

    async def heartbeat_task(self):     
        # initialize onboard led here so it's not dependant on other hardware init   
        self.led = digitalio.DigitalInOut(board.IO21)
        self.led.direction = digitalio.Direction.OUTPUT
        
        while True:
            self.led.value = True
            await asyncio.sleep(0.5)
            self.led.value = False
            await asyncio.sleep(0.5)
            

    async def main(self):
        heartbeat_task = asyncio.create_task(self.heartbeat_task())

        asyncio.run(self.init_hardware())

        try:
            asyncio.run(self.configure_wifi())
        except:
            print("Wifi connection failed.\nRestarting in 5 seconds.")
            time.sleep(5)
            supervisor.reload()

        time.sleep(1)

        asyncio.run(self.display_tag())
        
        asyncio.create_task(self.receive_task())

        await asyncio.funcs.gather(heartbeat_task)

if __name__ == "__main__":
    asyncio.run(Main().main())
        
