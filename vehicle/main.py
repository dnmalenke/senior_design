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
import terminalio
import pwmio
import supervisor
import time


from adafruit_display_text import label
from adafruit_st7789 import ST7789


from adafruit_motor import motor as Motor

WIFI_SSID = "SDES"
WIFI_PASSWORD = "password"

UDP_PORT = 1234

class Main:
    async def init_hardware(self):
        backlight = pwmio.PWMOut(board.IO6, frequency=500000)
        backlight.duty_cycle = int(0.2 * 65535)
                
        mctl_sleep = digitalio.DigitalInOut(board.IO5)
        mctl_sleep.direction = digitalio.Direction.OUTPUT
        mctl_sleep.value = True

        motor_a_in1= pwmio.PWMOut(board.IO1, frequency = 50)
        motor_a_in2= pwmio.PWMOut(board.IO2, frequency = 50)

        motor_b_in1= pwmio.PWMOut(board.IO3, frequency = 50)
        motor_b_in2= pwmio.PWMOut(board.IO4, frequency = 50)
        
        self.motor_a = Motor.DCMotor(motor_a_in1, motor_a_in2)
        self.motor_b = Motor.DCMotor(motor_b_in1, motor_b_in2)


        # https://docs.circuitpython.org/projects/st7789/en/latest/index.html

        displayio.release_displays()
        spi = busio.SPI(board.IO7, MOSI=board.IO9)
        while not spi.try_lock():
            pass        
        spi.configure(baudrate=24000000) # Configure SPI for 24MHz
        spi.unlock()

        display_bus = displayio.FourWire(spi, command=board.IO43, chip_select=board.IO44) # check if we don't need reset and check if we can go without command pin as well

        self.display = ST7789(display_bus, width=280, height=240, rowstart=20, rotation=90)
                
        print("display done")

        pass

    # David
    async def configure_wifi(self):
        wifi.radio.connect(WIFI_SSID, WIFI_PASSWORD)
        pool = socketpool.SocketPool(wifi.radio)
        
        udp_host = str(wifi.radio.ipv4_address)

        self.sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
        self.sock.bind((udp_host, UDP_PORT))
        self.sock.setblocking(False)
        

        print(f"my ip: {udp_host}")

    # David
    async def display_tag(self):
        id = 0
        id = int(str(wifi.radio.ipv4_address).split('.')[3])
    
        if id < 0 or id > 249:
            print(f'ip address: {id} out of range')
            return
        
        data = arucodict.aruco_dict[id]
        binary_data = ''

        for d in data:
            binary_data = binary_data + f'{d:>08b}'

        print(binary_data)

        splash = displayio.Group()
        self.display.show(splash)

        size = 32

        a_bitmap = displayio.Bitmap(280, 240, 1)
        a_palette = displayio.Palette(1)
        a_palette[0] = 0xFFFFFF
        bg_sprite = displayio.TileGrid(a_bitmap, pixel_shader=a_palette, x=0, y=0)
        splash.append(bg_sprite)

        a_bitmap = displayio.Bitmap(6*size, 6*size, 1)
        a_palette = displayio.Palette(1)
        a_palette[0] = 0x000000
        bg_sprite = displayio.TileGrid(a_bitmap, pixel_shader=a_palette, x = (280 - 6 * size)//2, y = (240 - 6 * size)//2)
        splash.append(bg_sprite)

        color_bitmap = displayio.Bitmap(size, size, 1)
        color_palette = displayio.Palette(1)
        color_palette[0] = 0xFFFFFF

        
        black_palette = displayio.Palette(1)
        black_palette[0] = 0x000000


        xstart = (280 - 6 * size)//2 + size
        x = xstart
        y = (240 - 6 * size)//2 + size

        for b in binary_data:
            if b == '1':
                bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=x, y=y)
                splash.append(bg_sprite)
                pass
            else:
                # bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=black_palette, x=x, y=y)
                # splash.append(bg_sprite)
                pass
            x = x + size
            if x == xstart + 4 * size:
                x = xstart
                y = y + size

    async def receive_task(self):
        if self.sock is None:
            return

        while True:
            udp_buffer = bytearray(512)
            await asyncio.sleep(0.005)
            try:
                size, _ = self.sock.recvfrom_into(udp_buffer)
                if size > 0:
                    packet = json.loads(udp_buffer)
                    # print(f"l: {packet['left_speed']} r: {packet['right_speed']}")
                    self.motor_a.throttle = float(packet['left_speed'])
                    self.motor_b.throttle = float(packet['right_speed'])
            except:
                pass

    async def heartbeat_task(self):        
        self.led = digitalio.DigitalInOut(board.IO21)
        self.led.direction = digitalio.Direction.OUTPUT
        
        while True:
            self.led.value = True
            await asyncio.sleep(0.5)
            self.led.value = False
            await asyncio.sleep(0.5)
            

    async def main(self):
        heartbeat_task = asyncio.create_task(self.heartbeat_task())

        # asyncio.run(self.init_hardware)        
        # self.init_hardware()
        asyncio.run(self.init_hardware())
        try:
            asyncio.run(self.configure_wifi())
        except:
            print("Wifi connection failed.\nRestarting in 5 seconds.")
            time.sleep(5)
            supervisor.reload()

        asyncio.run(self.display_tag())
        
        receive_task = asyncio.create_task(self.receive_task())

        await asyncio.funcs.gather(heartbeat_task)
        # while True:
        #     pass

if __name__ == "__main__":
    asyncio.run(Main().main())
        
