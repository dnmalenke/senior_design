import board
import digitalio
import wifi
import socketpool
import asyncio
import pickle
import arucodict
import displayio
import busio
import terminalio

from adafruit_display_text import label
from adafruit_st7789 import ST7789

from controlpacket import *

WIFI_SSID = "SDES"
WIFI_PASSWORD = "password"

UDP_PORT = 1234

class Main:
    def init_hardware(self):
        self.led = digitalio.DigitalInOut(board.IO21)
        self.led.direction = digitalio.Direction.OUTPUT

        # https://docs.circuitpython.org/projects/st7789/en/latest/index.html

        displayio.release_displays()

        spi = busio.SPI(board.IO8, board.IO10, board.IO9)
        while not spi.try_lock():
            pass        
        spi.configure(baudrate=24000000) # Configure SPI for 24MHz
        spi.unlock()

        display_bus = displayio.FourWire(spi, command=board.IO6, chip_select=board.IO7, reset=board.IO5) # check if we don't need reset and check if we can go without command pin as well

        display = ST7789(display_bus, width=280, height=240, rowstart=20, rotation=90)
        splash = displayio.Group()
        display.show(splash)

        color_bitmap = displayio.Bitmap(280, 240, 1)
        color_palette = displayio.Palette(1)
        color_palette[0] = 0x00FF00  # Bright Green

        bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
        splash.append(bg_sprite)

        # Draw a smaller inner rectangle
        inner_bitmap = displayio.Bitmap(240, 200, 1)
        inner_palette = displayio.Palette(1)
        inner_palette[0] = 0xAA0088  # Purple
        inner_sprite = displayio.TileGrid(inner_bitmap, pixel_shader=inner_palette, x=20, y=20)
        splash.append(inner_sprite)

        # Draw a label
        text_group = displayio.Group(scale=3, x=37, y=120)
        text = "Hello World!"
        text_area = label.Label(terminalio.FONT, text=text, color=0xFFFF00)
        text_group.append(text_area)  # Subgroup for text scaling
        splash.append(text_group)
        

        pass

    # David
    def configure_wifi(self):
        wifi.radio.connect(WIFI_SSID,WIFI_PASSWORD)
        pool = socketpool.SocketPool(wifi.radio)
        
        udp_host = str(wifi.radio.ipv4_address)

        self.sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
        self.sock.bind((udp_host, UDP_PORT))

    # Jon
    def handle_packet(self, packet: ControlPacket):

        pass

    # David
    def display_tag(self):
        id = int(str(wifi.radio.ipv4_address).split('.')[3])

        if id < 0 or id > 249:
            print('ip address out of range')
            return
        
        data = arucodict.aruco_dict[id]

    async def receive_task(self):
        if self.sock is None:
            return
        
        udp_buffer = bytearray(512)

        while True:
           self.sock.recvfrom_into(udp_buffer)           
           packet = pickle.loads(udp_buffer)
           if packet is ControlPacket:
                self.handle_packet(packet)

    async def heartbeat_task(self):
        while True:
            self.led.value = True
            await asyncio.sleep(0.5)
            self.led.value = False
            await asyncio.sleep(0.5)
            

    async def main(self):
        heartbeat_task = asyncio.create_task(self.heartbeat_task())

        self.init_hardware()
        self.configure_wifi()
        self.display_tag()
        
        receive_task = asyncio.create_task(self.receive_task())

        # don't await heartbeat_task because we want it to stop if receive task fails
        await asyncio.wait([receive_task])

if __name__ == "__main__":
    asyncio.run(Main().main())
        
