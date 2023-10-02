import board
import digitalio
import wifi
import socketpool
import asyncio
import pickle
import arucodict
import displayio
from adafruit_st7789 import ST7789

from controlpacket import *

WIFI_SSID = "SDES"
WIFI_PASSWORD = "password"

UDP_PORT = 1234

class Main:
    def __init__(self):
        pass

    def init_hardware(self):
        self.led = digitalio.DigitalInOut(board.IO21)
        self.led.direction = digitalio.Direction.OUTPUT
        # https://docs.circuitpython.org/projects/st7789/en/latest/index.html
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

        self.configure_wifi()
        self.display_tag()
        
        receive_task = asyncio.create_task(self.receive_task())

        # don't await heartbeat_task because we want it to stop if receive task fails
        await asyncio.wait([receive_task])

if __name__ == "__main__":
    asyncio.run(Main().main())
        
