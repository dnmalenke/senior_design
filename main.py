import time
import board
import digitalio
import wifi
import socketpool

from controlpacket import *

class Main:
    def __init__(self):
        pass

    def init_hardware(self):
        self.led = digitalio.DigitalInOut(board.IO21)
        self.led.direction = digitalio.Direction.OUTPUT
        pass

    # David
    def configure_wifi(self):
        # https://github.com/adafruit/circuitpython/blob/main/tests/circuitpython-manual/socketpool/server/cpy-server.py
        # asyncio for packet waiting/handling
        pass

    # Jon
    def handle_packet(self, packet: ControlPacket):
        pass

    # David
    def display_tag(self):
        pass

    def main(self):
        while True:
            self.led.value = True
            time.sleep(0.5)
            self.led.value = False
            time.sleep(0.5)


if __name__ == "__main__":
    Main().main()
        
