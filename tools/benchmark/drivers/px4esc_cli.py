#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#
# PX4ESC CLI API
#

import logging
from generic_serial_cli import SerialCli

class Px4EscCli(SerialCli):
    def __init__(self, port, logger=None):
        SerialCli.__init__(self, port=port, baudrate=115200, timeout=1.0, logger=logger)

    def arm(self):
        self.writeln('sp arm')

    def set_duty_cycle(self, dc):
        self.writeln('sp %.3f', dc)

    def stop(self):
        self.set_duty_cycle(0)
        self.set_duty_cycle(0)

    def beep(self, frequency=500, duration=0.3, sync=True):
        self.writeln('beep %d %d', frequency, int(duration * 1000 + .5))
        if sync:
            time.sleep(duration)

if __name__ == '__main__':
    import time, sys
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(asctime)s %(levelname)s: %(message)s')
    px4esc = Px4EscCli('/dev/ttyUSB1')
    px4esc.arm()
    px4esc.beep(frequency=1000, duration=1.0)
    px4esc.set_duty_cycle(0.01)
    time.sleep(2.0)
    px4esc.set_duty_cycle(0.4)
    time.sleep(2.0)
    px4esc.set_duty_cycle(0.01)
    time.sleep(0.4)
    px4esc.stop()
