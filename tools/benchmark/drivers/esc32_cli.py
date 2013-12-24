#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#
# ESC32 CLI API
#

import logging, time
from itertools import count
from generic_serial_cli import SerialCli

class Esc32CliException(Exception):
    pass

class Esc32Cli(SerialCli):
    def __init__(self, port, logger=None):
        SerialCli.__init__(self, port=port, baudrate=115200, timeout=1.0, logger=logger)

    def start(self):
        self.writeln('arm')
        self.writeln('start')
        # ESC32 spinup algorithm sucks so much.
        while 1:
            time.sleep(0.2)
            self.writeln('status')
            while 1:
                line = self.readln()
                if line.strip().startswith('ESC STATE'):
                    state = line.split()[-1]
                    if state == 'RUNNING':
                        return
                    if state == 'STARTING' or state == 'PRE-START':
                        break
                    self.stop()
                    raise Esc32CliException('Spinup failed (no surprise)')
                elif not line:
                    break

    def stop(self):
        self.writeln('disarm')

    def set_duty_cycle(self, dc):
        dc = min(max(dc, 0.0), 1.0)
        self.writeln('duty %.2f', dc * 100)

    def beep(self):
        self.writeln('beep 300 1000')

if __name__ == '__main__':
    import time, sys
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(asctime)s %(levelname)s: %(message)s')
    esc32 = Esc32Cli('/dev/ttyUSB1')
    esc32.beep()
    esc32.start()
    esc32.set_duty_cycle(0.1)
    time.sleep(2.0)
    esc32.set_duty_cycle(0.3)
    time.sleep(2.0)
    esc32.set_duty_cycle(0.5)
    time.sleep(2.0)
    esc32.set_duty_cycle(0.1)
    time.sleep(2.0)
    esc32.stop()
