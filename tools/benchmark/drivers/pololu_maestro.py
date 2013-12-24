#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#
# Pololu Maestro PPM Controller API
#

import serial

BAUDRATE = 9600
TIMEOUT = 1.0

class PololuMaestro:
    def __init__(self, port):
        self._port = serial.Serial(port=port, baudrate=BAUDRATE, timeout=TIMEOUT, writeTimeout=TIMEOUT)

    def set_ppm_fraction(self, channel, value):
        integer = int(value * 254 + 0.5)
        integer = min(max(integer, 0), 254)
        channel = min(max(int(channel), 0), 24)
        self._port.write('\xFF' + chr(channel) + chr(integer))

    def perform_interactive_esc_calibration(self):
        def getchar():
            import sys, tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        print 'Commands: + for max, - for min'
        # Turnigy: http://www.hobbyking.com/hobbyking/store/uploads/981522291X561242X33.pdf
        while 1:
            ch = getchar()
            print ch
            if ch == '+':
                self.set_ppm_fraction(0, 1.0)
            elif ch == '-':
                self.set_ppm_fraction(0, 0.0)
            else:
                break

if __name__ == '__main__':
    import time
    pm = PololuMaestro('/dev/ttyACM0')
    pm.set_ppm_fraction(0, 0.0)
    time.sleep(4.0)
    pm.set_ppm_fraction(0, 0.1)
    time.sleep(4.0)
    pm.set_ppm_fraction(0, 0.3)
    time.sleep(2.0)
    pm.set_ppm_fraction(0, 0.05)
    time.sleep(2.0)
    pm.set_ppm_fraction(0, 0.0)
