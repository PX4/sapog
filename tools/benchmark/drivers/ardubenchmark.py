#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#
# Ardubenchmark listener
#

import serial, struct

TIMEOUT = 3.0
BAUDRATE = 115200

FRAME_HEADER = 0xFA

VOLT_REFERENCE = 5.0

# 3DR Power Brick
#VOLT_MULT = 10.0
#CURR_MULT = 17.0

# AUAV ACSP4
#VOLT_MULT = 13.653333333
#CURR_MULT = 36.367515152

VOLT_MULT = 12.8
CURR_MULT = 33.0

def _scale_tach(x):
    # tick_duration = 1 / (16e6 / 256)
    # 16 us per tick
    return x * 16 / 1e6

def _scale_voltage(x):
    unscaled = x * (VOLT_REFERENCE / 1024.0)
    return unscaled * VOLT_MULT

def _scale_current(x):
    unscaled = x * (VOLT_REFERENCE / 1024.0)
    return unscaled * CURR_MULT

class Ardubenchmark:
    def __init__(self, port):
        self._port = serial.Serial(port=port, baudrate=BAUDRATE, timeout=TIMEOUT, writeTimeout=TIMEOUT)
        self._keep_going = True

    def poll(self, handler):
        while FRAME_HEADER != ord(self._port.read(1)):
            pass
        checksum = ord(self._port.read(1))
        payload = self._port.read(2 + 2 + 2)
        if checksum != (sum(map(ord, payload)) & 0xFF):
            return False
        tach, vtg, cur = struct.unpack('<HHH', payload)
        handler(_scale_tach(tach), _scale_voltage(vtg), _scale_current(cur))
        return True

    def run(self, handler):
        self._keep_going = True
        while self._keep_going:
            try:
                res = self.poll(handler)
                if not res:
                    print 'Ardubenchmark checksum mismatch'
            except serial.serialutil.SerialException:
                raise
            except Exception, ex:
                print 'Ardubenchmark poll failed:', ex

    def stop(self):
        self._keep_going = False

if __name__ == '__main__':
    import time
    import sys

    def handler(tach, voltage, current):
        print('%.3f % 6.0f RPM  % 5.1f V  % 5.1f A' % (time.time(), tach, voltage, current))

    abm = Ardubenchmark(sys.argv[1])
    abm.run(handler)
