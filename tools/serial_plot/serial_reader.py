from __future__ import print_function
#
# Pavel Kirienko, 2013 <pavel.kirienko@gmail.com>
#

import serial

class SerialReader:
    def __init__(self, port, baudrate, timeout=None, value_prefix='$'):
        self._value_prefix = value_prefix
        self._port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, writeTimeout=timeout)
        self._x = 0

    def poll(self, value_handler, raw_handler):
        line = self._port.readline()
        if not line.startswith(self._value_prefix):
            raw_handler(line)
        else:
            self._x += 1
            items = line[len(self._value_prefix):].split()
            if items:
                value_handler(self._x, map(float, items))

    def run(self, value_handler, raw_handler):
        while 1:
            try:
                self.poll(value_handler, raw_handler)
            except Exception as ex:
                print('Serial poll failed:', ex)
