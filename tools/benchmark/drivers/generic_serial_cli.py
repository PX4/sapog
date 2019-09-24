#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#

import serial, time, logging

NEWLINE = '\r\n'

class SerialCliError(Exception):
    pass

class SerialCli:
    def __init__(self, port, baudrate, timeout, logger=None):
        self._port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, writeTimeout=timeout)
        self._logger = logger or logging.getLogger()
        self.require_echo = True

    def writeln(self, fmt, *args, **kwoptions):
        enforce_echo = kwoptions.get('enforce_echo', False)
        command = NEWLINE + ((fmt % args) if args else fmt) + NEWLINE
        # Old data in the RX buffer may contain exactly the same string as we're going to get echoed back
        # Thus, in order to avoid possible collision we need to drop the RX buffer before write()ing
        for dropped_line in self.drop_rx_fifo().split('\n'):
            self._logger.debug('CLI xx> %s', repr(dropped_line.rstrip()))
        self._logger.debug('CLI <-- %s', repr(command.rstrip()))
        self._port.write(command)
        if self.require_echo:
            time.sleep(0.001)
            deadline = time.time() + self._port.timeout
            while deadline > time.time():
                try:
                    line = self.readln()
                except serial.serialutil.SerialException as ex:
                    if enforce_echo:
                        raise SerialCliError('Echo check failed: %s' % ex)
                    self._logger.error(ex, exc_info=True)
                    break
                else:
                    if line.strip().endswith(command.strip()): # There may or may not be a CLI prompt
                        break

    def readln(self):
        line = self._port.readline()
        self._logger.debug('CLI --> %s', repr(line.rstrip()))
        return line

    def drop_rx_fifo(self):
        return self._port.read(self._port.inWaiting())
