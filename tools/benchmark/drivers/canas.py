#!/usr/bin/env python
#
# This is actually a quickly implemented testing tool, it is not intended for real use
#

import sys, threading, struct, time
from itertools import count
import pycanbus

class CanAerospaceEsc:
    DATA_TYPE = 2
    CAN_ID = 200

    def __init__(self, iface, self_node_id=None):
        self._self_node_id = int(self_node_id or 254)
        self._sock = pycanbus.Socket(iface)
        self._keep_running = True
        self._dc = 0.0
        self._msgcode = 0
        self._update_event = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()

    def _send_once(self):
        data = struct.pack('>BBBBf', self._self_node_id, CanAerospaceEsc.DATA_TYPE, 0, self._msgcode, self._dc)
        self._sock.send(pycanbus.Frame(data, CanAerospaceEsc.CAN_ID, False))
        self._msgcode = (self._msgcode + 1) & 0xFF

    def _run(self):
        try:
            while self._keep_running:
                self._update_event.wait(0.01 if self._dc > 0 else 1.0)
                self._update_event.clear()
                self._send_once()
        finally:
            self._dc = 0.0
            self._send_once()

    def set_duty_cycle(self, dc):
        self._dc = float(dc)
        self._update_event.set()

    def start(self):
        self.set_duty_cycle(0.01)

    def stop(self):
        self.set_duty_cycle(0.0)

    def dispose(self):
        self._keep_running = False
        self._thread.join()

if __name__ == '__main__':
    try:
        import readline
    except ImportError:
        pass
    cesc = CanAerospaceEsc('slcan0')
    while 1:
        try:
            dc = float(raw_input('Duty cycle: '))
        except ValueError:
            dc = 0.0
        cesc.set_duty_cycle(dc)
