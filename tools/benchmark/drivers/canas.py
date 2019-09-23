#!/usr/bin/env python
#
# This is actually a quickly implemented testing tool, it is not intended for real use
#

from __future__ import print_function
import threading, struct, time
from itertools import count
import pycanbus

class CanAerospaceEsc:
    DATA_TYPE = 2
    CAN_ID = 200

    def __init__(self, iface, self_node_id=None, redundancy_channel_id=None, service_code=None):
        self._self_node_id = int(self_node_id or 254)
        self._redund_chan = int(redundancy_channel_id or 0)
        self._service_code = int(service_code or 0)
        self._sock = pycanbus.Socket(iface)
        self._keep_running = True
        self._dc = 0.0
        self._msgcode = 0
        self._update_event = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()

    def _send_once(self):
        data = struct.pack('>BBBBf', self._self_node_id, CanAerospaceEsc.DATA_TYPE, self._service_code, self._msgcode,
                           self._dc)
        can_id = CanAerospaceEsc.CAN_ID | (65536 * self._redund_chan)
        extended_frame = False if self._redund_chan == 0 else True
        self._sock.send(pycanbus.Frame(data, can_id, extended_frame))
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
    import sys
    try:
        import readline
    except ImportError:
        pass
    redund_chan = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    service_code = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    cesc = CanAerospaceEsc('can0', redundancy_channel_id=redund_chan, service_code=service_code)
    print('Redundancy channel:', redund_chan, 'Service code:', service_code)
    while 1:
        try:
            dc = float(raw_input('Duty cycle: '))
        except ValueError:
            dc = 0.0
        cesc.set_duty_cycle(dc)
