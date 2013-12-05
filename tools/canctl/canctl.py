#!/usr/bin/env python
#
# This is actually a quickly implemented testing tool, it is not intended for real use
#

import sys, threading, struct, time
from itertools import count
import pycanbus
try:
    import readline
except ImportError:
    pass

NODE_ID = 254
DATA_TYPE = 2
CAN_ID = 200

dc = None

def can_thread(sock):
    for i in count():
        #print sock.recv()
        time.sleep(0.1)
        if dc is not None:
            data = struct.pack('>BBBBf', NODE_ID, DATA_TYPE, 0, i & 0xFF, dc)
            sock.send(pycanbus.Frame(data, CAN_ID, False))

sock = pycanbus.Socket('slcan0')

thd = threading.Thread(target=can_thread, args=(sock,))
thd.daemon = True
thd.start()

while 1:
    try:
        dc = float(raw_input('Duty cycle: '))
    except ValueError:
        dc = 0.0
