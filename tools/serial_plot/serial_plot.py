#!/usr/bin/env python
#
# Pavel Kirienko, 2013 <pavel.kirienko@gmail.com>
#

from __future__ import print_function
from PyQt4 import QtGui
from plot_widget import RealtimePlotWidget
from serial_reader import SerialReader
import sys, threading, time


SER_PORT = '/dev/ttyUSB0' if len(sys.argv) < 2 else sys.argv[1]
SER_BAUDRATE = 115200


def raw_handler(line):
    print(line)

def value_handler(x, values):
    for i,val in enumerate(values):
        add_data_point(i, x, val)
    plot.lazy_redraw(0.2)

def add_data_point(curve_id, x, y):
    try:
        plot.update_values(curve_id, [x], [y])
    except KeyError:
        plot.add_curve(curve_id, str(curve_id), [x], [y])


initial_timestamp = time.time()

app = QtGui.QApplication(sys.argv)
plot = RealtimePlotWidget()

reader = SerialReader(SER_PORT, SER_BAUDRATE)

thd = threading.Thread(target=reader.run, args=(value_handler, raw_handler))
thd.daemon = True
thd.start()

plot.redraw()
plot.show()
exit(app.exec_())
