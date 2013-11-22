#
# Pavel Kirienko, 2013 <pavel.kirienko@gmail.com>
#

from PyQt4.QtCore import Qt
from PyQt4.QtGui import QColor, QVBoxLayout, QWidget
from pyqtgraph import PlotWidget, mkPen
import numpy, time


class RealtimePlotWidget(QWidget):
    COLORS = [Qt.red, Qt.blue, Qt.green, Qt.magenta, Qt.cyan,
              Qt.darkRed, Qt.darkBlue, Qt.darkGreen, Qt.darkYellow, Qt.gray]

    def __init__(self, parent=None):
        super(RealtimePlotWidget, self).__init__(parent)
        self._plot_widget = PlotWidget()
        self._plot_widget.setBackground((0, 0, 0))
        self._plot_widget.addLegend()
        self._plot_widget.showButtons()
        self._plot_widget.enableAutoRange()
        self._plot_widget.showGrid(x=True, y=True, alpha=0.2)
        vbox = QVBoxLayout()
        vbox.addWidget(self._plot_widget)
        self.setLayout(vbox)

        self._color_index = 0
        self._curves = {}

    def add_curve(self, curve_id, curve_name, data_x=[], data_y=[]):
        color = QColor(self.COLORS[self._color_index % len(self.COLORS)])
        self._color_index += 1
        pen = mkPen(color, width=1)
        plot = self._plot_widget.plot(name=curve_name, pen=pen)
        data_x = numpy.array(data_x)
        data_y = numpy.array(data_y)
        self._curves[curve_id] = {'x': data_x, 'y': data_y, 'plot': plot}

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._plot_widget.removeItem(self._curves[curve_id]['plot'])
            del self._curves[curve_id]

    def set_x_range(self, left, right):
        self._plot_widget.setRange(xRange=(left, right))

    def update_values(self, curve_id, x, y):
        curve = self._curves[curve_id]
        curve['x'] = numpy.append(curve['x'], x)
        curve['y'] = numpy.append(curve['y'], y)

    def redraw(self):
        for curve in self._curves.values():
            if len(curve['x']):
                curve['plot'].setData(curve['x'], curve['y'])

    def lazy_redraw(self, period):
        timestamp = time.time()
        if not hasattr(self, '_prev_lazy_redraw'):
            self._prev_lazy_redraw = 0.0
        if timestamp - self._prev_lazy_redraw > period:
            self._prev_lazy_redraw = timestamp
            self.redraw()


if __name__ == '__main__':
    from PyQt4 import QtGui
    import sys, random, threading, time

    app = QtGui.QApplication(sys.argv)
    w = RealtimePlotWidget()

    w.add_curve('1', 'Vincent van Gogh')
    for i in xrange(100):
        w.update_values('1', [i], [random.random()])

    w.redraw()
    w.show()

    def thread():
        x = 100
        while 1:
            time.sleep(0.1)
            w.update_values('1', [x], [random.random()])
            w.redraw()
            x += 1
    thd = threading.Thread(target=thread)
    thd.daemon = True
    thd.start()

    exit(app.exec_())
