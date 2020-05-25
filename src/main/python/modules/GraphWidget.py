import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QRectF, pyqtSlot
from PyQt5.QtGui import QColor


class GraphWidget(pg.PlotWidget):
    def __init__(self):
        super().__init__()
        self.an_plots = list()      # [id, plot, pos]
        self.tag_plots = list()     # [id, plot, pos]

    def set_room(self, x, y):
        rect = QRectF(0, 0, float(x)+0.5, float(y)+0.5)
        self.plot_item().setRange(rect)
        self.view_box().setLimits(xMin=0, yMin=0, xMax=float(x), yMax=float(y))
        return

    def plot_item(self):
        return self.getPlotItem()

    def view_box(self):
        return self.getPlotItem().getViewBox()

    @pyqtSlot(list)
    def update_anchor(self, data):
        pos = data[1]
        for plot in self.an_plots:
            if plot[0] == data[0]:
                self.plot_update_anchor(plot[1], [pos[0], pos[1], pos[2]])
                return

        anchor = self.plot_add_anchor([pos[0], pos[1], pos[2]])
        self.an_plots.append([data[0], anchor, [pos[0], pos[1], pos[2]]])
        return anchor

    @pyqtSlot(list)
    def update_tag(self, data):
        pos = data[1]
        for plot in self.tag_plots:
            if plot[0] == data[0]:
                self.plot_update_tag(plot[1], [pos[0], pos[1], pos[2]])
                return
        tag = self.plot_add_tag([1, 1, 1])
        self.tag_plots.append([data[0], tag, [0, 0, 0]])
        return tag

    def plot_add_anchor(self, pos):
        plot_data_item = pg.ScatterPlotItem(
            [float(pos[0])],
            [float(pos[1])],
            size=10,
            brush=(QColor('red')))
        self.addItem(plot_data_item)
        self.invertY(True)
        self.invertX(True)

        return plot_data_item

    def plot_add_tag(self, pos):
        plot_data_item = pg.ScatterPlotItem(
            [float(pos[0])],
            [float(pos[1])],
            size=7,
            brush=(QColor('blue')))
        self.addItem(plot_data_item)
        return plot_data_item

    @staticmethod
    def plot_update_anchor(plot, pos):
        plot.setData([pos[0]], [pos[1]])

    @staticmethod
    def plot_update_tag(plot, pos):
        plot.setData([pos[0]], [pos[1]])
