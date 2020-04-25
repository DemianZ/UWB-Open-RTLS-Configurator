from PyQt5.QtCore import QThread, pyqtSignal
import atexit
import logging as log
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv


class UneNavMethod(Enum):
    # Class enumerators
    lse = 1
    weighted_lse = 2
    kalman = 3


class UneTwr(QThread):

    # Task signals

    # Class variables
    m_X = 0
    m_Y = 1
    m_Z = 2
    m_D = 3
    m_nav_method = 0
    m_anchor_min = 4
    m_anchor_dict = dict()
    m_tag_dict = dict()
    m_tag_pos = [0, 0, 0]

    def __init__(self, nav_method):
        self.m_nav_method = nav_method
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

    # Task loop
    def run(self):
        self.prepare_data()
        while True:
            self.get_position()

    # !!! Test method. Delete later
    def prepare_data(self):
        self.add_anchor('A01',  1,  1, 10)
        self.add_anchor('A02', 65,  3, 10)
        self.add_anchor('A03', 71, 27, 10)
        self.add_anchor('A04',  5, 50, 10)
        self.add_anchor('A05', 30, 20, 10)
        self.add_anchor('A06', 68, 56, 10)
        self.add_anchor('A07', 30, 53, 10)

        self.add_tag_meas('A01', -39.407794428087070)
        self.add_tag_meas('A02', -12.838443445709743)
        self.add_tag_meas('A03',   3.778255171437351)
        self.add_tag_meas('A04',  42.789885180275920)
        self.add_tag_meas('A05',  -4.076523920811560)
        self.add_tag_meas('A06',  22.185267236214287)
        self.add_tag_meas('A07',  32.712274988174485)

    # Add new anchor with position
    def add_anchor(self, key, x, y, z):
        self.m_anchor_dict[key] = [x, y, z]

    # Get anchor position
    def get_anchor_pos(self, key):
        return self.m_anchor_dict[key]

    # Get distance between anchor and point [x, y, z]
    def get_mtxh_meas(self, key, point):
        dX = self.m_anchor_dict[key][self.m_X] - point[self.m_X]
        dY = self.m_anchor_dict[key][self.m_Y] - point[self.m_Y]
        dZ = self.m_anchor_dict[key][self.m_Z] - point[self.m_Z]

        return [dX, dY, dZ, np.sqrt(dX**2 + dY**2 + dZ**2)]

    # Add tag's measurement
    def add_tag_meas(self, key_anchor, meas, snr=-1):
        self.m_tag_dict[key_anchor] = [meas, snr]

    # Get tag measurements
    def get_tag_measurement(self):
        return self.m_tag_dict

    # Get current tag position
    def get_tag_pos(self):
        return self.m_tag_pos

    # Set tag position
    def set_tag_pos(self, x, y, z):
        self.m_tag_pos = [x, y, z]

    # Clear anchors dictionary
    def clr_anchor(self, key, meas, snr=-1):
        self.m_anchor_dict.clear()

    # Clear tag's  measurement (used before add measurements for new epoch)
    def clr_tag_meas(self):
        self.m_tag_dict.clear()

    def get_position(self):
        if self.m_nav_method == UneNavMethod.lse:
            self.get_position_lse()
        elif self.m_nav_method == UneNavMethod.weighted_lse:
            self.get_position_wlse()
        elif self.m_nav_method == UneNavMethod.Kalman:
            self.get_position_kalman()

    def get_position_lse(self):

        num_anchors = len(self.m_anchor_dict)
        num_attempts = 6
        flg_stop_calc = False

        if num_anchors < self.m_anchor_min:
            print("ERROR: [UneTwr::get_position_lse] Not enough anchors")
            return

        while (num_attempts > 0) and (flg_stop_calc is True):

            num_attempts -= 1

            # Fill matricies
            tag_prev_pos = self.get_tag_pos()

            mtx_h = []
            mtx_y = []
            mtx_x = np.array(tag_prev_pos)
            for a in self.m_anchor_dict:
                tr = self.get_mtxh_meas(a, tag_prev_pos)
                meas_dict = self.get_tag_measurement()
                if len(mtx_h) == 0:
                    mtx_h = np.array([tr[self.m_X] / tr[self.m_D], tr[self.m_Y] / tr[self.m_D], tr[self.m_Z] / tr[self.m_D]])
                    mtx_y = np.array([meas_dict[a]])
                else:
                    mtx_h = np.append((mtx_h, [tr[self.m_X] / tr[self.m_D], tr[self.m_Y] / tr[self.m_D], tr[self.m_Z] / tr[self.m_D]]))
                    mtx_y = np.append(mtx_y, [meas_dict[a]])

            mtx_h_trp = mtx_h.transpose()
            k = np.matmul(inv(np.matmul(mtx_h_trp, mtx_h)), mtx_h_trp)
            mtx_x = np.matmul(k, mtx_y)

    def get_position_wlse(self):
        pass

    def get_position_kalman(self):
        pass
