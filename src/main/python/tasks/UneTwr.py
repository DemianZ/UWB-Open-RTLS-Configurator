"""@package UneTwr
Documentation for this module.

More details.
"""
from PyQt5.QtCore import QThread, pyqtSignal
import atexit
import logging as log
from enum import Enum
import time
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from numpy.linalg import inv


class UneNavMethod(Enum):
    """Documentation for a class.

    More details.
    """
    lse = 1
    weighted_lse = 2
    kalman = 3


class UneTwr(QThread):
    """Documentation for a class.

    More details.
    """
    # Auxiliary variables
    m_X = 0
    m_Y = 1
    m_Z = 2
    m_D = 3
    m_T = 3
    m_nav_method = 0
    m_anchor_min = 4
    m_poserr_trsh = 0.001

    # Indices of measurements
    m_ind_dist = 0
    m_ind_cnr = 1

    m_anchors = dict()
    m_tag_meas = dict()
    m_tag_pvt = [0, 0, 0]

    # For postprocessing
    m_pp_meas = dict()
    m_pp_meas_num = 0

    def __init__(self, nav_method):
        """ The constructor """
        self.m_nav_method = nav_method
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

    def run(self):
        """ Documentation for a method """

        self.prepare_data()

        x, y, z = [], [], []

        flg_compute = True

        while True:
            if flg_compute is True:

                flg_compute = False

                for epoch in self.m_pp_meas:
                    print('-I- [UneTwr::run] Compute tag PVT. UTC:', epoch)

                    # Forming tag measurement dictionary for current epoch
                    self.clr_tag_meas()
                    for a_name, a_meas in self.m_pp_meas[epoch].items():
                        dist = a_meas[self.m_ind_dist]
                        cnr = a_meas[self.m_ind_cnr]
                        self.add_tag_meas(a_name, dist, cnr)

                    res = self.get_position()
                    x.append(res[0])
                    y.append(res[1])
                    z.append(res[2])

                    time.sleep(0.05)

                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.plot_wireframe(x, y, np.array([z, z]), rstride=10, cstride=10)
                ax.set_xlim3d(min(x) - 2, max(x) + 2)
                ax.set_ylim3d(min(y) - 2, max(y) + 2)
                ax.set_zlim3d(min(z) - 2, max(z) + 2)
                ax.set_xlabel('X axis, m')
                ax.set_ylabel('Y axis, m')
                ax.set_zlabel('Z axis, m')
                plt.show()

            time.sleep(30)

    def prepare_data(self):
        """ !!! Test method. Delete later.
            Load description and measurements from JSON file
            for postprocessing
        """

        # Load data for postprocessing
        with open("logs\\my_new_log.json", "r") as read_file:
            desc_meas = json.load(read_file)

        # Add all available anchors and their positions
        for a_name, a_pos in desc_meas['description']['anchors'].items():
            if len(a_pos) != 3:
                print("-E-: [UneTwr::prepare_data] Invalid anchor position")
                return False

            # Add the new anchor into the dictionary
            self.add_anchor(a_name, a_pos[0], a_pos[1], a_pos[2])

        # Fill all measurements dictionary
        self.m_pp_meas = desc_meas['measurements']
        self.m_pp_meas_num = 0

    def add_anchor(self, key, x, y, z):
        """ Add new anchor with position """
        self.m_anchors[key] = [x, y, z]

    def get_anchor_pos(self, key):
        """ Get anchor position

            Get position [x, y, z] of anchor by key relative origin
        """
        return self.m_anchors[key]

    def get_dist(self, p1, p2):
        """ Get distance between two points [x, y, z] """
        dX = p1[self.m_X] - p2[self.m_X]
        dY = p1[self.m_Y] - p2[self.m_Y]
        dZ = p1[self.m_Z] - p2[self.m_Z]

        return np.sqrt(dX ** 2 + dY ** 2 + dZ ** 2)

    def get_pos_and_dist(self, key, p2):
        """ Get position of anchor by key and distance between anchor and point [x, y, z] """
        p1 = [
            self.m_anchors[key][self.m_X],
            self.m_anchors[key][self.m_Y],
            self.m_anchors[key][self.m_Z]
        ]

        dX = p1[self.m_X] - p2[self.m_X]
        dY = p1[self.m_Y] - p2[self.m_Y]
        dZ = p1[self.m_Z] - p2[self.m_Z]

        return [dX, dY, dZ, self.get_dist(p1, p2)]

    def add_tag_meas(self, key_anchor, meas, snr=-128):
        """ Add tag measurement """
        self.m_tag_meas[key_anchor] = [meas, snr]

    def get_tag_measurement(self):
        """ Get tag measurements """
        return self.m_tag_meas

    def get_tag_pvt(self):
        """ Get current tag position/velocity and error of time """
        return self.m_tag_pvt

    def upd_tag_pvt(self, dx, dy, dz):
        """ Update current tag position/velocity and error of time """
        self.m_tag_pvt[self.m_X] += dx
        self.m_tag_pvt[self.m_Y] += dy
        self.m_tag_pvt[self.m_Z] += dz

    def set_tag_pvt(self, x, y, z):
        """ Set tag position """
        self.m_tag_pvt = [x, y, z]

    def clr_anchor(self, key, meas, snr=-128):
        """ Clear anchors dictionary """
        self.m_anchors.clear()

    def clr_tag_meas(self):
        """ Clear tag measurement (used before add measurements for new epoch) """
        self.m_tag_meas.clear()

    def get_position(self):
        """ Comment... """

        tag_state_vector = list()

        if self.m_nav_method == UneNavMethod.lse:
            tag_state_vector = self.get_position_lse()
        elif self.m_nav_method == UneNavMethod.weighted_lse:
            tag_state_vector = self.get_position_wlse()
        elif self.m_nav_method == UneNavMethod.Kalman:
            tag_state_vector = self.get_position_kalman()

        return tag_state_vector

    def get_position_lse(self):
        """ Method to compute tag position using system of the linear equations
            y = H*x -> x = inv(Ht * H) * Ht * y
            where:
            y - measurement vector
            H - geometry matrix
            x - tag state vector (position, velocity,...)
         """
        num_anchors = len(self.m_anchors)
        num_max_iteration = 9
        flg_stop_computation = False

        if num_anchors < self.m_anchor_min:
            print("-E-: [UneTwr::get_position_lse] Not enough anchors")
            return [0, 0, 0]

        # Get distances between anchors and tag position for current epoch
        meas_curr = self.get_tag_measurement()

        # List to save tag position
        p_curr = []

        # Iterative (9 attempts maximum recommended) computation of tag position by using LSE method
        while (num_max_iteration > 0) and (flg_stop_computation is False):

            num_max_iteration -= 1

            # Get tag position attempted on the previous step
            tag_pos_prev = self.get_tag_pvt()

            mtx_h = []
            mtx_y = []
            mtx_x = np.array(tag_pos_prev)

            for a_name, a_meas in meas_curr.items():

                # Get distance between anchor and tag position for previous computation step
                [x, y, z, d_prev] = self.get_pos_and_dist(a_name, tag_pos_prev)

                # Get distances between anchors and tag position for current epoch
                d_curr = a_meas[self.m_ind_dist]

                # Get difference between previous and current distances to compute error in current user position
                d_err = d_prev - d_curr

                # Filling geometry and measurements matrices to
                # compute user position and time state vector [dx, dy, dz]
                if len(mtx_h) == 0:
                    mtx_h = np.array([[x/d_prev, y/d_prev, z/d_prev]])
                    mtx_y = np.array([d_err])
                else:
                    mtx_h = np.append(mtx_h, [[x/d_prev, y/d_prev, z/d_prev]], axis=0)
                    mtx_y = np.append(mtx_y, [d_err])

            # Using the LSE method for estimating user state vector: mtx_x = [dx, dy, dz]
            mtx_h_trp = mtx_h.transpose()
            mtx_k = np.matmul(inv(np.matmul(mtx_h_trp, mtx_h)), mtx_h_trp)
            mtx_x = np.matmul(mtx_k, mtx_y)

            # Update tag position using estimated errors in mtx_k
            p_prev = [
                tag_pos_prev[self.m_X],
                tag_pos_prev[self.m_Y],
                tag_pos_prev[self.m_Z]
            ]

            self.upd_tag_pvt(mtx_x[self.m_X], mtx_x[self.m_Y], mtx_x[self.m_Z])

            # Get tag position attempted on the current step
            tag_pos_curr = self.get_tag_pvt()

            p_curr = [
                tag_pos_curr[self.m_X],
                tag_pos_curr[self.m_Y],
                tag_pos_curr[self.m_Z]
            ]

            # Compute error between previous and current estimate of position
            # and if it less than threshold then stop computation
            if self.get_dist(p_prev, p_curr) <= self.m_poserr_trsh:
                flg_stop_computation = True

            # Go to next iteration step (above) ^^^

        # If tag position was successfully computed
        if flg_stop_computation is True:
            return p_curr
        else:
            print("-E-: [UneTwr::get_position_lse] Position isn't available")
            return p_curr

    def get_position_wlse(self):
        """ Comment... """
        return [0, 0, 0]

    def get_position_kalman(self):
        """ Comment... """
        return [0, 0, 0]
