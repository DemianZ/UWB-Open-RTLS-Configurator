"""@package UneTwrTask
UNE for Two Way Ranging approach

UNE - UWB Navigation Engine
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
from time import monotonic
from accessify import private


class UneConstant:
    """ This class contains different constant values
        which are used in main class of UNE
    """

    @private
    @staticmethod
    def epoch_period_const():
        """ Method for implementing constant value
            of epoch period in seconds
        """
        return 0.1

    @private
    @staticmethod
    def meas_min_const():
        """ Method for implementing constant value
            of minimum number of measurements for
            computing tag position (It's recommended 4)
        """
        return 4

    @private
    @staticmethod
    def lse_max_iter_const():
        """ Method for implementing constant value
            of maximum number of iteration for LSE method
        """
        return 9

    # List of a constants
    EPOCH_PERIOD = epoch_period_const()
    MEAS_NUM_MIN = meas_min_const()
    LSE_MAX_ITER = lse_max_iter_const()


class UneErrors(Enum):
    """ Enumerator class describing errors in the UNE module """
    none = 0
    not_enough_meas = 1
    too_many_iteration = 2


class UneFlags(Enum):
    """ Enumerator class describing all flags used in the UNE module """
    new_tag_meas = 1
    get_new_pos = 2


class UneNavMethod(Enum):
    """ Enumerator class describing different navigation
        methods for the UNE module """
    lse = 1
    weighted_lse = 2
    kalman = 3


class UneTwrTask(QThread):
    """Documentation for a class.

    More details.
    """
    # Flags
    m_flg_compute_pos = False

    # Auxiliary variables
    m_X = 0
    m_Y = 1
    m_Z = 2
    m_D = 3
    m_T = 3
    m_nav_method = 0
    m_pos_err_threshold = 0.001

    # Indices of measurements
    m_ind_dist = 0
    m_ind_cnr = 1

    m_anchors = dict()
    m_tag_meas = dict()
    m_tag_pvt = [0, 0, 0]

    # For postprocessing
    # m_pp_meas = dict()
    # m_pp_meas_num = 0

    def __init__(self, nav_method):
        """ The constructor.
            To choose nav_method you should use
            the UneNavMethod enumerator
        """
        self.m_nav_method = nav_method
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

    def run(self):
        """ Documentation for a method
            ..........................
        """
        # Reset tag PVT
        self.set_tag_pvt(0, 0, 0)

        # Prepare auxiliary variables
        pos = [0, 0, 0]
        err_code = UneErrors.none

        t = monotonic()
        while True:
            if monotonic() - t > UneConstant.EPOCH_PERIOD:
                t = monotonic()

                # If need to compute new tag position
                if self.get_flags(UneFlags.get_new_pos) is True:

                    # Reset the flag
                    self.set_flags(UneFlags.get_new_pos, False)

                    # Compute position
                    err_code = self.get_position(pos)

                    # Clear previous tag measurement list
                    self.clr_tag_meas()

                    # Send message to another thread
                    print('Error: {}\t\tPosition: {}, {}, {}'.format(err_code, pos[0], pos[1], pos[2]))

        # self.prepare_data()
        # x, y, z = [], [], []
        # flg_compute = True
        # while True:
        #     if flg_compute is True:
        #
        #         flg_compute = False
        #
        #         for epoch in self.m_pp_meas:
        #             print('-I- [UneTwr::run] Compute tag PVT. UTC:', epoch)
        #
        #             # Forming tag measurement dictionary for current epoch
        #             self.clr_tag_meas()
        #             for a_name, a_meas in self.m_pp_meas[epoch].items():
        #                 dist = a_meas[self.m_ind_dist]
        #                 cnr = a_meas[self.m_ind_cnr]
        #                 self.add_tag_meas(a_name, dist, cnr)
        #
        #             res = [0, 0, 0]
        #             err_code = self.get_position(res)
        #             x.append(res[0])
        #             y.append(res[1])
        #             z.append(res[2])
        #
        #             time.sleep(0.05)
        #
        #         fig = plt.figure()
        #         ax = fig.add_subplot(111, projection='3d')
        #         ax.plot_wireframe(x, y, np.array([z, z]), rstride=10, cstride=10)
        #         ax.set_xlim3d(min(x) - 2, max(x) + 2)
        #         ax.set_ylim3d(min(y) - 2, max(y) + 2)
        #         ax.set_zlim3d(min(z) - 2, max(z) + 2)
        #         ax.set_xlabel('X axis, m')
        #         ax.set_ylabel('Y axis, m')
        #         ax.set_zlabel('Z axis, m')
        #         plt.show()
        #
        #     time.sleep(30)

    def set_flags(self, flg, val=True):
        """ Method to set flags """
        if (val is not True) or (val is not False):
            print('-E- [UneTwrTask::setFlags] Incorrect argument value: {}'.format(val))

        if flg is UneFlags.get_new_pos:
            self.m_flg_compute_pos = val
        else:
            print('-E- [UneTwrTask::setFlags] Flag {} not supported'.format(flg))

    def get_flags(self, flg):
        """ Method to get flags """
        if flg is UneFlags.get_new_pos:
            return self.m_flg_compute_pos
        else:
            print('-E- [UneTwrTask::getFlags] Flag {} not supported'.format(flg))

    # def prepare_data(self):
    #     """ !!! Test method. Delete later.
    #         Load description and measurements from JSON file
    #         for postprocessing
    #     """
    #     # Load data for postprocessing
    #     with open("logs/my_new_log.json", "r") as read_file:
    #         desc_meas = json.load(read_file)
    #
    #     # Add all available anchors and their positions
    #     for a_name, a_pos in desc_meas['description']['anchors'].items():
    #         if len(a_pos) != 3:
    #             print("-E-: [UneTwr::prepare_data] Invalid anchor position")
    #             return False
    #
    #         # Add the new anchor into the dictionary
    #         self.add_anchor(a_name, a_pos[0], a_pos[1], a_pos[2])
    #
    #     # Fill all measurements dictionary
    #     self.m_pp_meas = desc_meas['measurements']
    #     self.m_pp_meas_num = 0

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

    def get_position(self, pos):
        """ Comment... """
        err_code = UneErrors.none

        if self.m_nav_method == UneNavMethod.lse:
            err_code = self.get_position_lse(pos)
        elif self.m_nav_method == UneNavMethod.weighted_lse:
            err_code = self.get_position_wlse(pos)
        elif self.m_nav_method == UneNavMethod.Kalman:
            err_code = self.get_position_kalman(pos)

        return err_code

    def get_position_lse(self, pos):
        """ Method to compute tag position using system of the linear equations
            y = H*x -> x = inv(Ht * H) * Ht * y
            where:
            y - measurement vector
            H - geometry matrix
            x - tag state vector (position, velocity,...)
         """
        num_anchors = len(self.m_anchors)
        num_max_iteration = UneConstant.LSE_MAX_ITER
        flg_stop_computation = False

        if num_anchors < UneConstant.MEAS_NUM_MIN:
            print("-E-: [UneTwr::get_position_lse] Not enough anchors")
            pos[0] = 0
            pos[1] = 0
            pos[2] = 0
            return UneErrors.not_enough_meas

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
                    mtx_h = np.array([[x / d_prev, y / d_prev, z / d_prev]])
                    mtx_y = np.array([d_err])
                else:
                    mtx_h = np.append(mtx_h, [[x / d_prev, y / d_prev, z / d_prev]], axis=0)
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
            if self.get_dist(p_prev, p_curr) <= self.m_pos_err_threshold:
                flg_stop_computation = True

            # Go to next iteration step (above) ^^^

        # If tag position was successfully computed
        if flg_stop_computation is True:
            pos[0] = p_curr[0]
            pos[1] = p_curr[1]
            pos[2] = p_curr[2]
            return UneErrors.none
        else:
            print("-E-: [UneTwr::get_position_lse] Too many iteration")
            # pos = ...
            return UneErrors.too_many_iteration

    def get_position_wlse(self, pos):
        """ Comment... """
        pos = [0, 0, 0]
        return UneErrors.none

    def get_position_kalman(self, pos):
        """ Comment... """
        pos = [0, 0, 0]
        return UneErrors.none
