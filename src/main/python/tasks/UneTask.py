"""@package UneTask

UNE - UWB Navigation Engine.

UNE is used to compute the tag PVT (position/velocity/time) vector using different algorithms for it. UNE implements
algorithms such as LSE, WLSE, Kalman Filter, Fusion filter based on Kalman Filter in which are used
measurements form the tag IMU subsystem.

There're two main classes in this module:
1. UneTag class that contains properties, methods to describe and control Tag. This class allows to add a unique set of
anchors for a tag, setting/getting different flags, and it allows to get the information about the PVT tag,
residuals and state. Also you can meas how long the PVT for tag is computed using methods
tic (at the start) and toc (in the end).

2. Une class implements algorithms for computing tag's PVT vector such as LSE, WLSE, Kalman Filter, Fusion filter,
algorithms to control integrity and fault detection such as RAIM/FDE, to estimate dilution of precision
(HDOP, VDOP, PDOP). This class also implements different flags, e.g. flag UneFlags.compute_is_done that is set when the
computation has performed for all tags.
"""

from PyQt5.QtCore import QThread, pyqtSignal
import atexit
import logging as log
from enum import Enum
import time
import json
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv


class UneConstant:
    """ This class contains different constant values
        which are used in main class of UNE
    """

    @staticmethod
    def epoch_period_const():
        """ Method for implementing constant value
            of epoch period in seconds
        """
        return 0.1

    @staticmethod
    def meas_min_const():
        """ Method for implementing constant value
            of minimum number of measurements for
            computing tag position (It's recommended 4)
        """
        return 4

    @staticmethod
    def lse_max_iter_const():
        """ Method for implementing constant value
            of maximum number of iteration for LSE method
        """
        return 9


class UneErrors(Enum):
    """ Enumerator class describing errors in the UNE module """
    none = 0
    not_enough_meas = 1
    too_many_iteration = 2


class UneFlags(Enum):
    """ Enumerator class describing all flags used in the UNE module """
    noname_flag      = 1 #
    get_new_pvt      = 2 # Set this flag after adding tag measurements to start PVT computation
    new_pvt_is_ready = 3 # If set, a new PVT is available
    compute_is_done  = 4 # If set, computation for all tags has been done


class UneNavMethod(Enum):
    """ Enumerator class describing different navigation
        methods for the UNE module """
    lse = 1
    weighted_lse = 2
    kalman = 3


class UneTag:
    """ Class that contains methods, properties and other fields
        to describe states of tag and to control it
    """
    # Properties
    m_name      = ''              # Unique tag name
    m_pvt_prev  = [0, 0, 0]       # Previous tag Position/Velocity/Time
    m_pvt_curr  = [0, 0, 0]       # Current tag Position/Velocity/Time
    m_residuals = list()          # Residual vector
    m_meas_prev = dict()          # Previous epoch measurements between tag and anchors -> dictionary {name: [distance, snr, ...], ...}
    m_meas_curr = dict()          # Current epoch measurements between tag and anchors -> dictionary {name: [distance, snr, ...], ...}
    m_anchors   = dict()          # Anchors for this tag -> dictionary {name: [x, y, z], ...}
    m_err_code  = UneErrors.none  # Error code
    m_time_for_compute = 0        # To meas time that is required for compute PVT
    m_pdop      = 0               # Position dilution of precision
    m_hdop      = 0               # Horizontal dilution of precision
    m_vdop      = 0               # Vertical dilution of precision

    # Auxiliary variables
    m_X = 0
    m_Y = 1
    m_Z = 2
    m_D = 3
    m_T = 3

    # Flags
    m_flg_get_new_pvt = False
    m_flg_got_new_pvt = False

    def __init__(self, name):
        """ The constructor """
        self.m_name = name

    def get_name(self):
        """ Get tag name """
        return self.m_name

    def add_meas(self, key_anchor, meas, snr=-128):
        """ Add tag measurement """
        self.m_meas_curr[key_anchor] = [meas, snr]
        return

    def get_meas(self):
        """ Get tag measurements """
        return self.m_meas_curr

    def clr_meas(self):
        """ Clear tag measurement (used before add measurements for new epoch) """
        self.m_meas_prev = self.m_meas_curr.copy()
        self.m_pvt_prev = self.m_pvt_curr.copy()
        self.m_meas_curr.clear()
        return

    def get_pvt(self):
        """ Get current tag position/velocity and error of time """
        return self.m_pvt_curr

    def upd_pvt(self, dx, dy, dz):
        """ Update current tag position/velocity and error of time """
        self.m_pvt_curr[self.m_X] += dx
        self.m_pvt_curr[self.m_Y] += dy
        self.m_pvt_curr[self.m_Z] += dz
        return

    def set_pvt(self, x, y, z):
        """ Set tag position """
        self.m_pvt_curr = [x, y, z]
        return

    def set_dops(self, h=100, v=100, p=100):
        """ Set DOP factors """
        self.m_hdop = h
        self.m_vdop = v
        self.m_pdop = p

    def get_dops(self):
        """ Get DOP factors of tag """
        return [self.m_hdop, self.m_vdop, self.m_pdop]

    def set_residuals(self, r):
        """ Set residual vector for tag
            and automatically calculate
            DOP factors
        """
        self.m_residuals = list()
        self.m_residuals = r.copy()

    def get_residuals(self):
        """ Get residual vector of tag """
        return self.m_residuals.copy()

    def set_flags(self, flg, val=True):
        """ Method to set flags """
        if (val is not True) and (val is not False):
            print('-E- [UneTag::setFlags::tag: {}] Incorrect argument value: {}'.format(self.m_name, val))

        if flg is UneFlags.get_new_pvt:
            self.m_flg_get_new_pvt = val
        else:
            print('-E- [UneTag::setFlags::tag: {}] Flag {} not supported'.format(self.m_name, flg))

    def set_int_flags(self, flg, val=True):
        """ Method to set flags.
            It's used only in UNE class.
            You shouldn't set/reset any flags.
        """
        if (val is not True) and (val is not False):
            print('-E- [UneTag::_setFlags::tag: {}] Incorrect argument value: {}'.format(self.m_name, val))

        if flg is UneFlags.new_pvt_is_ready:
            self.m_flg_got_new_pvt = val
        else:
            print('-E- [UneTag::_setFlags::tag: {}] Flag {} not supported'.format(self.m_name, flg))

    def get_flags(self, flg):
        """ Method to get flags """
        if flg is UneFlags.get_new_pvt:
            res = self.m_flg_get_new_pvt
            # Reset flag after request
            self.m_flg_get_new_pvt = False
            return res
        elif flg is UneFlags.new_pvt_is_ready:
            res = self.m_flg_got_new_pvt
            # Reset flag after request
            self.m_flg_got_new_pvt = False
            return res
        else:
            print('-E- [UneTag::getFlags::tag_{}] Flag {} not supported'.format(self.m_name, flg))

    def set_err_code(self, val=UneErrors.none):
        """ Method to set error code.
            It's used only in UNE class.
            You shouldn't call this method.
        """
        self.m_err_code = val

    def get_err_code(self):
        """ Method to get error code """
        return self.m_err_code

    def add_anchor(self, key, x, y, z):
        """ Add new anchor with position """
        self.m_anchors[key] = [x, y, z]

    def clr_anchor(self, key, meas, snr=-128):
        """ Clear anchors dictionary """
        self.m_anchors.clear()

    def get_anchors(self):
        """ Get anchors dictionary """
        return self.m_anchors

    def get_anchor(self, key):
        """ Get anchor by key """
        if key in self.m_anchors:
            return self.m_anchors[key]
        else:
            print("-E-: [Une::get_anchor] Anchor dictionary doesn't contain {} key".format(key))

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

    def tic(self):
        """ Method to start meas time of computation of PVT """
        self.m_time_for_compute = time.time()

    def toc(self):
        """ Method to stop meas time of computation of PVT.
            Return time in ms
        """
        self.m_time_for_compute = time.time() - self.m_time_for_compute

        return self.m_time_for_compute * 1000.0


class Une:
    """ Class that contains methods, properties and other fields
        to describe states of tag and to control it
    """
    # Properties
    m_nav_method = UneNavMethod.lse

    # Auxiliary variables
    m_X = 0
    m_Y = 1
    m_Z = 2
    m_D = 3
    m_T = 3
    m_num_of_pvt_prm = 3    # 3 parameters now: [x, y, z]
    m_pos_err = 0.001       # Error threshold between the previous and current computation steps to stop computation [m]

    # Flags
    m_flg_compute_is_done = False

    # Indices of measurements
    m_ind_dist = 0
    m_ind_cnr = 1

    # List of tags
    m_tags = list()

    def __init__(self, nav_method):
        """ The constructor.
            To choose nav_method you should use
            the UneNavMethod enumerator.
        """
        self.m_nav_method = nav_method

    def add_tag(self, name):
        """ Method to add new tag """
        for tag in self.m_tags:
            if name == tag.get_name():
                print('-W- [Une::add_new_tag] Tag {} has already added'.format(name))
                return

        self.m_tags.append(UneTag(name))

        # Return current tag object
        return self.m_tags[-1]

    def get_tag(self, name):
        """ Method to get one tag object """
        for tag in self.m_tags:
            if name == tag.get_name():
                return tag

    def get_tags(self):
        """ Method to get tags list """
        return self.m_tags

    def rem_tag(self, name):
        """ Method to remove tag from tags list """
        for tag in self.m_tags:
            if name == tag.get_name():
                self.m_tags.remove(tag)

    def clr_tags(self):
        """ Method to clear tags list """
        self.m_tags.clear()

    def get_position(self):
        """ Comment... """

        # Get PVT for all tags in the list m_tags
        for tag in self.m_tags:
            # If need to compute new tag position
            if tag.get_flags(UneFlags.get_new_pvt) is True:

                tag.tic()

                if self.m_nav_method == UneNavMethod.lse:
                    self.get_position_lse(tag)
                elif self.m_nav_method == UneNavMethod.weighted_lse:
                    self.get_position_wlse(tag)
                elif self.m_nav_method == UneNavMethod.Kalman:
                    self.get_position_kalman(tag)

                print('Computation time of the PVT for tag: {} is equal {:5.3f} ms'.format(tag.get_name(), tag.toc()))

                # Clear tag measurements
                tag.clr_meas()

        self.set_flags(UneFlags.compute_is_done, True)

    def get_position_lse(self, tag):
        """ Method to compute tag position using system of the linear equations
            y = H*x -> x = inv(Ht * H) * Ht * y
            where:
            y - measurement vector
            H - geometry matrix
            x - tag state vector (position, velocity,...)
         """
        num_anchors = len(tag.get_anchors())
        num_max_iteration = UneConstant.lse_max_iter_const()
        flg_stop_computation = False

        if num_anchors < UneConstant.meas_min_const():
            print("-E-: [UneTwr::get_position_lse] Not enough anchors for tag: {}".format(tag.get_name()))
            tag.set_pvt(0, 0, 0)
            tag.set_err_code(UneErrors.not_enough_meas)
            return

        # Get distances between anchors and tag position for current epoch
        meas_curr = tag.get_meas().items()

        # Tag state vector
        mtx_x = np.zeros(self.m_num_of_pvt_prm)

        # Geometry matrix
        mtx_g = np.zeros((num_anchors, 3))

        # Measurement vector
        mtx_y = np.zeros(num_anchors)

        # Residual vector
        mtx_r = np.zeros(num_anchors)

        # Matrix to compute DOPs
        mtx_q = []

        # Iterative (9 attempts maximum recommended) computation of tag position by using LSE method
        while (num_max_iteration > 0) and (flg_stop_computation is False):

            # Allocate memory for matrices
            mtx_g = np.zeros((num_anchors, 3))
            mtx_y = np.zeros(num_anchors)

            num_max_iteration -= 1

            # Get tag position got on the previous step
            tag_pos_prev = tag.get_pvt()

            cnt_meas = 0
            for a_name, a_meas in meas_curr:

                # Get distance between anchor and tag position for previous computation step
                [x, y, z, d_prev] = tag.get_pos_and_dist(a_name, tag_pos_prev)

                # Get distances between anchors and tag position for current epoch
                d_curr = a_meas[self.m_ind_dist]

                # Get difference between previous and current distances to compute error in current user position
                d_err = d_prev - d_curr

                # Filling geometry and measurements matrices to
                # compute user position and time state vector [dx, dy, dz]
                mtx_g[cnt_meas] = [x / d_prev, y / d_prev, z / d_prev]
                mtx_y[cnt_meas] = d_err
                cnt_meas += 1

            # Using the LSE method for estimating user state vector: mtx_x = [dx, dy, dz]
            mtx_g_trp = mtx_g.transpose()
            mtx_q = inv(np.matmul(mtx_g_trp, mtx_g))
            mtx_s = np.matmul(mtx_q, mtx_g_trp)
            mtx_x = np.matmul(mtx_s, mtx_y)

            # Update tag position using estimated errors in mtx_k
            p_prev = [
                tag_pos_prev[self.m_X],
                tag_pos_prev[self.m_Y],
                tag_pos_prev[self.m_Z]
            ]

            tag.upd_pvt(mtx_x[self.m_X], mtx_x[self.m_Y], mtx_x[self.m_Z])

            # Get tag position attempted on the current step
            tag_pos_curr = tag.get_pvt()

            # List to save tag position
            p_curr = [
                tag_pos_curr[self.m_X],
                tag_pos_curr[self.m_Y],
                tag_pos_curr[self.m_Z]
            ]

            # Computation of the error between the previous and current position estimates
            # and if it less than threshold then stop computation
            if tag.get_dist(p_prev, p_curr) <= self.m_pos_err:
                flg_stop_computation = True

            # Go to the next iteration step (above) ^^^

        # Get residuals vector
        mtx_yr = np.matmul(mtx_g, mtx_x)
        mtx_r = np.subtract(mtx_y, mtx_yr)
        tag.set_residuals(mtx_r)

        # Get DOP factors
        pdop = mtx_q.trace()
        tag.set_dops(100, 100, pdop)

        # If tag position was successfully computed
        if flg_stop_computation is True:
            tag.set_int_flags(UneFlags.new_pvt_is_ready, True)
            tag.set_err_code(UneErrors.none)
        else:
            print("-E-: [Une::get_position_lse] Too many iteration for tag {}".format(tag.get_name()))
            # pos = ...
            tag.set_err_code(UneErrors.too_many_iteration)

    def get_position_wlse(self, pos):
        """ Comment... """
        pos = [0, 0, 0]
        return UneErrors.none

    def get_position_kalman(self, pos):
        """ Comment... """
        pos = [0, 0, 0]
        return UneErrors.none

    def set_flags(self, flg, val=True):
        """ Method to set flags """
        if (val is not True) and (val is not False):
            print('-E- [Une::set_flags] Incorrect argument value: {}'.format(val))

        if flg is UneFlags.compute_is_done:
            self.m_flg_compute_is_done = val
        else:
            print('-E- [Une::set_flags] Flag {} not supported'.format(flg))

    def get_flags(self, flg):
        """ Method to get flags """
        if flg is UneFlags.compute_is_done:
            res = self.m_flg_compute_is_done
            # Reset flag after request
            self.m_flg_compute_is_done = False
            return res
        else:
            print('-E- [UneTwrTask::get_flags] Flag {} not supported'.format(flg))


class UneTask(QThread):
    """ Documentation for a class.

        More details.
    """
    # !!! For test
    m_pp_meas = dict()

    # UWB Navigation Engine object
    une = None

    def __init__(self, nav_method):
        """ The constructor.
            To choose nav_method you should use
            the UneNavMethod enumerator.
        """
        # Init UNE object
        self.une = Une(nav_method)

        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

    def run(self):
        """ Documentation for a method
            ..........................
        """

        my_tag = self.une.add_tag('My tag')

        # !!! For test. Adding anchors for this tag
        self.prepare_data(my_tag)
        x, y, z = [], [], []
        flg_compute = True
        while True:
            if flg_compute is True:
                flg_compute = False
                for epoch in self.m_pp_meas:
                    print('-I- [UneTask::run] Compute tag PVT. UTC:', epoch)

                    # Adding measurement for this tag for current epoch
                    for a_name, a_meas in self.m_pp_meas[epoch].items():
                        dist = a_meas[0]
                        cnr = a_meas[1]
                        my_tag.add_meas(a_name, dist, cnr)

                    # Set flag to start computation of the PVT
                    my_tag.set_flags(UneFlags.get_new_pvt, True)

                    # Compute position
                    self.une.get_position()

                    if self.une.get_flags(UneFlags.compute_is_done):
                        err = my_tag.get_err_code()
                        if err is not UneErrors.none:
                            print('-I- [UneTask::run] Error {} during compute PVT for tag: {}'.format(err, my_tag.get_name()))
                        elif my_tag.get_flags(UneFlags.new_pvt_is_ready):
                            [px, py, pz] = my_tag.get_pvt()
                            x.append(px)
                            y.append(py)
                            z.append(pz)

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

        # t = time.time()
        # while True:
        #     if time.time() - t > UneConstant.epoch_period_const():
        #         t = time.time()
        #
        #         # Compute position
        #         self.une.get_position()

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

    def prepare_data(self, tag):
        """ !!! Test method. Delete later.
            Load description and measurements from JSON file
            for postprocessing
        """
        # Load data for postprocessing
        with open("logs/my_new_log.json", "r") as read_file:
            desc_meas = json.load(read_file)

        # Add all available anchors and their positions
        for a_name, a_pos in desc_meas['description']['anchors'].items():
            if len(a_pos) != 3:
                print("-E-: [UneTwr::prepare_data] Invalid anchor position")
                return False

            # Add the new anchor into the dictionary
            tag.add_anchor(a_name, a_pos[0], a_pos[1], a_pos[2])

        # Fill all measurements dictionary
        self.m_pp_meas = desc_meas['measurements']
