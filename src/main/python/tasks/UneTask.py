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

from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
import atexit
import logging as log
from enum import Enum
import time
import json
import numpy as np
from numpy.linalg import inv
import math
from pympler import asizeof


class UneConst:
    """ This class contains different constant values
        which are used in main class of UNE
    """

    _epoch_period = 0.001
    _meas_min = 3
    _lse_max_iter = 9
    _pvt_length = 3
    _X = 0
    _Y = 1
    _Z = 2
    _D = 3
    _T = 3
    _XV = 4
    _YV = 5
    _ZV = 6
    _NV = 7

    @property
    def epoch_period(self):
        """ Epoch period in seconds """
        return self._epoch_period

    @property
    def meas_min(self):
        """ Minimum number of measurements for computing tag position (It's recommended 4) """
        return self._meas_min

    @property
    def lse_max_iter(self):
        """ Maximum number of iteration for LSE method """
        return self._lse_max_iter

    @property
    def pvt_length(self):
        """ Length of PVT vector """
        return self._pvt_length

    @property
    def x(self):
        """ Array index. Tag position X axis """
        return self._X

    @property
    def y(self):
        """ Array index. Tag position Y axis """
        return self._Y

    @property
    def z(self):
        """ Array index. Tag position Z axis """
        return self._Z

    @property
    def t(self):
        """ Array index. Tag time error """
        return self._T

    @property
    def xv(self):
        """ Array index. Tag velocity along X axis """
        return self._XV

    @property
    def yv(self):
        """ Array index. Tag velocity along Y axis """
        return self._YV

    @property
    def zv(self):
        """ Array index. Tag velocity along Z axis """
        return self._ZV

    @property
    def nv(self):
        """ Array index. Euclidean norm of velocity vector """
        return self._NV


class UneErrors(Enum):
    """ Enumerator class describing errors in the UNE module """
    none = 0
    not_enough_meas = 1
    too_many_iteration = 2
    linal_error = 3


class UneFlags(Enum):
    """ Enumerator class describing all flags used in the UNE module """
    noname_flag = 1       #
    get_new_pvt = 2       # Set this flag after adding tag measurements to start PVT computation
    new_pvt_is_ready = 3  # If set, a new PVT is available
    compute_is_done = 4   # If set, computation for all tags has been done
    tag_extend_info = 5   # Flag for tag to send adding some information to PVT: DOP


class UneNavMethod(Enum):
    """ Enumerator class describing different navigation
        methods for the UNE module """
    lse = 1
    weighted_lse = 2
    fusion = 3


class UneTag:
    """ Class that contains methods, properties and other fields
        to describe states of tag and to control it
    """
    # Tag properties
    m_name = ''                            # Unique tag name
    m_pvt_prev = [0, 0, 0, 0, 0, 0, 0, 0]  # Previous tag Position/Velocity/Time
    m_pvt_curr = [0, 0, 0, 0, 0, 0, 0, 0]  # Current tag Position/Velocity/Time
    m_residuals = list()                   # Residual vector
    m_meas_prev = dict()                   # Previous epoch measurements between tag and anchors
    m_meas_curr = dict()                   # Current epoch measurements between tag and anchors
    m_anchors = dict()                     # Anchors for this tag -> dictionary {name: [x, y, z], ...}
    m_err_code = UneErrors.none            # Error code
    m_time_for_compute = 0                 # To meas time that is required for compute PVT
    m_pdop = 0                             # Position dilution of precision
    m_hdop = 0                             # Horizontal dilution of precision
    m_vdop = 0                             # Vertical dilution of precision
    m_epoch_prev = 0                       # Time of previous epoch (UTC with ms)
    m_epoch_curr = 0                       # Time of current epoch (UTC with ms)

    # Flags
    m_flg_get_new_pvt = False
    m_flg_got_new_pvt = False
    m_flg_extend_info = False

    # Auxiliary variables
    const = UneConst()

    def __init__(self, name):
        """ The constructor """
        self.m_name = name

    def get_name(self):
        """ Get tag name """
        return self.m_name

    def set_epoch_curr(self, t):
        """ Set time of previous epoch """
        self.m_epoch_curr = t

    def add_meas(self, key_anchor, meas, snr=-128):
        """ Add tag measurement """
        self.m_meas_curr[key_anchor] = [meas, snr]
        return

    def get_meas_curr(self):
        """ Get tag measurements for current epoch """
        return self.m_meas_curr

    def get_meas_prev(self):
        """ Get tag measurements for previous epoch """
        return self.m_meas_prev

    def clr_meas(self):
        """ Clear tag measurement (it's used before add measurements for new epoch) """
        self.m_meas_prev = self.m_meas_curr.copy()
        self.m_pvt_prev = self.m_pvt_curr.copy()
        self.m_meas_curr.clear()
        return

    def get_pvt(self):
        """ Get current tag position/velocity and error of time """
        return self.m_pvt_curr

    def upd_pos(self, dx, dy, dz, dt):
        """ Update current tag position and time error """
        self.m_pvt_curr[self.const.x] += dx
        self.m_pvt_curr[self.const.y] += dy
        self.m_pvt_curr[self.const.z] += dz
        self.m_pvt_curr[self.const.t] += dt
        return

    def upd_vel(self, p):
        """ Update current tag velocity
            p1 - current position
        """
        if self.m_epoch_prev > 0:
            dt = self.m_epoch_curr - self.m_epoch_prev

            xv = (p[self.const.x] - self.m_pvt_prev[self.const.x]) / dt
            yv = (p[self.const.y] - self.m_pvt_prev[self.const.y]) / dt
            zv = (p[self.const.z] - self.m_pvt_prev[self.const.z]) / dt

            self.m_pvt_curr[self.const.xv] = xv
            self.m_pvt_curr[self.const.yv] = yv
            self.m_pvt_curr[self.const.zv] = zv
            self.m_pvt_curr[self.const.nv] = math.sqrt(xv ** 2 + yv ** 2 + zv ** 2)

        # Update previous epoch time
        self.m_epoch_prev = self.m_epoch_curr

        return

    def set_pvt(self, x, y, z, t, xv, yv, zv, nv):
        """ Set tag position """
        self.m_pvt_curr = [x, y, z, t, xv, yv, zv, nv]
        return

    def set_dops(self, h=100, v=100, p=100):
        """ Set DOP factors: HDOP, VDOP, PDOP """
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
            log.debug('-E- [UneTag::setFlags::tag: {}] Incorrect argument value: {}'.format(self.m_name, val))

        if flg is UneFlags.get_new_pvt:
            self.m_flg_get_new_pvt = val
        elif flg is UneFlags.tag_extend_info:
            self.m_flg_extend_info = val
        else:
            log.debug('-E- [UneTag::setFlags::tag: {}] Flag {} not supported'.format(self.m_name, flg))

    def set_int_flags(self, flg, val=True):
        """ Method to set flags.
            It's used only in UNE class.
            You shouldn't set/reset any flags.
        """
        if (val is not True) and (val is not False):
            log.debug('-E- [UneTag::_setFlags::tag: {}] Incorrect argument value: {}'.format(self.m_name, val))

        if flg is UneFlags.new_pvt_is_ready:
            self.m_flg_got_new_pvt = val
        else:
            log.debug('-E- [UneTag::_setFlags::tag: {}] Flag {} not supported'.format(self.m_name, flg))

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
        elif flg is UneFlags.tag_extend_info:
            return self.m_flg_extend_info
        else:
            log.debug('-E- [UneTag::getFlags::tag_{}] Flag {} not supported'.format(self.m_name, flg))

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
            log.debug("-E-: [Une::get_anchor] Anchor dictionary doesn't contain {} key".format(key))

    def get_anchor_pos(self, key):
        """ Get anchor position
            Get position [x, y, z] of anchor by key relative origin
        """
        return self.m_anchors[key]

    def get_dist(self, p1, p2):
        """ Get distance between two points [x, y, z] """
        dX = p1[self.const.x] - p2[self.const.x]
        dY = p1[self.const.y] - p2[self.const.y]
        dZ = p1[self.const.z] - p2[self.const.z]

        return np.sqrt(dX ** 2 + dY ** 2 + dZ ** 2)

    def get_pos_and_dist(self, key, p2):
        """ Get position of anchor by key and distance between anchor and point [x, y, z] """
        p1 = self.m_anchors[str(key)]

        dX = p1[self.const.x] - p2[self.const.x]
        dY = p1[self.const.y] - p2[self.const.y]
        dZ = p1[self.const.z] - p2[self.const.z]

        return [dX, dY, dZ, math.sqrt(dX ** 2 + dY ** 2 + dZ ** 2)]

    def tic(self):
        """ Method to start meas time of computation of PVT """
        self.m_time_for_compute = time.time_ns()

    def toc(self):
        """ Method for stopping the measurement of PVT computation time.
            Return time in ms
        """
        self.m_time_for_compute = time.time_ns() - self.m_time_for_compute

        return self.m_time_for_compute / 1000.0


class Une:
    """ Class that contains methods, properties and other fields
        to describe states of tag and to control it
    """
    # Properties
    m_nav_method = UneNavMethod.lse

    # Auxiliary variables
    m_num_of_pt_prm = 3     # 3 parameters now: [x, y, z]
    m_num_of_v_prm = 3      # 3 parameters: [xv, yv, zv]
    m_pos_err = 0.001       # Error threshold between the previous and current computation steps to stop computation [m]

    const = UneConst()

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

    def add_tag(self, tag_new):
        """ Method to add new tag """
        for tag in self.m_tags:
            if tag_new.get_name() == tag.get_name():
                log.debug('-W- [Une::add_new_tag] Tag {} has already added'.format(tag_new.get_name()))
                return -1

        self.m_tags.append(tag_new)

        # Return current tag object
        return 0

    def get_tag(self, name):
        """ Method to get one tag object """
        for tag in self.m_tags:
            if name == tag.get_name():
                return tag
        return None

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

                # tag.tic()

                if self.m_nav_method == UneNavMethod.lse:
                    self.get_position_lse(tag)
                elif self.m_nav_method == UneNavMethod.weighted_lse:
                    self.get_position_wlse(tag)
                elif self.m_nav_method == UneNavMethod.Kalman:
                    self.get_position_kalman(tag)

                # toc_t = tag.toc()
                # log.debug('Computation time of the PVT for tag: {} is equal {:5.3f} ms'.format(tag.get_name(), toc_t))

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
        num_max_iteration = self.const.lse_max_iter
        flg_stop_computation = False

        if num_anchors < self.const.meas_min:
            log.debug("-E-: [UneTwr::get_position_lse] Not enough anchors for tag: {}".format(tag.get_name()))
            tag.set_pvt(0, 0, 0)
            tag.set_err_code(UneErrors.not_enough_meas)
            return

        # Get distances between anchors and tag position for current epoch
        meas_curr = tag.get_meas_curr().items()
        meas_prev = tag.get_meas_prev()

        # Tag state vector
        mtx_x = np.zeros(self.m_num_of_pt_prm)

        # Tag state vector
        mtx_v = np.zeros(self.m_num_of_v_prm)

        # Geometry matrix
        mtx_g = np.zeros((num_anchors, 3))

        # Measurement vector for position
        mtx_y = np.zeros(num_anchors)

        # Residual vector
        mtx_r = np.zeros(num_anchors)

        # Matrices to compute DOPs and auxiliary matrix
        mtx_q = np.zeros((3, 3))
        mtx_s = np.zeros((3, num_anchors))

        # Iterative (9 attempts maximum recommended) computation of tag position by using LSE method
        while (num_max_iteration > 0) and (flg_stop_computation is False):

            # Clear matrices
            mtx_g.fill(0)
            mtx_y.fill(0)
            mtx_q.fill(0)
            mtx_s.fill(0)

            num_max_iteration -= 1

            # Get tag position got on the previous step
            tag_pvt_prev = tag.get_pvt()

            cnt_meas = 0
            for a_name, a_meas in meas_curr:
                # Get distance between anchor and tag position on previous computation step
                [x, y, z, d_prev] = tag.get_pos_and_dist(a_name, tag_pvt_prev)

                # Get difference between previous and current distances to compute error in current user position
                d_err = d_prev - a_meas[0]

                # Filling geometry and measurements matrices to
                # compute errors of user position, velocity and time [dx, dy, dz, dt] [dvx, dvy, dvz]
                mtx_g[cnt_meas] = [x / d_prev, y / d_prev, z / d_prev]
                mtx_y[cnt_meas] = d_err
                cnt_meas += 1

            # Using the LSE method for estimating user state vector: mtx_x = [dx, dy, dz]
            mtx_g_trp = mtx_g.transpose()
            try:
                mtx_q = inv(np.matmul(mtx_g_trp, mtx_g))
            except np.linalg.LinAlgError:
                tag.set_err_code(UneErrors.linal_error)
                return

            mtx_s = np.matmul(mtx_q, mtx_g_trp)
            mtx_x = np.matmul(mtx_s, mtx_y)

            # Update tag position using estimated errors in mtx_k
            p_prev = [
                tag_pvt_prev[self.const.x],
                tag_pvt_prev[self.const.y],
                tag_pvt_prev[self.const.z]
            ]

            tag.upd_pos(mtx_x[self.const.x], mtx_x[self.const.y], mtx_x[self.const.z], 0)

            # Get tag position attempted on the current step
            tag_pos_curr = tag.get_pvt()

            # List to save tag position
            p_curr = [
                tag_pos_curr[self.const.x],
                tag_pos_curr[self.const.y],
                tag_pos_curr[self.const.z]
            ]

            # Computation of the error between the previous and current position estimates
            # and if it less than threshold then stop computation
            if tag.get_dist(p_prev, p_curr) <= self.m_pos_err:
                flg_stop_computation = True

            # Go to the next iteration step (above) ^^^

        # Update tag velocity
        tag.upd_vel(p_curr)

        # Get residuals vector
        mtx_yr = np.matmul(mtx_g, mtx_x)
        mtx_r = np.subtract(mtx_y, mtx_yr)
        tag.set_residuals(mtx_r)

        # Get DOP factors
        p_dop = mtx_q.trace()
        h_dop = math.sqrt(mtx_q[0][0] ** 2 + mtx_q[1][1] ** 2)
        v_dop = math.sqrt(mtx_q[2][2] ** 2)
        tag.set_dops(h_dop, v_dop, p_dop)

        # Setting this flag to emit signal from UneTask
        tag.set_int_flags(UneFlags.new_pvt_is_ready, True)

        # If tag position was successfully computed
        if flg_stop_computation is True:
            tag.set_err_code(UneErrors.none)
        else:
            # log.debug("-E-: [Une::get_position_lse] Too many iteration for tag {}".format(tag.get_name()))
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
            log.debug('-E- [Une::set_flags] Incorrect argument value: {}'.format(val))

        if flg is UneFlags.compute_is_done:
            self.m_flg_compute_is_done = val
        else:
            log.debug('-E- [Une::set_flags] Flag {} not supported'.format(flg))

    def get_flags(self, flg):
        """ Method to get flags """
        if flg is UneFlags.compute_is_done:
            res = self.m_flg_compute_is_done
            # Reset flag after request
            self.m_flg_compute_is_done = False
            return res
        else:
            log.debug('-E- [UneTwrTask::get_flags] Flag {} not supported'.format(flg))


class UneTask(QThread):
    """ Documentation for a class.

        More details.
    """
    # Define API signals
    # This defines a signal that takes one argument: list of UneTag objects that were used in computation
    api_sig_new_pvt = pyqtSignal(list)

    # UWB Navigation Engine object
    une = None

    # Auxiliary variables
    const = UneConst()

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
        while True:
            self.msleep(int(self.const.epoch_period * 1000))
            # Compute position
            self.une.get_position()

            if self.une.get_flags(UneFlags.compute_is_done):

                tags_to_send = list()

                for tag in self.une.get_tags():
                    if tag.get_flags(UneFlags.new_pvt_is_ready) is True:
                        tags_to_send.append(tag)

                if len(tags_to_send) > 0:
                    pvt_size = asizeof.asizeof(tags_to_send)
                    # Signal emission with a new PVT vector for all tags used in current epoch
                    self.api_sig_new_pvt.emit(tags_to_send)

    def stop(self):
        self.quit()

    @pyqtSlot(UneTag)
    def api_slot_add_tag(self, tag):
        """ Slot method to add new tag in UNE.
            If tag exist then do nothing.
        """
        self.une.add_tag(tag)
        log.debug('-I- [UneTask::api_slot_add_tag] Add tag: {}'.format(tag.get_name()))

    @pyqtSlot(str, float, dict)
    def api_slot_tag_upd_meas(self, tag_name, epoch, meas):
        """ Slot method to add tag measurements.
            If tag doesn't exist then do nothing.
        """
        tag = self.une.get_tag(tag_name)

        if tag is not None:
            # Set epoch time
            tag.set_epoch_curr(epoch)

            # Add measurements
            for a_name, m_list in meas.items():
                tag.add_meas(a_name, m_list[0], m_list[1])

            # Set flag to start computation of the PVT
            tag.set_flags(UneFlags.get_new_pvt, True)

            # log.debug('-I- [UneTask::api_slot_tag_upd_meas] Update tag measurements')


# !!! For test. Delete later
class UneTaskTst(QThread):
    """ Documentation for a class.

        More details.
    """
    # Define API signals
    # This defines a signal for adding new tag in UNE, it takes one argument: UneTag
    sig_add_new_tag = pyqtSignal(UneTag)
    # This defines a signal for updating tag measurement: tag name, {'anchor': [distance, snr], ...}
    sig_upd_tag_meas = pyqtSignal(str, float, dict)

    # !!! For test
    m_pp_meas = dict()
    x, y, z, = [], [], []

    # Auxiliary variables
    const = UneConst()

    def __init__(self):
        """ The constructor """
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

    def run(self):
        """ Documentation for a method
            ..........................
        """
        # !!! For test. Adding anchors for this tag
        my_tag = UneTag('My tag')

        self.prepare_tag(my_tag)

        # Emit signal for adding a new tag in UNE
        self.sig_add_new_tag.emit(my_tag)

        flg_compute = True
        while True:
            self.msleep(100)
            if flg_compute is True:
                flg_compute = False
                for epoch in self.m_pp_meas:
                    self.msleep(100)

                    log.debug('-I- [UneTaskTst::run] Compute tag PVT. UTC:', epoch)

                    # Emit signal for updating tag measurements in UNE
                    self.sig_upd_tag_meas.emit('My tag', float(epoch), self.m_pp_meas[epoch])

    @pyqtSlot(list)
    def slot_new_pvt(self, tags):
        """ Slot method to get new tag position """
        for tag in tags:

            [t_x, t_y, t_z, t_t, t_xv, t_yv, t_zv, t_nv] = tag.get_pvt()
            t_e = tag.get_err_code()

            # If there is additional data then fill list for it
            [hd, vd, pd] = tag.get_dops()

            if t_e is not UneErrors.none:
                log.debug('-I- [UneTaskTst::slot_new_pvt] Error {} during compute PVT for tag: {}'.
                          format(t_e, tag.get_name()))
            else:
                self.x.append(t_x)
                self.y.append(t_y)
                self.z.append(t_z)
                # log.debug('-I- [UneTaskTst::slot_new_pvt] x={:.3f}, y={:.3f}, z={:.3f},\t'
                #       'xv={:.3f}, yv={:.3f}, zv={:.3f}, nv={:.3f}'.format(t_x, t_y, t_z, t_xv, t_yv, t_zv, t_nv))

    def prepare_tag(self, tag):
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
                log.debug("-E-: [UneTaskTst::prepare_tag] Invalid anchor position")
                return False

            # Add the new anchor into the dictionary
            tag.add_anchor(a_name, a_pos[0], a_pos[1], a_pos[2])

        # Fill all measurements dictionary
        self.m_pp_meas = desc_meas['measurements']
