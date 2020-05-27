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


class UneMeas:
    """ This class is used for transmitting measurements
        to UneTask signal
    """

    _tag_name = ''    # Name of the tag
    _meas_time = 0    # Time of measurement
    _meas = dict()    # Dictionary of measurements {'anchor_name':[meas1, meas2,..],...}

    def __init__(self, tag_name, meas_time, meas_dict):
        self._tag_name = tag_name
        self._meas_time = meas_time
        self._meas = meas_dict

    @property
    def name(self):
        """ Get tag name to identify measurements """
        return self._tag_name

    @property
    def meas_time(self):
        """ Get measurements time (UTC) """
        return self._meas_time

    @meas_time.setter
    def meas_time(self, t):
        """ Set measurements time (UTC) """
        self._meas_time = t

    @property
    def meas(self):
        """ Get measurements dictionary """
        return self._meas

    @meas.setter
    def meas(self, m):
        """ Set measurements dictionary """
        self._meas = m


class UneConst:
    """ This class contains different constant values
        which are used in main class of UNE
    """

    _epoch_period = 2
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
    _calib_meas_min_len = 200
    _max_pdop = 6

    @property
    def epoch_period(self):
        """ Epoch period in milliseconds """
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

    @property
    def calib_meas_min_len(self):
        """ Minimum number of measurements for calibrating """
        return self._calib_meas_min_len

    @property
    def max_pdop(self):
        """ Maximum value of PDOP factor """
        return self._max_pdop


class UneErrors(Enum):
    """ Enumerator class describing errors in the UNE module """
    none = 0
    not_enough_meas = 1
    too_many_iteration = 2
    linalg_error = 3
    big_dop = 4


class UneFlags(Enum):
    """ Enumerator class describing all flags used in the UNE module """
    noname_flag = 1       #
    get_new_pvt = 2       # Set this flag after adding tag measurements to start PVT computation
    new_pvt_is_ready = 3  # If set, a new PVT is available
    compute_is_done = 4   # If set, computation for all tags has been done
    tag_extend_info = 5   # Flag for tag to send adding some information to PVT: DOP
    calib_start = 6       # Flag to start calibrating process
    calib_finished = 7    # Flag is set when calibration is finished


class UneNavMethod(Enum):
    """ Enumerator class describing different navigation
        methods for the UNE module """
    lsm = 1
    weighted_lsm = 2
    kalman = 3


class UneTag:
    """ Class that contains methods, properties and other fields
        to describe states of tag and to control it
    """
    # Tag properties
    _name = ''                            # Unique tag name
    _pvt_prev = [0, 0, 0, 0, 0, 0, 0, 0]  # Previous tag Position/Velocity/Time
    _pvt_curr = [0, 0, 0, 0, 0, 0, 0, 0]  # Current tag Position/Velocity/Time
    _residuals = list()                   # Residual vector
    _meas_prev = dict()                   # Previous epoch measurements between tag and anchors
    _meas_curr = dict()                   # Current epoch measurements between tag and anchors
    _anchors = dict()                     # Anchors for this tag -> dictionary {name: [x, y, z], ...}
    _err_code = UneErrors.none            # Error code
    _time_for_compute = 0                 # To meas time that is required for compute PVT
    _pdop = 0                             # Position dilution of precision
    _hdop = 0                             # Horizontal dilution of precision
    _vdop = 0                             # Vertical dilution of precision
    _epoch_prev = 0                       # Time of previous epoch (UTC with ms)
    _epoch_curr = 0                       # Time of current epoch (UTC with ms)

    # Flags
    _flg_get_new_pvt = False
    _flg_got_new_pvt = False
    _flg_extend_info = False

    # Auxiliary variables
    const = UneConst()

    def __init__(self, name):
        """ The constructor """
        self._name = name

    def get_name(self):
        """ Get tag name """
        return self._name

    def set_epoch_curr(self, t):
        """ Set time of current epoch """
        self._epoch_curr = t

    def get_epoch_curr(self):
        """ Get time of current epoch """
        return self._epoch_curr

    def add_meas(self, key_anchor, meas, snr=-128):
        """ Add tag measurement """
        self._meas_curr[key_anchor] = [meas, snr]
        return

    def get_meas_curr(self):
        """ Get tag measurements for current epoch """
        return sorted(self._meas_curr.items())

    def get_meas_prev(self):
        """ Get tag measurements for previous epoch """
        return self._meas_prev.copy()

    def clr_meas(self):
        """ Clear tag measurement (it's used before add measurements for new epoch) """
        self._meas_prev = self.get_meas_curr()
        self._pvt_prev = self.get_pvt()
        self._meas_curr.clear()
        return

    def get_pvt(self):
        """ Get current tag position/velocity and error of time """
        return self._pvt_curr.copy()

    def upd_pos(self, dx, dy, dz, dt):
        """ Update current tag position and time error """
        self._pvt_curr[self.const.x] += dx
        self._pvt_curr[self.const.y] += dy
        self._pvt_curr[self.const.z] += dz
        self._pvt_curr[self.const.t] += dt
        return

    def upd_vel(self, p):
        """ Update current tag velocity
            p1 - current position
        """
        if self._epoch_prev > 0:
            dt = self._epoch_curr - self._epoch_prev

            xv = (p[self.const.x] - self._pvt_prev[self.const.x]) / dt
            yv = (p[self.const.y] - self._pvt_prev[self.const.y]) / dt
            zv = (p[self.const.z] - self._pvt_prev[self.const.z]) / dt

            self._pvt_curr[self.const.xv] = xv
            self._pvt_curr[self.const.yv] = yv
            self._pvt_curr[self.const.zv] = zv
            self._pvt_curr[self.const.nv] = math.sqrt(xv ** 2 + yv ** 2 + zv ** 2)

        return

    def upd_epoch_time(self):
        """" Update previous epoch time """
        self._epoch_prev = self._epoch_curr

    def set_pvt(self, x, y, z, t, xv, yv, zv, nv):
        """ Set tag position/velocity/time vector """
        self._pvt_curr = [x, y, z, t, xv, yv, zv, nv]
        return

    def set_pvt(self, pvt):
        """ Set tag pvt """
        self._pvt_curr = pvt.copy()
        return

    def set_dops(self, h=100, v=100, p=100):
        """ Set DOP factors: HDOP, VDOP, PDOP """
        self._hdop = h
        self._vdop = v
        self._pdop = p

    def get_dops(self):
        """ Get DOP factors of tag """
        return [self._hdop, self._vdop, self._pdop]

    def set_residuals(self, r):
        """ Set residual vector for tag
            and automatically calculate
            DOP factors
        """
        self._residuals = list()
        self._residuals = r.copy()

    def get_residuals(self):
        """ Get residual vector of tag """
        return self._residuals.copy()

    def set_flags(self, flg, val=True):
        """ Method to set flags """
        if (val is not True) and (val is not False):
            log.debug('-E- [UneTag::setFlags::tag: {}] Incorrect argument value: {}'.format(self.m_name, val))

        if flg is UneFlags.get_new_pvt:
            self._flg_get_new_pvt = val
        elif flg is UneFlags.tag_extend_info:
            self._flg_extend_info = val
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
            self._flg_got_new_pvt = val
        else:
            log.debug('-E- [UneTag::_setFlags::tag: {}] Flag {} not supported'.format(self.m_name, flg))

    def get_flags(self, flg):
        """ Method to get flags """
        if flg is UneFlags.get_new_pvt:
            res = self._flg_get_new_pvt
            # Reset flag after request
            self._flg_get_new_pvt = False
            return res
        elif flg is UneFlags.new_pvt_is_ready:
            res = self._flg_got_new_pvt
            # Reset flag after request
            self._flg_got_new_pvt = False
            return res
        elif flg is UneFlags.tag_extend_info:
            return self._flg_extend_info
        else:
            log.debug('-E- [UneTag::getFlags::tag_{}] Flag {} not supported'.format(self.m_name, flg))

    def set_err_code(self, val=UneErrors.none):
        """ Method to set error code.
            It's used only in UNE class.
            You shouldn't call this method.
        """
        self._err_code = val

    def get_err_code(self):
        """ Method to get error code """
        return self._err_code

    def add_anchor(self, key, x, y, z):
        """ Add new anchor with position """
        self._anchors[key] = [x, y, z]

    def clr_anchor(self, key, meas, snr=-128):
        """ Clear anchors dictionary """
        self._anchors.clear()

    def get_anchors(self):
        """ Get anchors dictionary """
        return self._anchors.copy()

    def get_anchor(self, key):
        """ Get anchor by key """
        if key in self._anchors:
            return self._anchors[key]
        else:
            log.debug("-E-: [Une::get_anchor] Anchor dictionary doesn't contain {} key".format(key))

    def get_anchor_pos(self, key):
        """ Get anchor position
            Get position [x, y, z] of anchor by key relative origin
        """
        return self._anchors[key]

    def get_dist(self, p1, p2=None):
        """ Get distance between two points [x, y, z] """
        if p2 is None:
            p2 = [0, 0, 0]

        dX = p1[self.const.x] - p2[self.const.x]
        dY = p1[self.const.y] - p2[self.const.y]
        dZ = p1[self.const.z] - p2[self.const.z]

        return (dX**2 + dY**2 + dZ**2)**0.5

    def get_pos_and_dist(self, key, p2):
        """ Get position of anchor by key and distance between anchor and point [x, y, z] """
        p1 = self._anchors[str(key)]

        dX = p2[self.const.x] - p1[self.const.x]
        dY = p2[self.const.y] - p1[self.const.y]
        dZ = p2[self.const.z] - p1[self.const.z]

        return [dX, dY, dZ, (dX**2 + dY**2 + dZ**2)**0.5]

    def tic(self):
        """ Method to start meas time of computation of PVT """
        self._time_for_compute = time.time_ns()

    def toc(self):
        """ Method for stopping the measurement of PVT computation time.
            Return time in ms
        """
        self._time_for_compute = time.time_ns() - self._time_for_compute

        return self._time_for_compute / 1000.0


class Une:
    """ Class that contains methods, properties and other fields
        to describe states of tag and to control it
    """
    # Properties
    _nav_method = UneNavMethod.lsm

    # Auxiliary variables
    _num_of_pt_prm = 3     # 3 parameters now: [x, y, z]
    _num_of_v_prm = 3      # 3 parameters: [xv, yv, zv]
    _pos_err = 0.005       # Error threshold between the previous and current computation steps to stop computation [m]

    _const = UneConst()

    # For antenna delay estimation
    _calib_info = dict()
    _calib_meas = dict()
    _calib_rslt = list()

    # Flags
    _flg_compute_is_done = False
    _flg_calib_start = False
    _flg_calib_finished = False

    # Indices of measurements
    _ind_dist = 0
    _ind_cnr = 1

    # List of tags
    _tags = list()

    def __init__(self, nav_method):
        """ The constructor.
            To choose nav_method you should use
            the UneNavMethod enumerator.
        """
        self._nav_method = nav_method

    def add_tag(self, tag_new):
        """ Method to add new tag """
        for tag in self._tags:
            if tag_new.get_name() == tag.get_name():
                log.debug('-W- [Une::add_new_tag] Tag {} has already added'.format(tag_new.get_name()))
                return -1

        self._tags.append(tag_new)

        # Return current tag object
        return 0

    def get_tag(self, name):
        """ Method to get one tag object """
        for tag in self._tags:
            if name == tag.get_name():
                return tag
        return None

    def get_tags(self):
        """ Method to get tags list """
        return self._tags

    def rem_tag(self, name):
        """ Method to remove tag from tags list """
        for tag in self._tags:
            if name == tag.get_name():
                self._tags.remove(tag)

    def clr_tags(self):
        """ Method to clear tags list """
        self._tags.clear()

    def get_position(self):
        """ Comment... """

        # Get PVT for all tags in the list m_tags
        for tag in self._tags:
            # If need to compute new tag position
            if tag.get_flags(UneFlags.get_new_pvt) is True:

                # tag.tic()

                if self._nav_method == UneNavMethod.lsm:
                    self.get_position_lsm(tag)
                elif self._nav_method == UneNavMethod.weighted_lsm:
                    self.get_position_wlsm(tag)
                elif self._nav_method == UneNavMethod.Kalman:
                    self.get_position_kalman(tag)

                # toc_t = tag.toc()
                # log.debug('Computation time of the PVT for tag: {} is equal {:5.3f} ms'.format(tag.get_name(), toc_t))

        self.set_flags(UneFlags.compute_is_done, True)

    def get_position_lsm(self, tag):
        """ Method to compute tag position using a system of linear equations
            and least squares method
            y = H*x -> x = inv(Ht * H) * Ht * y
            where:
            y - measurement vector
            H - geometry matrix
            x - tag state vector (position, velocity,...)
         """
        num_anchors = len(tag.get_anchors())
        num_max_iteration = self._const.lse_max_iter
        flg_stop_computation = False

        if num_anchors < self._const.meas_min:
            tag.set_pvt(0, 0, 0)
            # Setting this flag to emit signal from UneTask
            tag.set_int_flags(UneFlags.new_pvt_is_ready)
            tag.set_err_code(UneErrors.not_enough_meas)
            return

        # Get measurements for current & previous epoch
        meas_prev = tag.get_meas_prev()
        meas_curr = tag.get_meas_curr()

        # Tag state vector
        mtx_x = np.zeros(self._num_of_pt_prm)

        # Tag state vector
        mtx_v = np.zeros(self._num_of_v_prm)

        # Geometry matrix
        mtx_g = np.zeros((num_anchors, 3))

        # Measurement vector for position
        mtx_y = np.zeros(num_anchors)

        # Residual vector
        mtx_r = np.zeros(num_anchors)

        # Matrices to compute DOPs and auxiliary matrix
        mtx_q = np.zeros((3, 3))
        mtx_s = np.zeros((3, num_anchors))

        # Save current tag position
        tag_pvt_save = tag.get_pvt()

        # Iterative (9 attempts maximum recommended) computation of tag position by using LSE method
        while (num_max_iteration > 0) and (flg_stop_computation is False):

            # Clear matrices
            mtx_g.fill(0)
            mtx_y.fill(0)
            mtx_q.fill(0)
            mtx_s.fill(0)

            num_max_iteration -= 1

            # Get tag position obtained at the previous step
            tag_pvt_prev = tag.get_pvt()

            cnt_meas = 0
            for a_name, a_meas in meas_curr:
                # Get distance between anchor and tag position on previous computation step
                [x, y, z, d_prev] = tag.get_pos_and_dist(a_name, tag_pvt_prev)

                # Get difference between previous and current distances to compute error in current user position
                d_err = a_meas[0] - d_prev

                # Filling geometry and measurements matrices for
                # computing errors of user position, velocity and time [dx, dy, dz, dt] [dvx, dvy, dvz]
                mtx_g[cnt_meas] = [x / d_prev, y / d_prev, z / d_prev]
                mtx_y[cnt_meas] = d_err
                cnt_meas += 1

            # Using the LSE method for estimating user state vector: mtx_x = [dx, dy, dz]
            mtx_g_trp = mtx_g.transpose()
            try:
                mtx_q = np.linalg.inv(np.matmul(mtx_g_trp, mtx_g))
            except np.linalg.LinAlgError as e:
                log.debug("-E-: [Une::get_position_lse] LinAlgError for tag {}: {}".format(tag.get_name(), e))
                tag.set_pvt(tag_pvt_save)
                tag.set_err_code(UneErrors.linalg_error)
                # Setting this flag to emit signal from UneTask
                tag.set_int_flags(UneFlags.new_pvt_is_ready)
                return

            # Get DOP factors
            p_dop = mtx_q.trace()
            h_dop = math.sqrt(mtx_q[0][0] ** 2 + mtx_q[1][1] ** 2)
            v_dop = math.sqrt(mtx_q[2][2] ** 2)

            if p_dop > self._const.max_pdop and tag.get_dist(tag_pvt_save[:3]) > 0.1:
                # Set previous PVT
                tag.set_pvt(tag_pvt_save)
                # Set error code
                tag.set_err_code(UneErrors.big_dop)
                # Set this flag to emit signal from UneTask
                tag.set_int_flags(UneFlags.new_pvt_is_ready)
                return

            mtx_s = np.matmul(mtx_q, mtx_g_trp)
            mtx_x = np.matmul(mtx_s, mtx_y)

            # Correct current position by errors from state vector mtx_x
            tag.upd_pos(mtx_x[self._const.x], mtx_x[self._const.y], mtx_x[self._const.z], 0)

            # Get tag position attempted on the current step
            tag_pos_curr = tag.get_pvt()

            # List to save tag position
            p_curr = tag_pos_curr[:4]

            # Computation of the error between the previous and current position estimates
            # and if it less than threshold then stop computation
            err_dist = tag.get_dist(tag_pvt_prev[:4], p_curr)
            if err_dist <= self._pos_err:
                flg_stop_computation = True

            # Go to the next iteration step (above) ^^^

        # Update tag velocity
        tag.upd_vel(p_curr)

        # Update previous epoch timestamp
        tag.upd_epoch_time()

        # Get residuals vector
        mtx_yr = np.matmul(mtx_g, mtx_x)
        mtx_r = np.subtract(mtx_y, mtx_yr)
        tag.set_residuals(mtx_r)

        # Set DOPs
        tag.set_dops(h_dop, v_dop, p_dop)

        # Setting this flag to emit signal from UneTask
        tag.set_int_flags(UneFlags.new_pvt_is_ready)

        # If tag position was successfully computed
        if flg_stop_computation is True:
            tag.set_err_code(UneErrors.none)
        else:
            log.debug("-E-: [Une::get_position_lse] Too many iteration for tag {}".format(tag.get_name()))
            tag.set_pvt(tag_pvt_save)
            tag.set_err_code(UneErrors.too_many_iteration)

    def get_position_wlsm(self, pos):
        """ Comment... """
        pos = [0, 0, 0]
        return UneErrors.none

    def get_position_kalman(self, pos):
        """ Comment... """
        pos = [0, 0, 0]
        return UneErrors.none

    def calibration_init(self, info, meas):
        """ Method to check data for calibrating, updating local dictionaries with info/meas
            and setting flag UneFlags.calib_start if there isn't any errors

            info: { triangle side length [m]: ['id1', 'id2', 'id3']}
            meas: {
                    'id1':
                    {
                      'id2': [dist1, dist2,...],
                      'id3': [dist1, dist2,...]
                    },
                    'id2':
                    {
                      'id1': [dist1, dist2,...],
                      'id3': [dist1, dist2,...]
                    },
                    'id3':
                    {
                      'id1': [dist1, dist2,...],
                      'id2': [dist1, dist2,...]
                    }
                  }
        """
        key = list(info.keys())[0]

        if len(info[key]) != 3:
            log.debug('-E-: [Une::calibration_init] Invalid init: {}'.format(info))
            return False

        id1 = info[key][0]
        id2 = info[key][1]
        id3 = info[key][2]

        if (id1 not in meas) or (id2 not in meas) or (id3 not in meas):
            log.debug('-E-: [Une::calibration_init] meas doesnt have all the ids')
            return False

        if (id2 not in meas[id1]) or (id3 not in meas[id1]):
            log.debug('-E-: [Une::calibration_init] meas[id1] doesnt have all the ids')
            return False

        if (id1 not in meas[id2]) or (id3 not in meas[id2]):
            log.debug('-E-: [Une::calibration_init] meas[id2] doesnt have all the ids')
            return False

        if (id1 not in meas[id3]) or (id2 not in meas[id3]):
            log.debug('-E-: [Une::calibration_init] meas[id3] doesnt have all the ids')
            return False

        if len(meas[id1][id2]) < self._const.calib_meas_min_len:
            log.debug('-E-: [Une::calibration_init] Not enough measurements')
            return False

        self._calib_info = info
        self._calib_meas = meas

        # If there isn't errors, then start calibrating
        self.set_flags(UneFlags.calib_start)

        return True

    def calibration(self):
        """ Method to estimate antennas delay.
            The Kalman filter is used to estimate the antenna delays
            of three devices arranged in an equilateral triangle.

            Note
            -------
            !!! NOTE: This method is called automatically.
            You only should call the calibration_init method.
        """

        log.debug('-I- [Une::_calibration] Start calibration')

        const = UneConst

        key = list(self._calib_info.keys())[0]

        # Get init parameters
        D = float(key)
        id1 = self._calib_info[key][0]
        id2 = self._calib_info[key][1]
        id3 = self._calib_info[key][2]

        # Number of measurements
        N = len(self._calib_meas[id1][id2])

        # Auxiliary variables
        nWay = 6
        nPrm = 8
        lspd = 299792485         # Light speed[m / s]
        posc = np.zeros((N, 2))  # Position after compensating the antenna delays

        # To get system unit from distance
        DWT_TIME_UNITS = (1.0 / 499.2e6 / 128.0)

        # Devices position
        class Point:
            _x = 0
            _y = 0

            def __init__(self, x, y):
                self._x = x
                self._y = y

            @property
            def x(self):
                return self._x

            @x.setter
            def x(self, val):
                self._x = val

            @property
            def y(self):
                return self._y

            @y.setter
            def y(self, val):
                self._y = val

        p1 = Point(1.0, 1.0)
        p2 = Point(p1.x + D, p1.y)
        p3 = Point(p1.x + D / 2, p1.y + np.sqrt(D**2 - (D / 2)**2))
        ref = Point(p1.x + D / 2, p1.y + np.sqrt(D**2 - (D / 2)**2) / 3)

        # Getting standard deviation of measurement
        meas_std = np.std(self._calib_meas[id1][id2])

        # Define variables for init state
        P_INIT = 10.0
        R_INIT = meas_std**2
        Q_INIT_ANT = 1.0e-9
        Q_INIT_POS = 1.0e-5

        # Main matrices
        X = np.zeros(nPrm)
        F = np.diag(np.ones(nPrm))
        I = np.diag(np.ones(nPrm))
        H = np.zeros((nWay, nPrm))
        Y = np.zeros(nWay)
        P = np.diag(np.ones(nPrm) * P_INIT)
        Q = np.diag(np.hstack([np.ones(2) * Q_INIT_POS, np.ones(6) * Q_INIT_ANT]))
        R = np.diag(np.ones(nWay) * R_INIT)

        # Coefficient to recomupte range error to distance error
        h = 1 / (2 * np.cos(30 * np.pi / 180))

        # Range to triangle centroid
        d = D * h
        x1 = (p1.x - ref.x) / d
        y1 = (p1.y - ref.y) / d
        x2 = (p2.x - ref.x) / d
        y2 = (p2.y - ref.y) / d
        x3 = (p3.x - ref.x) / d
        y3 = (p3.y - ref.y) / d

        # Init the measurement matrix
        H[0][:] = [x1, y1, h, 0, 0, h, 0, 0]
        H[1][:] = [x2, y2, 0, 0, h, 0, 0, h]
        H[2][:] = [x3, y3, 0, h, 0, 0, h, 0]
        H[3][:] = [x1, y1, h, 0, 0, 0, 0, h]
        H[4][:] = [x3, y3, 0, 0, 0, h, h, 0]
        H[5][:] = [x2, y2, 0, h, h, 0, 0, 0]

        for i in range(N):

            # Updating the measurement vector
            Y[0] = self._calib_meas[id1][id2][i] * h - d
            Y[1] = self._calib_meas[id2][id3][i] * h - d
            Y[2] = self._calib_meas[id3][id1][i] * h - d
            Y[3] = self._calib_meas[id1][id3][i] * h - d
            Y[4] = self._calib_meas[id3][id2][i] * h - d
            Y[5] = self._calib_meas[id2][id1][i] * h - d

            # Step1. Predicting
            X = np.matmul(F, X)
            P = np.matmul(np.matmul(F, P), F.transpose()) + Q

            # Step2. Computing the Kalman gain
            try:
                S = np.linalg.inv(np.matmul(np.matmul(H, P), H.transpose()) + R)
            except np.linalg.LinAlgError as e:
                log.debug('-E- [Une::calibration] Error: {}'.format(e))

            K = np.matmul(np.matmul(P, H.transpose()), S)

            # Step3. Updating the state vector and the process covariance matrix
            X = X + np.matmul(K, Y - np.matmul(H, X))
            P = np.matmul((I - np.matmul(K, H)), P)

            # Save current position estimate for plotting
            posc[i][:] = [ref.x + X[0], ref.y + X[1]]

            # Reset position in state vector
            X[:2] = 0

        # Change distance to system unit
        for i in range(2, 8):
            X[i] = round(X[i] / lspd / DWT_TIME_UNITS)

        log.debug('-I- [Une::_calibration] Result (tx1, rx1, tx2, rx2, tx3, rx3): {}'.format(X[2:]))

        # The result (tx1, rx1, tx2, rx2, tx3, rx3)
        self._calib_rslt = X[2:]

        self.set_flags(UneFlags.calib_finished)

    def get_calib_result(self):
        """ Return list of antenna delays (tx1, rx1, tx2, rx2, tx3, rx3) """
        return list(self._calib_rslt)

    def set_flags(self, flg, val=True):
        """ Method to set flags

            flg: UneFlags field
            val: Boolean value
        """
        if (val is not True) and (val is not False):
            log.debug('-E- [Une::set_flags] Incorrect argument value: {}'.format(val))

        if flg is UneFlags.compute_is_done:
            self._flg_compute_is_done = val
        elif flg is UneFlags.calib_start:
            self._flg_calib_start = val
        elif flg is UneFlags.calib_finished:
            self._flg_calib_finished = val
        else:
            log.debug('-E- [Une::set_flags] Flag {} not supported'.format(flg))

    def get_flags(self, flg):
        """ Method to get flags

            flg: UneFlags field
        """
        if flg is UneFlags.compute_is_done:
            res = self._flg_compute_is_done
            # Reset flag after request
            self._flg_compute_is_done = False
            return res
        elif flg is UneFlags.calib_start:
            res = self._flg_calib_start
            # Reset flag after request
            self._flg_calib_start = False
            return res
        elif flg is UneFlags.calib_finished:
            res = self._flg_calib_finished
            # Reset flag after request
            self._flg_calib_finished = False
            return res
        else:
            log.debug('-E- [UneTwrTask::get_flags] Flag {} not supported'.format(flg))


class UneTask(QThread):
    """ Documentation for a class.

        More details.
    """
    # Define API signals
    # This defines a signal that takes one argument: list of UneTag objects that are used in computation
    api_sig_new_pvt = pyqtSignal(list)
    # This defines a signal that takes one argument: list of antenna delays (tx1, rx1, tx2, rx2, tx3, rx3)
    api_sig_calib_finished = pyqtSignal(list)

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
            self.msleep(int(self.const.epoch_period))

            # Compute position
            self.une.get_position()

            # Check flags
            if self.une.get_flags(UneFlags.compute_is_done):

                tags_to_send = list()

                for tag in self.une.get_tags():
                    if tag.get_flags(UneFlags.new_pvt_is_ready) is True:
                        tags_to_send.append(tag)

                if len(tags_to_send) > 0:
                    # Signal emission with a new PVT vector for all tags used in current epoch
                    self.api_sig_new_pvt.emit(tags_to_send)

            if self.une.get_flags(UneFlags.calib_start):
                self.une.calibration()
            elif self.une.get_flags(UneFlags.calib_finished):
                self.api_sig_calib_finished.emit(self.une.get_calib_result())

    def stop(self):
        self.quit()

    @pyqtSlot(dict, dict)
    def api_slot_calibrate(self, info, meas):
        """ Slot method to start calibrating process
            to find antenna delays.

            Note
            ______
            200 measurements for each pair minimum
            ------
            info: { triangle side length [m]: ['id1', 'id2', 'id3']}
            meas: {
                    'id1':
                    {
                      'id2': [dist1, dist2,...],
                      'id3': [dist1, dist2,...]
                    },
                    'id2':
                    {
                      'id1': [dist1, dist2,...],
                      'id3': [dist1, dist2,...]
                    },
                    'id3':
                    {
                      'id1': [dist1, dist2,...],
                      'id2': [dist1, dist2,...]
                    }
                  }
        """

        self.une.calibration_init(info, meas)

    @pyqtSlot(UneTag)
    def api_slot_add_tag(self, tag):
        """ Slot method to add new tag in UNE.
            If tag exist then do nothing.

            tag: UneTag object with unique name
        """
        self.une.add_tag(tag)
        log.debug('-I- [UneTask::api_slot_add_tag] Add tag: {}'.format(tag.get_name()))

    @pyqtSlot(list)
    def api_slot_tag_upd_meas(self, meas_list):
        """ Slot method to add tag measurements.
            If tag doesn't exist then do nothing.

            meas_list: list of UneMeas objects
        """
        for meas in meas_list:
            tag = self.une.get_tag(meas.name)

            if tag is None:
                log.debug('-I- [UneTask::api_slot_tag_upd_meas] Tag {} does not exist'.format(meas.name))
                continue

            # Save previous measurement and clear current
            tag.clr_meas()

            # Set epoch time
            tag.set_epoch_curr(meas.meas_time)

            # Add measurements
            for a_name, m_list in meas.meas.items():
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

    # This defines a signal for updating tag measurements.
    # Arguments: list of UneMeas objects
    sig_upd_tag_meas = pyqtSignal(list)
    # To start calibration
    sig_calib_start = pyqtSignal(dict, dict)

    # Flags
    _flg_next_step = False

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

        # Load data for calibrating
        with open("logs/calib_data_5_4.json", "r") as read_file:
            calib_meas = json.load(read_file)

        info = {2.6: ['12', '13', '14']}
        meas = calib_meas
        self.sig_calib_start.emit(info, meas)

        flg_compute = True
        while True:
            self.msleep(100)
            if flg_compute is True:
                flg_compute = False
                for epoch in self.m_pp_meas:
                    self.msleep(100)

                    log.debug('-I- [UneTaskTst::run] Compute tag PVT. UTC: {}'.format(epoch))

                    # Emit signal for updating tag measurements in UNE
                    tag_meas = list()
                    tag_meas.append(UneMeas('My tag', float(epoch), self.m_pp_meas[epoch]))
                    self.sig_upd_tag_meas.emit(tag_meas)

                    # Waiting while
                    while self._flg_next_step is False:
                        self.msleep(100)
                    self._flg_next_step = False

    @pyqtSlot(list)
    def slot_calib_finished(self, result):
        """ Slot method to get result of antenna delay calibration """
        log.debug('-I- [UneTaskTst::slot_calib_finished] Antenna delay calibration result: {}'.format(result))
        pass

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
        self._flg_next_step = True

    def prepare_tag(self, tag):
        """ !!! Test method. Delete later.
            Load description and measurements from JSON file
            for postprocessing
        """
        # # Load data for postprocessing
        # with open("logs/my_new_log.json", "r") as read_file:
        #     desc_meas = json.load(read_file)
        #
        # # Add all available anchors and their positions
        # for a_name, a_pos in desc_meas['description']['anchors'].items():
        #     if len(a_pos) != 3:
        #         log.debug("-E-: [UneTaskTst::prepare_tag] Invalid anchor position")
        #         return False
        #
        #     # Add the new anchor into the dictionary
        #     tag.add_anchor(a_name, a_pos[0], a_pos[1], a_pos[2])
        #
        # # Fill all measurements dictionary
        # self.m_pp_meas = desc_meas['measurements']

        # CRASH LOG
        # Load data for postprocessing
        with open("logs/crash_log.json", "r") as read_file:
            desc_meas = json.load(read_file)

        # Add all available anchors and their positions
        tag.add_anchor('11', 3.795, 0.255, 1.090)
        tag.add_anchor('12', 0.347, 0.240, 1.090)
        tag.add_anchor('13', 0.256, 2.820, 1.116)
        tag.add_anchor('14', 3.640, 2.330, 1.063)

        init_epoch = 1588786132
        meas = dict()

        for epoch, val in desc_meas.items():
            meas[str(init_epoch)] = val[1]
            init_epoch += 1

        # Fill all measurements dictionary
        self.m_pp_meas = meas
