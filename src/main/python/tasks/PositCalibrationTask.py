from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal
from enum import Enum
import logging as log
import numpy as np
import json


class State(Enum):
    IDLE = 1
    PREPARE = 2
    WAIT_TWR = 3
    COMPUTING = 4
    READY = 5


class PositCalibrationTask(QThread):

    NUM_RAW_TWR_MEASURES = 1000

    sig_update_status = pyqtSignal(str, name='PositCalibration_UIUpdateStatus') # [node_ids]

    def __init__(self):
        super().__init__()
        self.distance = float()
        self.nodes = list()             # [node_ids]
        self.raw_meas = dict()          # {node_id: {node_id: [measure]}}
        self.meas = dict()              # {node_id: {node_id: measure}}
        self.computed_delays = dict()   # {node_id: delay}}
        self.estimates = None
        self.state = State(State.IDLE)
        return

    def run(self):
        while True:
            if self.state is State.IDLE:
                pass
            elif self.state is State.WAIT_TWR:
                pass
            elif self.state is State.COMPUTING:
                self.compute(self.raw_meas, self.distance)
                self.state = State.READY
            elif self.state is State.READY:
                self.state = State.IDLE

            self.usleep(100)

    def stop(self):
        self.quit()

    @pyqtSlot(object)
    def twr_received(self, twr_info):
        if self.state is State.WAIT_TWR:
            if self.collect_measure(twr_info) is True:
                self.state = State.COMPUTING
        pass

    def start_calibration(self, nodes, distance):
        self.nodes = nodes
        for i in nodes:
            self.raw_meas[str(i)] = dict()
            for j in nodes:
                if j != i:
                    self.raw_meas[str(i)][str(j)] = list()

        if distance <= 0:
            self.distance = 2.6        # equal distance between all nodes
        else:
            self.distance = distance

        self.state = State.WAIT_TWR
        self.sig_update_status.emit('Calibration started')

    def set_state(self, state):
        self.state = state

    def collect_measure(self, twr_info):
        node_id = str(twr_info.NodeID)
        tag_id = str(twr_info.InitiatorID)
        if len(self.raw_meas[node_id][tag_id]) < self.NUM_RAW_TWR_MEASURES:
            self.raw_meas[node_id][tag_id].append(twr_info.Distance)
            status = 'Calibrating '
            for i in self.raw_meas:
                for j in self.raw_meas[i]:
                    status += ' {}/{} : {} '.format(i, j, len(self.raw_meas[i][j]))
            self.sig_update_status.emit(status)
        else:
            for node, values in self.raw_meas.items():
                for node_t, values_t in values.items():
                    if len(values_t) < self.NUM_RAW_TWR_MEASURES:
                        return False
            with open('./logs/calib_data.json', 'w') as outfile:
                json.dump(self.raw_meas, outfile)
            return True
        return False

    @staticmethod
    def compute(measures, distance):
        dwt_time_units = (1.0 / 499.2e6 / 128.0)
        sof = 299702547

        iterations = 100
        candidates_num = 1000

        # Remove invalid values from measures
        for values in measures.values():
            for val in values.values():
                medium = np.median(val)
                medium_p = 1.2 * medium
                medium_m = 0.8 * medium
                for v in val:
                    if (v < medium_m) or (v > medium_p):
                        val.remove(v)
        print(measures)

        # Fill EDM matrices
        n = len(measures)
        edm_meas = np.zeros((n, n))
        edm_act = np.zeros((n, n))
        for i, (di_k, di_v) in enumerate(measures.items()):
            for j, (dj, ranges) in enumerate(di_v.items()):
                if j >= i:
                    edm_meas[i, j+1] = np.median(ranges)
                    edm_act[i, j+1] = distance
                else:
                    edm_meas[i, j] = np.median(ranges)
                    edm_act[i, j] = distance

        print(edm_act)
        print(edm_meas)

        # EDM to TOF
        tof_meas = edm_meas / sof / dwt_time_units
        tof_act = edm_act / sof / dwt_time_units

        print('\n', tof_meas)
        print('\n', tof_act)

        init_delay = 513
        perturb_limit = 0.2
        cand_list = list()
        cand_norm_diff = dict()

        for i in range(iterations):
            # Populate set of candidate
            if i == 0:
                # Generate a set of random delays uniformly distributed round initial delay +-6ns
                cand_list = np.random.uniform(init_delay-6, init_delay+6, candidates_num)
            else:
                new_cand_list = cand_list[:iterations//4]
                for m in range(3):
                    add_cand_list = np.random.uniform(-perturb_limit, +perturb_limit, iterations//4)
                    for j, cand in enumerate(new_cand_list):
                        new_cand_list += cand + add_cand_list[j]
                if i % 20 == 0:
                    perturb_limit /= 2
                cand_list = new_cand_list

            # Evaluate the quality of the candidates
            print(cand_list, '---\n', tof_meas, '---\n', )
            for cand in cand_list:
                tof_cand = tof_meas + cand
                np.fill_diagonal(tof_cand, 0)
                cand_norm_diff[cand] = np.linalg.norm(tof_act - tof_cand)

            # Sort by value - lowest error first
            cand_norm_diff = {k: v for k, v in sorted(cand_norm_diff.items(), key=lambda item: item[1])}
            cand_list = list(cand_norm_diff.keys())

        return tof_meas + cand_list[0]
