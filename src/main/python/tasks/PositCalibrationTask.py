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

    sig_update_status = pyqtSignal(str, name='PositCalibration_UIUpdateStatus')  # [node_ids]
    sig_start_une_calib = pyqtSignal(dict, dict, name='PositCalibration_UNEStartCalib')

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

    @pyqtSlot(list)
    def calib_finished(self, result):
        log.debug('-I- [UneTaskTst::slot_calib_finished] Antenna delay calibration result: {}'.format(result))

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
            info = {str(self.distance): self.nodes}
            self.sig_start_une_calib.emit(info, self.raw_meas)
            return True
        return False

