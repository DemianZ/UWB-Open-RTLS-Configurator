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

    NUM_RAW_TWR_MEASURES = 100
    NUM_ESTIMATES = 1000
    NUM_ITERATIONS = 100

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
                for node_id in self.meas:
                    self.compute(self.meas[node_id])
                    self.usleep(100)
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
            self.distance = 1.5        # equal distance between all nodes
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
            for node in self.nodes:
                if len(self.raw_meas[node][tag_id]) < self.NUM_RAW_TWR_MEASURES:
                    return False
            with open('../logs/calib_data.json', 'w') as outfile:
                json.dump(self.nodes, outfile)
            return True
        return False

    def compute(self, measures):
        return
