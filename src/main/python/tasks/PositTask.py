from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import atexit
import logging as log
import json
import os
import sys
from tasks.UneTask import UneTag, UneErrors, UneMeas
from proto import Settings_pb2
from google.protobuf.json_format import Parse, ParseDict
from modules.wake import Wake
import time


# DEFAULT_ANT_DELAY = 16436
# STANDARD DELTA DELAYS: RX = 18418, TX = 14472
#
# @brief:
#   Task for controlling UNE client list and
#   collecting, storing and processing UNE data
# @args:
#   MainWindow
# @classSignals:
#   PositTask_UpdateSettings:   args: list [ip, {settings}]
#       - signal for updating settings in UI network settings table widget
#   PositTask_AddDevice:        args: list [ip, {settings}]
#       - signal for adding new device to UI device list table widget.
#   PositTask_RemoveDevice:     args: str (ip)
#       - signal for removing device from UI device list table widget
#   PositTask_UpdateDevice:     args: list [ip, id, type, mode, rx/tx, error]
#       - signal to update UI device une status parameters
#
#   PositTask_AddAnchorResp:    args: str(node_id)
#       - response to UI for adding anchor to UNE anchor list
#   PositTask_AddTagResp
#       - response to UI for adding tag to UNE tag list
#   PositTask_ConnectUneResp    args: dict {TagID: [AnchorId, ...]}
#       - response to UI for connecting anchors and nodes.
#
#   UneTask_AddTag:             @args UneTag()
#       - adding new tag in UNE
#   UneTask_UpdMeas:            @args: tag name, {'anchor': [distance, snr], ...}
#       - signal for updating tag measurement

class PositTask(QThread):
    sig_une_add_new_tag = pyqtSignal(UneTag, name='UneTask_AddTag')
    sig_une_upd_tag_meas = pyqtSignal(list, name='UneTask_UpdateMeasures')

    sig_ui_update_anchor_resp = pyqtSignal(list, name='PositTask_AddAnchorResp')
    sig_ui_update_tag_resp = pyqtSignal(list, name='PositTask_AddTagResp')
    sig_ui_connect_une_resp = pyqtSignal(list, name='PositTask_ConnectUneResp')

    sig_ui_new_pvt = pyqtSignal(list, name='PositTask_NewPVT')

    def __init__(self):
        QThread.__init__(self)
        atexit.register(self.terminate)     # function to be executed on exit

        self.anchor_list = list()           # [[ip, {settings}]]
        self.anchor_une_list = list()       # [[id, [pos_x, pos_y, pos_z]]]
        self.tag_une_list = list()          # [id]
        self.une_list = list()              # [[UneTag ,[anchor_id,...]]]
        self.une_activated = False
        self.epoch_pvt = dict()
        self.epoch_cnt = 0

        self.tdoa_pvt = dict()
        self.tdoa_pvt_max_len = 10000


    # Task loop
    def run(self):
        self.restore_default_une_config()
        while True:
            if self.une_activated:
                self.usleep(1)
                continue
            self.usleep(1)

    def stop(self):
        self.quit()

    def restore_default_une_config(self):
        with open(os.path.join(os.path.dirname(__file__), '../une_configs/home_room.json'), 'r') as f:
            def_config = json.load(f)
            for anchor, pos, in def_config['anchors'].items():
                self.add_anchor_req([anchor, [str(pos) for pos in pos]])
            for tag, value in def_config['tags'].items():
                self.add_tag_req(tag)

            self.connect_nodes_req([def_config['anchors'], def_config['tags']])

    def check_anchor_in_list(self, ip):
        if len(self.anchor_list):
            for _ip in self.anchor_list:
                if _ip == ip:
                    return True
        return False

    def check_anchor_in_une_list(self, node_id):
        if len(self.anchor_une_list):
            for i, anchor in enumerate(self.anchor_une_list):
                if anchor[0] == node_id:
                    return i
        return -1

    def check_tag_in_une_list(self, node_id):
        if len(self.tag_une_list):
            for i, tag in enumerate(self.tag_une_list):
                if tag.get_name() is node_id:
                    return i
        return -1

    def check_anchor_has_settings(self, ip):
        if len(self.anchor_list):
            for anchor in self.anchor_list:
                if anchor[0] == ip:
                    value = anchor[1]
                    if (type(value) is dict) and (len(value) > 0):
                        return value
        return False

    @pyqtSlot(list)
    def add_anchor_req(self, data):
        node_id = data[0]
        pos_x = data[1][0]
        pos_y = data[1][1]
        pos_z = data[1][2]
        anchor_i = self.check_anchor_in_une_list(node_id)
        if anchor_i >= 0:
            self.anchor_une_list[anchor_i][1] = [pos_x, pos_y, pos_z]
        else:
            self.anchor_une_list.append([node_id, [pos_x, pos_y, pos_z]])
            self.sig_ui_update_anchor_resp.emit(data)
        return

    @pyqtSlot(str)
    def add_tag_req(self, node_id):
        tag_i = self.check_tag_in_une_list(node_id)
        if tag_i >= 0:
            pass
        else:
            self.tag_une_list.append(UneTag(node_id))
            self.sig_ui_update_tag_resp.emit([node_id, []])
        return

    # @brief:   Request from UI to refresh une status list-view
    # @args:    [[anchor_indexes], [tag_indexes]]
    @pyqtSlot(list)
    def connect_nodes_req(self, data):
        self.une_list = list()
        anchor_list = list()
        anchor_id = data[0]      # array of anchor indexes
        tag_id = data[1]  # array of anchor indexes

        for an in self.anchor_une_list:
            for an_id in anchor_id:
                if an[0] == an_id:
                    anchor_list.append(an)
        for i in tag_id:
            self.une_list.append([UneTag(i), anchor_list])
        if len(self.une_list):
            self.sig_ui_connect_une_resp.emit(self.une_list)
        return

    @pyqtSlot()
    def start_une(self):
        self.une_activated = True
        # Connect all nodes to une task
        for tag in self.une_list:
            for an in tag[1]:
                tag[0].add_anchor(an[0], float(an[1][0]), float(an[1][1]), float(an[1][2]))
            self.sig_une_add_new_tag.emit(tag[0])

    @pyqtSlot(list)         # from UneTsk
    def une_new_pvt(self, tags):
        for tag in tags:
            err = tag.get_err_code()
            dop = tag.get_dops()
            pvt = tag.get_pvt()

            pos = pvt[:3]  # Position [x, y, z]
            if err is UneErrors.none:
                vel = pvt[-1]    # Velocity [m/s]
                self.sig_ui_new_pvt.emit([tag.get_name(), pos])
                log.debug('TAG{} X:{:f}, Y:{:f}, Z:{:f}'.format(tag, pos[0], pos[1], pos[2]))
            else:
                log.debug('Error {} TAG{} X:{:f}, Y:{:f}, Z:{:f}'.format(err, tag, pos[0], pos[1], pos[2]))

    @pyqtSlot(object)    # from UDPServerTask.positNetwork to UneTask
    def twr_received(self, twr_info):
        if self.une_activated is not True:
            return
        if twr_info.Distance > 100 or twr_info.Distance < -100:
            return
        if twr_info.InitiatorID not in self.epoch_pvt:
            self.epoch_pvt[twr_info.InitiatorID] = dict()

        if twr_info.NodeID not in self.epoch_pvt[twr_info.InitiatorID]:
            (self.epoch_pvt[twr_info.InitiatorID])[twr_info.NodeID] = [twr_info.Distance, -128]
        if len(self.epoch_pvt[twr_info.InitiatorID]) >= 4:
            tag_meas = list()
            tag_meas.append(UneMeas(str(twr_info.InitiatorID), time.time(), self.epoch_pvt[twr_info.InitiatorID]))
            self.sig_une_upd_tag_meas.emit(tag_meas)
            # try:
            #     with open("./logs/crash_log.json", "r+") as file:
            #         data = json.load(file)
            #         data.update({str(self.epoch_cnt): ['21', self.epoch_pvt]})
            #         file.seek(0)
            #         json.dump(data, file)
            # except:
            #     with open("./logs/crash_log.json", "w") as file:
            #         json.dump({str(self.epoch_cnt): ['21', self.epoch_pvt]}, file)
            self.epoch_pvt[twr_info.InitiatorID] = dict()
            self.epoch_cnt += 1

            # log.debug("TWR_EPOCH " + str(self.epoch_cnt) + "\n" + str(self.epoch_pvt))
        return

    @pyqtSlot(object)  # from UDPServerTask.positNetwork to UneTask
    def tdoa_sync_received(self, tdoa_info):
        log_str = "TDOA_SYNC: NodeID={}, SyncID={}, NN={}, TX_TS={}, RX_TS={} ".format(
            tdoa_info.NodeID,
            tdoa_info.SyncID,
            tdoa_info.SyncNN,
            tdoa_info.SyncTxTS,
            tdoa_info.SyncRxTS)
        self.tdoa_wr_log(log_str)

    @pyqtSlot(object)  # from UDPServerTask.positNetwork to UneTask
    def tdoa_blink_received(self, tdoa_info):
        log_str = "TDOA_BLINK: NodeID={}, BlinkID={}, NN={}, TS={}".format(
            tdoa_info.NodeID,
            tdoa_info.BlinkID,
            tdoa_info.BlinkNN,
            tdoa_info.BlinkTS)
        self.tdoa_wr_log(log_str)

    def tdoa_wr_log(self, log_str):
        if len(self.tdoa_pvt) < self.tdoa_pvt_max_len:
            self.tdoa_pvt[str(len(self.tdoa_pvt))] = log_str
        elif len(self.tdoa_pvt) == self.tdoa_pvt_max_len:
            self.tdoa_pvt[str(len(self.tdoa_pvt))] = log_str
            with open('./logs/tdoa_data.json', 'w') as outfile:
                json.dump(self.tdoa_pvt, outfile)
                log.debug('TDOA DATA COLLECTED')
                sys.exit(0)
