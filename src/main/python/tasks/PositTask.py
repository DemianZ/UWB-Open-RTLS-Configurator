from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import atexit
import logging as log
import json
import os


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
class PositTask(QThread):
    sig_update_settings = pyqtSignal(list, name='PositTask_UpdateSettings')
    sig_add_device = pyqtSignal(list, name='PositTask_AddDevice')
    sig_remove_device = pyqtSignal(str, name='PositTask_RemoveDevice')
    sig_update_device = pyqtSignal(list, name='PositTask_UpdateDevice')

    sig_add_anchor_resp = pyqtSignal(str, name='PositTask_AddAnchorResp')
    sig_add_tag_resp = pyqtSignal(str, name='PositTask_AddTagResp')
    sig_connect_une_resp = pyqtSignal(list, name='PositTask_ConnectUneResp')

    def __init__(self):
        QThread.__init__(self)
        atexit.register(self.terminate)     # function to be executed on exit

        self.anchor_list = list()           # [[ip, {settings}]]

        self.anchor_une_list = list()       # [[id, [pos_x, pos_y, pos_z]]]
        self.tag_une_list = list()          # [id]
        self.une_list = list()              # [[tag_id ,[anchor_id,...]]]
        self.une_activated = False

    # Task loop
    def run(self):
        self.restore_default_une_config()
        while True:
            if self.une_activated:
                self.usleep(1)
                continue

            self.usleep(100)

    def stop(self):
        self.quit()

    def restore_default_une_config(self):
        with open(os.path.join(os.path.dirname(__file__), '../modules/defaultUneConfig.json'), 'r') as f:
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
                if tag[0] == node_id:
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

    # @brief: Slot is fired every time hello cmd received from server
    @pyqtSlot(str)
    def slot_cmd_hello(self, ip):
        if not self.check_anchor_in_list(ip):
            self.anchor_list.append([ip])      # empty settings
            self.sig_add_device.emit([ip, dict()])
            return

    # @brief: Slot is fired every time device settings received from server
    #         Adds settings to local dict
    #         If new device, emit signal to UI for adding it to device list.
    @pyqtSlot(list)
    def slot_cmd_get_settings(self, data):
        ip = data[0]
        settings = data[1]
        self.anchor_list[str(ip)] = settings
        if not self.check_anchor_has_settings(ip):
            self.sig_add_device.emit([ip, settings])
        self.sig_update_device.emit([ip, settings['NodeID', 'NodeType', 'RTLSMode', 0, 0]])

    @pyqtSlot(list)
    def slot_twr_ranging(self, data):
        ip = data[0]
        monitoring = data[1]
        log.debug(str(ip) + ': ' + monitoring.TWR.Distance)

    @pyqtSlot(list)
    def add_anchor_req(self, data):
        node_id = data[0]
        pos_x = data[1][0]
        pos_y = data[1][0]
        pos_z = data[1][0]
        anchor_i = self.check_anchor_in_une_list(node_id)
        if anchor_i >= 0:
            self.anchor_une_list[anchor_i][1] = [pos_x, pos_y, pos_z]
        else:
            self.anchor_une_list.append([node_id, [pos_x, pos_y, pos_z]])
            self.sig_add_anchor_resp.emit(node_id)
        return

    @pyqtSlot(str)
    def add_tag_req(self, node_id):
        tag_i = self.check_tag_in_une_list(node_id)
        if tag_i >= 0:
            pass
        else:
            self.tag_une_list.append(node_id)
            self.sig_add_tag_resp.emit(node_id)
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
            self.une_list.append([i, anchor_list])
        if len(self.une_list):
            self.sig_connect_une_resp.emit(self.une_list)
        return

    # @pyqtSlot(str)
    # def ui_get_settings(self, ip):
    #     settings = self.check_client_has_settings(ip)
    #     if settings:
    #         self.sig_update_settings.emit([ip, settings])

