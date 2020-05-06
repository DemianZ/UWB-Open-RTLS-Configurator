from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import logging as log
from proto import Monitoring_pb2
from proto import Settings_pb2
from modules.wake import Wake
from modules.positSerial import PositSerial
from google.protobuf.json_format import MessageToDict, ParseDict

# Config logger to write to file
logger = log.getLogger('data_log')
logger.setLevel(log.DEBUG)
# fh = log.FileHandler('logs/pp_' + (datetime.datetime.now()).strftime("%d%m%Y%H%M%S") + '.log')
# fh.setLevel(log.DEBUG)
# formatter = log.Formatter('%(message)s\t%(asctime)s.%(msecs)03d', datefmt='%H:%M:%S')
# fh.setFormatter(formatter)
# logger.addHandler(fh)


#
# @brief:
#   Class for handling incoming packets from server clients
# @classSignals:
#   PositNetwork_CmdHello:          args: str (ip)
#   PositNetwork_CmdGetSettings:    args: list [ip, {settings}]
#   PositNetwork_CmdSetSettings:    args: str (ip)
#   PositNetwork_CmdTwrRanging:     args: list [ip, {monitoring}]
#
class PositNetwork(QObject):
    sig_posit_settings_received = pyqtSignal(str, dict, name='PositNetwork_PositSettingsReceived')
    sig_posit_hello_received = pyqtSignal(str, name='PositNetwork_PositHelloReceived')
    sig_posit_twr_received = pyqtSignal(str, float, name='PositNetwork_PositTwrReceived')

    sig_udp_transmit = pyqtSignal(str, list, name='PositNetwork_UdpTransmit')

    sig_ui_update_settings = pyqtSignal(dict, name='PositNetwork_UIUpdateSettings')
    sig_ui_add_device = pyqtSignal(list, name='PositNetwork_UIAddDevice')
    sig_ui_remove_device = pyqtSignal(str, name='PositNetwork_UIRemoveDevice')
    sig_ui_update_device = pyqtSignal(list, name='PositNetwork_UIUpdateDevice')

    PB_TWR_MSGTYPE_NONE = 0
    PB_TWR_MSGTYPE_TWR = 1

    def __init__(self):
        super().__init__()
        self.wake = Wake()
        self.monitoring = Monitoring_pb2.Monitoring()
        self.settings = Settings_pb2.Settings()
        self.net_device_list = list()  # [[NodeID, IP, Type, Mode, Rx/Tx, Error]]

    def process(self, address, data):
        ip = address[0]
        cmd_res = self.wake.process(data)
        if cmd_res is not None:
            # log.debug('CMD_' + str(hex(cmd_res['cmd'])) +
            #           ' DATA: ' + str(' '.join('{:02X}'.format(c) for c in cmd_res['data'])))
            return self.rx_callback(ip, cmd_res['cmd'], cmd_res['data'])

    # @brief:   Function for parsing wake cmd and data from server clients
    #           Function is called from UDPServerTask after successfully parsing wake packet
    # @args:    str(ip_address), int(cmd), list(data)
    #
    def rx_callback(self, ip, cmd, data):
        if cmd == Wake.CMD_I_AM_HERE_REQ:
            return self.hello_callback(ip)
        elif cmd == Wake.CMD_GET_SETTINGS_RESP:
            return self.get_settings_callback(ip, data)
        elif cmd == Wake.CMD_TWR_RANGING:
            return self.twr_ranging_callback(ip, data)
        else:
            return False

    # @brief: Slot is fired every time hello cmd received from server
    def hello_callback(self, ip):
        if self.check_ip_in_network_list(ip) < 0:
            self.net_device_list.append([None, ip, None, None, None, None])  # empty settings
            self.sig_ui_add_device.emit(self.net_device_list[len(self.net_device_list) - 1])
        return True

    # @brief: Slot is fired every time device settings received from server
    #         Adds settings to local dict
    #         If new device, emit signal to UI for adding it to device list.
    def get_settings_callback(self, ip, data):
        try:
            self.settings.ParseFromString(bytes(data))
            sets = MessageToDict(self.settings)
            # Check if new device in system
            i = self.check_ip_in_network_list(ip)
            if i < 0:
                self.net_device_list.append([sets['NodeID'],
                                             ip,
                                             sets['NodeType'],
                                             sets['RTLSMode'],
                                             None, None])  # empty settings
                self.sig_ui_add_device.emit(self.net_device_list[len(self.net_device_list)-1])
            else:
                self.net_device_list[i] = [sets['NodeID'],
                                           ip,
                                           sets['NodeType'],
                                           sets['RTLSMode'],
                                           None, None]

            self.sig_ui_update_device.emit(self.net_device_list[i])

            for key in PositSerial.ip32_keys:
                if key in sets:
                    sets[key] = PositSerial.ip32_to_str(int(sets[key]))

            self.sig_ui_update_settings.emit(sets)
        except Exception as e:
            log.error(e)
            return e
        return True

    def twr_ranging_callback(self, ip, data):
        try:
            self.monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e
        if self.monitoring.TWR.MessageType == PositNetwork.PB_TWR_MSGTYPE_TWR:
            self.sig_posit_twr_received.emit(ip, self.monitoring.TWR.Distance)
            # log.debug(str(ip) + ': ' + str(self.monitoring.TWR.Distance))
        return True

    # Requests to device
    def get_settings_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_GET_SETTINGS_REQ, [])
        self.sig_udp_transmit.emit(ip, buf)

    def set_settings_req(self, ip, settings_dict):
        settings_pb = Settings_pb2.Settings()
        ParseDict(settings_dict, settings_pb)
        settings_string = settings_pb.SerializeToString()
        buf = self.wake.prepare(Wake.CMD_SET_SETTINGS_REQ, settings_string)
        self.sig_udp_transmit.emit(ip, buf)

    def set_default_settings_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_SET_DEF_SETTINGS_REQ, [])
        self.sig_udp_transmit(ip, buf)

    def check_ip_in_network_list(self, ip):
        if len(self.net_device_list):
            for i, device in enumerate(self.net_device_list):
                if device[1] == ip:
                    return i
        return -1
