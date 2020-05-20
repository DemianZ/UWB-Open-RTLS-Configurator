from PyQt5.QtCore import QRunnable, pyqtSignal, QObject, pyqtSlot, QThreadPool
import logging as log
from proto import Monitoring_pb2
from proto import Settings_pb2
from modules.wake import Wake
from modules.PositSerial import PositSerial
from google.protobuf.json_format import MessageToDict, ParseDict
import binascii

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
    class RxCallback(QRunnable):
        def __init__(self, fn, *args, **kwargs):
            super().__init__()
            self.fn = fn
            self.args = args
            self.kwargs = kwargs

        @pyqtSlot()
        def run(self):
            self.fn(*self.args, **self.kwargs)

    sig_posit_settings_received = pyqtSignal(str, dict, name='PositNetwork_PositSettingsReceived')
    sig_posit_hello_received = pyqtSignal(str, name='PositNetwork_PositHelloReceived')
    sig_posit_twr_received = pyqtSignal(object, name='PositNetwork_PositTwrReceived')

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

        self.thread_pool = QThreadPool()
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
            callback = self.RxCallback(self.hello_callback, ip)
            self.thread_pool.start(callback)

        elif cmd == Wake.CMD_GET_SETTINGS_RESP:
            callback = self.RxCallback(self.get_settings_callback, ip, data)
            self.thread_pool.start(callback)

        if cmd == Wake.CMD_SET_SETTINGS_RESP:
            callback = self.RxCallback(self.set_settings_callback, ip, data)
            self.thread_pool.start(callback)

        elif cmd == Wake.CMD_TWR_RANGING:
            callback = self.RxCallback(self.twr_ranging_callback, ip, data)
            self.thread_pool.start(callback)

        elif cmd == Wake.CMD_REBOOT_RESP:
            log.debug("REBOOT OK: {}".format(ip))

        else:
            return False

    # @brief: Slot is fired every time hello cmd received from server
    def hello_callback(self, ip):
        if self.check_ip_in_network_list(ip) < 0:
            self.net_device_list.append([None, ip, None, None, None, None])  # empty settings
            self.sig_ui_add_device.emit(self.net_device_list[len(self.net_device_list) - 1])
            log.debug("NEW HELLO: {}".format(ip))
        buf = self.wake.prepare(Wake.CMD_I_AM_HERE_RESP, [])
        self.sig_udp_transmit.emit(ip, buf)
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
        log.debug("SETTINGS GET OK: {}".format(ip))
        return True

    def twr_ranging_callback(self, ip, data):
        try:
            self.monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e
        twr_info = self.monitoring.TWR
        self.sig_posit_twr_received.emit(twr_info)
        log.debug("TWR: NodeID={}, InitID={}, D={}, PollNN={}, RespNN={}, FinalNN={}".format(
            twr_info.NodeID,
            twr_info.InitiatorID,
            twr_info.Distance,
            twr_info.PollNN,
            twr_info.ResponseNN,
            twr_info.FinalNN))
        return True

    def set_settings_callback(self, ip, data):
        try:
            self.monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e
        log.debug("SETTINGS SET OK: {}".format(ip))

    # Requests to device
    def get_settings_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_GET_SETTINGS_REQ, [])
        self.sig_udp_transmit.emit(ip, buf)

    def set_settings_req(self, ip, settings_dict):
        if 'ConnectedAnchors' in settings_dict:
            sett_str = settings_dict['ConnectedAnchors']
            sett_list = sett_str[1:-1].split(',')
            settings_dict['ConnectedAnchors'] = [int(sett) for sett in sett_list]

        settings_pb = Settings_pb2.Settings()
        ParseDict(settings_dict, settings_pb)
        settings_string = settings_pb.SerializeToString()
        buf = self.wake.prepare(Wake.CMD_SET_SETTINGS_REQ, settings_string)
        self.sig_udp_transmit.emit(ip, buf)

    def set_default_settings_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_SET_DEF_SETTINGS_REQ, [])
        self.sig_udp_transmit.emit(ip, buf)

    #
    def reboot_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_REBOOT_REQ, [])
        self.sig_udp_transmit.emit(ip, buf)

    def check_ip_in_network_list(self, ip):
        if len(self.net_device_list):
            for i, device in enumerate(self.net_device_list):
                if device[1] == ip:
                    return i
        return -1
