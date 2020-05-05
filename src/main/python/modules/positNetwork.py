from PyQt5.QtCore import QObject, pyqtSignal
import logging as log
from proto import Monitoring_pb2
from proto import Settings_pb2
from modules.wake import Wake
from google.protobuf.json_format import MessageToDict

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
    sig_cmd_hello = pyqtSignal(str, name='PositNetwork_CmdHello')
    sig_cmd_get_settings = pyqtSignal(list, name='PositNetwork_CmdGetSettings')
    sig_twr_ranging = pyqtSignal(list, name='PositNetwork_CmdTwrRanging')

    PB_TWR_MSGTYPE_NONE = 0
    PB_TWR_MSGTYPE_TWR = 1

    def __init__(self):
        super().__init__()
        self.monitoring = Monitoring_pb2.Monitoring()
        self.settings = Settings_pb2.Settings()

    #
    # @brief:   Function for parsing wake cmd and data from server clients
    #           Function is called from UDPServerTask after successfully parsing wake packet
    # @args:    str(ip_address), int(cmd), list(data)
    #
    def rx_callback(self, ip, cmd, data):
        if cmd == Wake.CMD_GET_SETTINGS_RESP:
            return self.get_settings_callback(ip, data)
        elif cmd == Wake.CMD_TWR_RANGING:
            return self.twr_ranging_callback(ip, data)
        else:
            return False

    def hello_callback(self, ip):
        self.sig_cmd_hello.emit(ip)
        return True

    def get_settings_callback(self, ip, data):
        try:
            self.settings.ParseFromString(bytes(data))
            settings_dict = MessageToDict(self.settings)
            self.sig_cmd_get_settings.emit([ip, {settings_dict}])
        except Exception as e:
            log.error(e)
            return e
        return True

    def twr_ranging_callback(self, ip, data):
        try:
            self.monitoring.ParseFromString(bytes(data))
            monitoring_dict = MessageToDict(self.settings)
        except Exception as e:
            return e
        if self.monitoring.TWR.MessageType == PositNetwork.PB_TWR_MSGTYPE_TWR:
            self.sig_twr_ranging.emit([ip, monitoring_dict])
        return True
