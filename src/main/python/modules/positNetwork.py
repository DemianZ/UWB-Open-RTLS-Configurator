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


class PositNetwork(QObject):

    PB_TWR_MSGTYPE_NONE = 0
    PB_TWR_MSGTYPE_TWR = 1

    sig_posit_settings_received = pyqtSignal(dict, name='PositNetwork_SettingsReceived')
    sig_posit_hello_received = pyqtSignal(dict, name='PositNetwork_HelloReceived')

    def __init__(self):
        super().__init__()
        self.monitoring = Monitoring_pb2.Monitoring()
        self.settings = Settings_pb2.Settings()
        self.client_list = list()

    def rx_callback(self, ip, cmd, data):
        if cmd == Wake.CMD_GET_SETTINGS_RESP:
            return self.get_settings_callback(ip, data)
        if cmd == Wake.CMD_TWR_RANGING:
            return self.twr_ranging_callback(data)
        else:
            return False

    def hello_callback(self):
        self.sig_posit_hello_received.emit()
        return True

    def get_settings_callback(self, ip, data):
        try:
            self.settings.ParseFromString(bytes(data))
        except Exception as e:
            return e
        settings_dict = MessageToDict(self.settings)
        logger.debug('dev_settings:\n' + str(self.settings))
        self.sig_posit_hello_received.emit(settings_dict)
        return True

    def twr_ranging_callback(self, data):
        try:
            self.monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e

        if self.monitoring.TWR.MessageType == PositNetwork.PB_TWR_MSGTYPE_TWR:
            logger.debug('Net Ranging:\t' + str(self.monitoring.TWR.Distance))
        return True

    def check_new_client(self, ip):
        if len(self.client_list):
            for client in self.client_list:
                if client == ip:
                    return False
        self.client_list.append(ip)
        return True
