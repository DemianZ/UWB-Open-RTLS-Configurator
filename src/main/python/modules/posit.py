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


class Posit(QObject):

    PB_TWR_MSGTYPE_NONE = 0
    PB_TWR_MSGTYPE_RANGING = 1

    sig_posit_settings_received = pyqtSignal(dict, name='Posit_SettingsReceived')

    def __init__(self):
        super().__init__()
        self.monitoring = Monitoring_pb2.Monitoring()
        self.settings = Settings_pb2.Settings()
        pass

    def rx_callback(self, cmd, data):
        if cmd == Wake.CMD_GET_SETTINGS_RESP:
            self.get_settings_callback(data)
        if cmd == Wake.CMD_TWR_RANGING:
            self.twr_ranging_callback(data)
        else:
            pass

    def get_settings_callback(self, data):
        try:
            self.settings.ParseFromString(bytes(data))
        except Exception as e:
            return e
        settings_dict = MessageToDict(self.settings)
        logger.debug('dev_settings:\n' + str(self.settings))
        self.sig_posit_settings_received.emit(settings_dict)

    def twr_ranging_callback(self, data):
        try:
            self.monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e

        if self.monitoring.twr_message_type == Posit.PB_TWR_MSGTYPE_RANGING:
            logger.debug('twr_ranging_dist:\t' + str(self.monitoring.twr_distance))
