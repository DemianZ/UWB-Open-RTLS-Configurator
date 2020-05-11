from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import logging as log
from proto import Monitoring_pb2
from proto import Settings_pb2
from google.protobuf.json_format import MessageToDict, ParseDict
from modules.wake import Wake

# Config logger to write to file
logger = log.getLogger('data_log')
logger.setLevel(log.DEBUG)
# fh = log.FileHandler('logs/pp_' + (datetime.datetime.now()).strftime("%d%m%Y%H%M%S") + '.log')
# fh.setLevel(log.DEBUG)
# formatter = log.Formatter('%(message)s\t%(asctime)s.%(msecs)03d', datefmt='%H:%M:%S')
# fh.setFormatter(formatter)
# logger.addHandler(fh)


class PositSerial(QObject):
    ip32_keys = ['DeviceIp', 'SubnetMask', 'GatewayIp', 'ServerIp']

    PB_TWR_MSGTYPE_NONE = 0
    PB_TWR_MSGTYPE_RANGING = 1

    sig_serial_write = pyqtSignal(list, name='PositSerial_SettingsWrite')
    sig_posit_settings_received = pyqtSignal(dict, name='PositSerial_SettingsReceived')
    sig_posit_twr_received = pyqtSignal(object, name='PositSerial_PositTwrReceived')

    def __init__(self, tsk_serial):
        super().__init__()
        self.tsk_serial = tsk_serial
        self.wake = Wake()
        self.monitoring = Monitoring_pb2.Monitoring()
        self.settings = Settings_pb2.Settings()
        pass

    def process(self, data):
        cmd_res = self.wake.process(data)
        if cmd_res is not None:
            # log.debug('CMD_' + str(hex(cmd_res['cmd'])) +
            #           ' DATA: ' + str(' '.join('{:02X}'.format(c) for c in cmd_res['data'])))
            self.rx_callback(cmd_res['cmd'], cmd_res['data'])

    def rx_callback(self, cmd, data):
        if cmd == Wake.CMD_GET_SETTINGS_RESP:
            return self.get_settings_callback(data)

        if cmd == Wake.CMD_SET_SETTINGS_RESP:
            return self.set_settings_callback(data)

        if cmd == Wake.CMD_REBOOT_RESP:
            return self.reboot_callback(data)

        if cmd == Wake.CMD_TWR_RANGING:
            return self.twr_ranging_callback(data)

        else:
            return False

    @pyqtSlot()
    def get_settings(self):
        data = self.wake.prepare(Wake.CMD_GET_SETTINGS_REQ, [])
        self.sig_serial_write.emit(data)

    @pyqtSlot(dict)
    def set_settings(self, settings_dict):
        if 'ConnectedAnchors' in settings_dict:
            sett_str = settings_dict['ConnectedAnchors']
            sett_list = sett_str[1:-1].split(',')
            settings_dict['ConnectedAnchors'] = [int(sett) for sett in sett_list]

        settings_pb = Settings_pb2.Settings()
        ParseDict(settings_dict, settings_pb)
        settings_string = settings_pb.SerializeToString()
        data = self.wake.prepare(Wake.CMD_SET_SETTINGS_REQ, settings_string)
        self.sig_serial_write.emit(data)

    @pyqtSlot()
    def set_default_settings(self):
        data = self.wake.prepare(Wake.CMD_SET_DEF_SETTINGS_REQ, [])
        self.sig_serial_write.emit(data)

    @pyqtSlot()
    def reboot(self):
        data = self.wake.prepare(Wake.CMD_REBOOT_REQ, [])
        self.sig_serial_write.emit(data)

    def get_settings_callback(self, data):
        try:
            self.settings.ParseFromString(bytes(data))
        except Exception as e:
            return e
        settings_dict = MessageToDict(self.settings)

        for key in self.ip32_keys:
            if key in settings_dict:
                settings_dict[key] = self.ip32_to_str(int(settings_dict[key]))
        logger.debug('dev_settings:\n' + str(self.settings))
        self.sig_posit_settings_received.emit(settings_dict)
        return True

    def twr_ranging_callback(self, data):
        try:
            self.monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e

        twr_info = self.monitoring.TWR
        self.sig_posit_twr_received.emit(twr_info)
        log.debug("TWR: RespID={}, InitID={}, Distance={} PollNN={}, RespNN={}, FinalNN={}".format(
            twr_info.NodeID,
            twr_info.InitiatorID,
            twr_info.Distance
            ,
            twr_info.PollNN,
            twr_info.ResponseNN,
            twr_info.FinalNN))
        return True

    def set_settings_callback(self, data):
        try:
            self.settings.ParseFromString(bytes(data))
        except Exception as e:
            return e
        logger.debug('SETTINGS SET OK: {}'.format(self.settings.NodeID))
        return True

    def reboot_callback(self, data):
        try:
            self.settings.ParseFromString(bytes(data))
        except Exception as e:
            return e
        logger.debug('REBOOT OK: {}'.format(self.settings.NodeID))
        return True

    @staticmethod
    def ip32_to_str(ip32):
        ip = str()
        for i in range(3):
            ip = ip + str((ip32 >> 8*i) & 0xFF) + '.'
        ip = ip + str((ip32 >> 24) & 0xFF)
        return ip

    @staticmethod
    def str_to_ip32(ip_str):
        ip = 0
        ip_str = ip_str.split('.')
        for i in range(4):
            ip = ip + (int(ip_str[i]) << 8*i)
        return ip
