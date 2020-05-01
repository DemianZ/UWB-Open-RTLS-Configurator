import logging as log
import datetime
import struct
from proto import Monitoring_pb2

# Config logger to write to file
logger = log.getLogger('data_log')
logger.setLevel(log.DEBUG)
fh = log.FileHandler('logs/pp_' + (datetime.datetime.now()).strftime("%d%m%Y%H%M%S") + '.log')
fh.setLevel(log.DEBUG)
formatter = log.Formatter('%(message)s\t%(asctime)s.%(msecs)03d', datefmt='%H:%M:%S')
fh.setFormatter(formatter)
logger.addHandler(fh)


class Posit:
    PB_TWR_MSGTYPE_NONE = 0
    PB_TWR_MSGTYPE_RANGING = 1

    def __init__(self):

        pass

    @staticmethod
    def parse_udp_message(data, ip):
        monitoring = Monitoring_pb2.Monitoring()
        try:
            monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e

        if monitoring.twr_message_type == Posit.PB_TWR_MSGTYPE_RANGING:
            logger.debug('ip: ' + str(ip[0]) + '\n' + str(monitoring))

        return monitoring

    @staticmethod
    def parse_serial_message(data):
        monitoring = Monitoring_pb2.Monitoring()
        try:
            monitoring.ParseFromString(bytes(data))
        except Exception as e:
            return e

        if monitoring.twr_message_type == Posit.PB_TWR_MSGTYPE_RANGING:
            logger.debug('TWR_Ranging:\n' + str(monitoring))

        return monitoring
