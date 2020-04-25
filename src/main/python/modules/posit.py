import logging as log
import datetime

# Config logger to write to file
logger = log.getLogger('data_log')
logger.setLevel(log.DEBUG)
# fh = log.FileHandler('logs/pp_' + (datetime.datetime.now()).strftime("%d%m%Y%H%M%S") + '.log')
# fh.setLevel(log.DEBUG)
# formatter = log.Formatter('%(message)s\t%(asctime)s.%(msecs)03d', datefmt='%H:%M:%S')
# fh.setFormatter(formatter)
# logger.addHandler(fh)


class Posit:
    def __init__(self):
        pass

    @staticmethod
    def parse_udp_message(data, ip):
        if data[:4].decode() == 'SYNC':
            sync_n = int(data[6])
            sync_ts = int.from_bytes(data[9:14], "little")
            logger.debug(str(sync_n) + '\t' + str(sync_ts) + '\t' + str(ip[0]))
        pass
