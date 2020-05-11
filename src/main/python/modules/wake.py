class Wake:
    WAIT_FEND = 0
    WAIT_ADDR = 1
    WAIT_CMD = 2
    WAIT_NBT1 = 3
    WAIT_NBT2 = 4
    WAIT_DATA = 5
    WAIT_CRC1 = 6
    WAIT_CRC2 = 7
    WAIT_FIN = 8

    FEND = 0xC0  # Frame END
    FESC = 0xDB  # Frame ESCape
    TFEND = 0xDC  # Transposed Frame END
    TFESC = 0xDD  # Transposed Frame ESCape

    WAKE_PROCESS = 0
    WAKE_SUCCESS = 1
    WAKE_ERROR = 2
    WAKE_ERROR_CRC = 2

    CRC16_POLY = 0x1021

    CMD_I_AM_HERE_REQ = 0x31
    CMD_I_AM_HERE_RESP = 0x32
    CMD_REBOOT_REQ = 0x33
    CMD_REBOOT_RESP = 0x34
    CMD_GET_SETTINGS_REQ = 0x11
    CMD_GET_SETTINGS_RESP = 0x12
    CMD_SET_SETTINGS_REQ = 0x13
    CMD_SET_SETTINGS_RESP = 0x14
    CMD_SET_DEF_SETTINGS_REQ = 0x15
    CMD_SET_DEF_SETTINGS_RESP = 0x16
    CMD_TWR_RANGING = 0x21

    def __init__(self):
        self.len = 0
        self.sta = 0
        self.crc = 0
        self.pre = 0
        self.err_cnt = 0
        self.dbuf = []
        self.crc_pack = 0
        self.cmd = None
        pass

    @staticmethod
    def crc_16(init, byte):
        crc = init
        crc ^= byte << 8
        for index in range(0, 8):
            if crc & 0x8000:
                crc = (crc << 1) ^ Wake.CRC16_POLY
            else:
                crc = crc << 1
            crc &= 0xFFFF
        return crc

    def do_crc16(self, init, byte):
        crc16 = self.crc_16(init, byte)
        return crc16

    @staticmethod
    def _buf_add(pp, byte):
        if byte == Wake.FEND:
            pp.append(Wake.FESC)
            pp.append(Wake.TFEND)
        elif byte == Wake.FESC:
            pp.append(Wake.FESC)
            pp.append(Wake.TFESC)
        else:
            pp.append(byte)
        return

    def prepare(self, cmd, data):
        out = []
        # start
        crc16 = self.do_crc16(0xFFFF, Wake.FEND)
        out.append(Wake.FEND)
        # cmd
        tmp = cmd & 0x7F
        crc16 = self.do_crc16(crc16, tmp)
        self._buf_add(out, tmp)
        # len
        tmp = len(data)
        crc16 = self.do_crc16(crc16, tmp & 0xFF)
        self._buf_add(out, tmp & 0xFF)
        crc16 = self.do_crc16(crc16, (tmp >> 8))
        self._buf_add(out, (tmp >> 8))
        # data
        for i in range(len(data)):
            tmp = data[i]
            crc16 = self.do_crc16(crc16, tmp)
            self._buf_add(out, tmp)
        # crc16
        self._buf_add(out, crc16 & 0xFF)
        self._buf_add(out, (crc16 >> 8) & 0xFF)

        return out

    def rx_handler(self, byte):

        if byte == Wake.FEND:
            self.pre = byte
            self.crc = 0
            self.sta = Wake.WAIT_CMD
            self.crc = self.do_crc16(0xFFFF, byte)
            return Wake.WAKE_PROCESS

        if self.sta == Wake.WAIT_FEND:
                return Wake.WAKE_PROCESS

        pre_temp = self.pre
        self.pre = byte

        if pre_temp == Wake.FESC:
            if byte == Wake.TFESC:
                byte = Wake.FESC
            elif byte == Wake.TFEND:
                byte = Wake.FEND
            else:
                self.sta = Wake.WAIT_FEND
                self.err_cnt += 1
                return Wake.WAKE_ERROR
        else:
            if byte == Wake.FESC:
                return Wake.WAKE_PROCESS

        x = int(self.sta)

        if x == Wake.WAIT_CMD:
            if byte & 0x80:
                self.sta = Wake.WAIT_FEND
                self.err_cnt += 1
                return Wake.WAKE_ERROR
            self.cmd = byte
            self.crc = self.do_crc16(self.crc, byte)
            self.sta = Wake.WAIT_NBT1
            return Wake.WAKE_PROCESS
        elif x == Wake.WAIT_NBT1:
            self.len = byte
            self.crc = self.do_crc16(self.crc, byte)
            self.dbuf = []
            self.sta = Wake.WAIT_NBT2
            return Wake.WAKE_PROCESS
        elif x == Wake.WAIT_NBT2:
            self.len += (byte << 8)
            self.crc = self.do_crc16(self.crc, byte)
            self.dbuf = []
            self.sta = Wake.WAIT_DATA
            return Wake.WAKE_PROCESS
        elif x == Wake.WAIT_DATA:
            if len(self.dbuf) < self.len:
                self.dbuf.append(byte)
                self.crc = self.do_crc16(self.crc, byte)
                return Wake.WAKE_PROCESS
            self.sta = Wake.WAIT_CRC1
            self.crc_pack = byte
            self.sta = Wake.WAIT_CRC2
            return Wake.WAKE_PROCESS

        elif x == Wake.WAIT_CRC1:
            self.crc_pack = byte
            self.sta = Wake.WAIT_CRC2
            return Wake.WAKE_PROCESS

        elif x == Wake.WAIT_CRC2:
            self.crc_pack += (byte << 8)
            if self.crc != self.crc_pack:
                self.sta = Wake.WAIT_FEND
                self.err_cnt += 1
                return Wake.WAKE_ERROR_CRC
            self.sta = Wake.WAIT_FIN
            return Wake.WAKE_SUCCESS
        else:          # default
            return Wake.WAKE_ERROR

    def process(self, data):
        for v in data:              # parse incoming USB packet
            if self.rx_handler(v) == Wake.WAKE_SUCCESS:
                return {'cmd': self.cmd, 'data': self.dbuf}
        return None
