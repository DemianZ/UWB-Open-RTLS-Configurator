from PyQt5.QtCore import QThread
import atexit
import socket
import logging as log
from modules.positNetwork import PositNetwork
from modules.wake import Wake

UDP_SRV = "10.90.90.99"     # Your PC's IP
UDP_PORT = 30005


class UdpServerTask(QThread):
    def __init__(self, ui):
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

        self.ui = ui
        self.sock = None
        self.ip = UDP_SRV
        self.port = UDP_PORT
        self.wake = Wake()
        self.posit = PositNetwork()
        self.client_list = list()   # ip addresses

    # Task loop
    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while not self.bind_port(self.ip, self.port):
            self.sleep(5)

        while True:
            data, address = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
            ip = address[0]
            if len(data):
                cmd_res = self.wake.process(data)
                if cmd_res is not None:
                    log.debug('CMD_' + str(hex(cmd_res['cmd'])) +
                              ' DATA: ' + str(' '.join('{:02X}'.format(c) for c in cmd_res['data'])))
                    if self.posit.rx_callback(address, cmd_res['cmd'], cmd_res['data']) is True:
                        if self.posit.check_new_client(ip) is True:
                            self.get_settings(ip)

            self.usleep(50)

    def bind_port(self, ip, port):
        try:
            self.sock.bind((ip, port))
        except OSError as e:
            if e.errno == 49:
                # log.debug("ERRNO_" + str(e.errno) + " -> " + e.strerror)
                pass
            return False
        return True

    def get_settings(self, ip):
        buf = self.wake.prepare(Wake.CMD_GET_SETTINGS_REQ, [])
        self.sock.sendto(bytearray(buf), (ip, UDP_PORT))

