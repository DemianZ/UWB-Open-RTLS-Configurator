from PyQt5.QtCore import QThread
import atexit
import socket
import logging as log
from modules.posit import Posit
from modules.wake import Wake

UDP_SRV = "10.90.90.99"     # Your PC's IP
UDP_CLIENT = "10.90.0.1"    # Device IP
UDP_PORT = 30005


class UdpServerTask(QThread):
    def __init__(self, ui):
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

        self.ui = ui
        self.sock = None
        self.ip = UDP_SRV
        self.client_ip = UDP_CLIENT
        self.port = UDP_PORT
        self.wake = Wake()

    # Task loop
    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while not self.bind_port(self.ip, self.port):
            self.sleep(5)

        while True:
            data, address = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
            if len(data):
                cmd_res = self.wake.process(data)
                if cmd_res is not None:
                    log.debug('CMD_' + str(hex(cmd_res['cmd'])) +
                              ' DATA: ' + str(' '.join('{:02X}'.format(c) for c in cmd_res['data'])))
                    Posit.rx_callback(cmd_res['cmd'], cmd_res['data'])
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
