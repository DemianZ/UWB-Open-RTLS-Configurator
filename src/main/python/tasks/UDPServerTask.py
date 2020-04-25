from PyQt5.QtCore import QThread
import atexit
import socket
import logging as log
from modules.posit import Posit

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

    # Task loop
    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while not self.bind_port(self.ip, self.port):
            self.sleep(5)

        while True:
            data, address = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
            # log.info(address[0] + '\t' + str(data))
            Posit.parse_udp_message(data, address)

    def bind_port(self, ip, port):
        try:
            self.sock.bind((ip, port))
        except OSError as e:
            if e.errno == 49:
                log.error("ERRNO: " + str(e.errno) + " ERRSTR: " + e.strerror)
            return False
        return True
