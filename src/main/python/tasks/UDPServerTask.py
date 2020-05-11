from PyQt5.QtCore import QThread, pyqtSlot, QMutex, QThreadPool, QRunnable
import atexit
import socket
import logging as log
from modules.PositNetwork import PositNetwork

UDP_SRV = "10.90.90.99"     # Your PC's IP
UDP_PORT = 30005


# @brief:   USP server task for network communication with RTLS nodes
class UdpServerTask(QThread):
    class TxCallback(QRunnable):
        def __init__(self, fn, *args, **kwargs):
            super().__init__()
            self.fn = fn
            self.args = args
            self.kwargs = kwargs

        @pyqtSlot()
        def run(self):
            self.fn(*self.args, **self.kwargs)

    def __init__(self, ui):
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

        self.ui = ui
        self.sock = None
        self.ip = UDP_SRV
        self.port = UDP_PORT
        self.posit = PositNetwork()
        self.client_list = list()   # ip addresses
        self.thread_pool = QThreadPool()

    # Task loop
    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while not self.bind_port(self.ip, self.port):
            self.sleep(5)

        while True:
            try:
                data, address = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
                if address not in self.client_list:
                    self.client_list.append(address)
                if len(data):
                    self.posit.process(address, data)
            except OSError as err:
                log.error(err)
            self.usleep(1)

    def stop(self):
        self.quit()

    def bind_port(self, ip, port):
        try:
            self.sock.bind((ip, port))
        except OSError as e:
            if e.errno == 49:
                log.debug("SOCKET ERROR: {} {}".format(str(e.errno), e.strerror))
                pass
            return False
        return True

    @pyqtSlot(tuple, dict)
    def udp_transmit(self, ip, data):
        callback = self.TxCallback(self.sock.sendto, bytes(data), (ip, UDP_PORT))
        self.thread_pool.start(callback)


