from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal, QRunnable
import atexit
import socket
import logging as log
from proto import Monitoring_pb2
from proto import Settings_pb2
from modules.PositNetwork import PositNetwork
from google.protobuf.json_format import ParseDict
from modules.wake import Wake

TCP_SRV = "10.90.90.99"     # Your PC's IP
TCP_PORT = 30005


# @brief:   USP server task for network communication with RTLS nodes
class TcpServerTask(QThread):

    sig_tcp_transmit = pyqtSignal(str, list, name='TcpServerTask_TcpTransmit')

    class TxCallback(QRunnable):
        def __init__(self, fn, *args, **kwargs):
            super().__init__()
            self.fn = fn
            self.args = args
            self.kwargs = kwargs

        @pyqtSlot()
        def run(self):
            self.fn(*self.args, **self.kwargs)

    def __init__(self):
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit

        self.wake = Wake()
        self.ip = TCP_SRV
        self.port = TCP_PORT
        self.sock = None
        self.client_list = dict()   # {ip: connection}
        self.client_threads = dict()

    # Task loop
    def run(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while not self.bind_port(self.ip, self.port):
            self.sleep(5)

        self.sock.listen(5)  # queue up to 5 requests

        while True:
            log.debug("TCP: Waiting for connection")
            connection, address = self.sock.accept()
            ip, port = str(address[0]), str(address[1])
            if connection not in self.client_list.values():
                try:
                    client_thread = ClientThread(connection, ip)
                    client_thread.start()
                    self.client_list[ip] = connection
                    log.debug("TCP: Connected with {}: {}".format(ip, port))
                except:
                    log.error("TCP: Thread did not start.")
            else:
                pass

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

    def get_settings_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_GET_SETTINGS_REQ, [])
        self.tcp_transmit.emit(ip, buf)

    def set_settings_req(self, ip, settings_dict):
        if 'ConnectedAnchors' in settings_dict:
            sett_str = settings_dict['ConnectedAnchors']
            sett_list = sett_str[1:-1].split(',')
            settings_dict['ConnectedAnchors'] = [int(sett) for sett in sett_list]

        settings_pb = Settings_pb2.Settings()
        ParseDict(settings_dict, settings_pb)
        settings_string = settings_pb.SerializeToString()
        buf = self.wake.prepare(Wake.CMD_SET_SETTINGS_REQ, settings_string)
        self.tcp_transmit.emit(ip, buf)

    def set_default_settings_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_SET_DEF_SETTINGS_REQ, [])
        self.tcp_transmit.emit(ip, buf)

    def reboot_req(self, ip):
        buf = self.wake.prepare(Wake.CMD_REBOOT_REQ, [])
        self.tcp_transmit.emit(ip, buf)

    @pyqtSlot(str, list)
    def tcp_transmit(self, ip, data):
        callback = self.TxCallback(self.client_list[ip].sendall, bytes(data))
        self.thread_pool.start(callback)

#
#
#
#   Client Thread


class ClientThread(QThread):
    def __init__(self, connection, ip, max_buffer_size=1536):
        QThread.__init__(self)
        atexit.register(self.terminate)  # function to be executed on exit
        self.connection = connection
        self.ip = ip
        self.buf_size = max_buffer_size
        self.posit = PositNetwork()

    def run(self):
        alive_socket = True
        self.connection.setblocking(False)
        while alive_socket:
            try:
                data = self.connection.recv(self.buf_size)
            except socket.error as e:
                if e.errno == 35:
                    pass
                elif e.errno == 10054:    # reset by peer
                    print("Connection close")
                    self.connection.close()
                    alive_socket = False
                elif e.errno == 11:  # reset by peer
                    print("Error 11, Resource unavailable!")
                    self.connection.close()
                    alive_socket = False
                else:
                    alive_socket = False
                    print(e)
            else:
                # print '<-'," ".join("{:02x}".format(ord(c)) for c in data)
                resp = []
                if len(data) > 0:
                    self.posit.process(data)
