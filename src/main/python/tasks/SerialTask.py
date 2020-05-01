from PyQt5.QtCore import QThread, pyqtSignal
import atexit
import logging as log
import serial
import serial.tools.list_ports
from modules.posit import Posit
from modules.wake import Wake

class SerialTask(QThread):

    # Task signals
    sig_serial_list_ports = pyqtSignal(dict, name='SerialTask_ListPorts')

    def __init__(self):
        QThread.__init__(self)
        self.wake = Wake()
        self.serial = None
        self.port_list = list()
        self.com_port = None
        self.connected = None
        atexit.register(self.terminate)  # function to be executed on exit

    # Task loop
    def run(self):
        self.get_ports()

        while True:
            if self.serial and self.serial.is_open:
                data = self.serial.read(100)
                if len(data):
                    cmd_res = self.wake.process(data)
                    if cmd_res is not None:
                        log.debug(cmd_res['cmd'] + cmd_res['data'])
            else:
                self.sleep(3)

    def get_ports(self):
        self.port_list = list()
        available_ports = serial.tools.list_ports.comports()
        for port in available_ports:
            if port.description != "n/a":
                self.port_list.append(port)

        if len(self.port_list):
            self.sig_serial_list_ports.emit({'port_dev': [port.device for port in self.port_list],
                                             'port_desc': [port.description for port in self.port_list]})

    def port_changed(self, port_index):
        self.com_port = self.port_list[port_index]

    def open_port(self):
        self.serial = serial.Serial()
        self.serial.baudrate = 19200
        self.serial.port = self.com_port.device
        self.serial.timeout = 1
        self.serial.open()
        if self.serial.is_open:
            return True
        else:
            return False

    def close_port(self):
        if self.serial.is_open:
            self.serial.close()
