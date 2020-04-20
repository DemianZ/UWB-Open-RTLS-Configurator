from PyQt5.QtCore import QThread, pyqtSignal
import atexit
import logging as log

import serial.tools.list_ports


class SerialTask(QThread):

    # Task signals
    sig_serial_list_ports = pyqtSignal(list, name='SerialTask_ListPorts')

    def __init__(self):
        QThread.__init__(self)

        atexit.register(self.terminate)  # function to be executed on exit

    # Task loop
    def run(self):
        self.get_ports()
        while True:
            self.sleep(5)

    def get_ports(self):
        available_ports = serial.tools.list_ports.comports()
        port_names = list()
        for port in available_ports:
            if port.description != "n/a":
                port_names.append(port.description)
        log.debug(port_names)
        self.sig_serial_list_ports.emit(port_names)
