from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor
import atexit
import logging as log
import serial
import json
import serial.tools.list_ports

from modules.PositSerial import PositSerial
from proto import Settings_pb2
from google.protobuf.json_format import Parse, ParseDict


class SerialTask(QThread):

    # Task signals
    sig_serial_list_ports = pyqtSignal(dict, name='SerialTask_ListPorts')
    sig_status_changed = pyqtSignal(str, name='SerialTask_StatusChanged')
    sig_add_console_logs = pyqtSignal(str, QColor, name='SerialTask_AddLogs')

    def __init__(self):
        QThread.__init__(self)
        self.serial = None
        self.port_list = list()
        self.com_port = None
        self.connected = None
        self.posit = PositSerial(self.serial)
        atexit.register(self.terminate)  # function to be executed on exit

    # Task loop
    def run(self):
        self.get_ports()

        while True:
            if self.serial and self.connected:
                try:
                    data = self.serial.read()
                except serial.SerialException as e:
                    self.usleep(100)
                    continue
                if len(data):
                    self.posit.process(data)
            self.usleep(1)

    def stop(self):
        self.quit()

    def get_ports(self):
        self.port_list = list()
        available_ports = serial.tools.list_ports.comports()
        for port in available_ports:
            if port.description != "n/a":
                self.port_list.append(port)

        num_devices = len(self.port_list)
        if num_devices:
            self.sig_status_changed.emit('Found ' + str(num_devices) + ' device(s).')
            self.sig_serial_list_ports.emit({'port_dev': [port.device for port in self.port_list],
                                             'port_desc': [port.description for port in self.port_list]})
        else:
            self.sig_status_changed.emit('Device not found.')

    def port_changed(self, port_index):
        self.com_port = self.port_list[port_index]

    def open_port(self):
        if self.com_port.device is None:
            return False

        if self.serial and self.serial.isOpen():
            self.close_port()
        try:
            self.serial = serial.Serial(
                port=self.com_port.device,
                baudrate=19200,
                parity=serial.PARITY_ODD,
                stopbits=serial.STOPBITS_TWO,
            )
            if self.serial.isOpen():
                self.connected = True
                self.sig_status_changed.emit('Connected to ' + self.com_port.description)
                self.sig_add_console_logs.emit('Connected to ' + self.com_port.device, QColor("blue"))
                return True
        except serial.serialutil.SerialException as e:
            log.debug(str(e))
        return False

    @pyqtSlot()
    def close_port(self):
        if self.serial.is_open:
            self.connected = False
            self.sig_status_changed.emit('Disconnected from ' + self.com_port.description)
            self.serial.close()

    @pyqtSlot()
    def serial_write(self, data):
        if self.serial and self.serial.is_open:
            self.serial.write(data)
