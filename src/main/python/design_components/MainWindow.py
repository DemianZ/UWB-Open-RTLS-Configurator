import platform

from PyQt5.QtWidgets import *

from designs.mainwindow_ui import Ui_MainWindow
from tasks.SerialTask import SerialTask
from tasks.UDPServerTask import UdpServerTask


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        """
        Call attribute class constructors,
        connect stop/start buttons with signals
        """
        super().__init__()
        self.setupUi(self)
        if platform.system() == 'Linux':
            pass
            # self.showFullScreen()
        else:
            pass
            # self.showMaximized()

        self.serial_task = SerialTask()
        self.udp_task = UdpServerTask(self)
        self.connect_ui_signals()
        self.connect_external_signals()

    def start_tasks(self):
        self.serial_task.start()
        self.udp_task.start()

    # @brief Connect MW ui signals
    def connect_ui_signals(self):
        self.pushButton_reloadPorts.clicked.connect(self.serial_task.get_ports)
        self.comboBox_port.currentIndexChanged.connect(lambda:
                                                       self.serial_task.port_changed(self.comboBox_port.currentIndex()))
        self.pushButton_connect.clicked.connect(self.serial_task.open_port)

    # @brief Connect signals from other tasks to local functions
    def connect_external_signals(self):
        # Connect serial task signals
        self.serial_task.sig_serial_list_ports.connect(lambda ports: self.add_ports(ports))

    # @brief Refresh ports at choose port combo-box
    def add_ports(self, ports):
        self.comboBox_port.clear()
        self.comboBox_port.addItems(ports['port_desc'])
        self.comboBox_port.setCurrentIndex(0)
        self.comboBox_port.setCurrentText(ports['port_desc'][0])
