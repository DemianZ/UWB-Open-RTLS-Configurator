import platform

from PyQt5.QtWidgets import *

from designs.mainwindow_ui import Ui_MainWindow
from modules.SerialTask import SerialTask


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
        self.connect_ui_slots()
        self.connect_signals()

    def start_tasks(self):
        self.serial_task.start()

    # @brief Connect MW ui slots
    def connect_ui_slots(self):
        self.pushButton_reloadPorts.clicked.connect(self.serial_task.get_ports)

    # @brief Connect signals from other tasks to local functions
    def connect_signals(self):
        # Connect serial task signals
        self.serial_task.sig_serial_list_ports.connect(lambda ports: self.add_ports(ports))

    # @brief Refresh ports at choose port combo-box
    def add_ports(self, ports):
        self.comboBox_port.clear()
        self.comboBox_port.addItems(ports)
