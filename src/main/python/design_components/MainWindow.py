import platform

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from designs.mainwindow_ui import Ui_MainWindow
from tasks.SerialTask import SerialTask
from tasks.UDPServerTask import UdpServerTask
from tasks.UneTwrTask import UneTwrTask, UneNavMethod


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
        self.twr_task = UneTwrTask(UneNavMethod.lse)

        self.connect_ui_signals()
        self.connect_external_signals()

    def start_tasks(self):
        self.serial_task.start()
        self.udp_task.start()
        # self.twr_task.start()

    # @brief Connect MW ui signals
    def connect_ui_signals(self):
        self.pushButton_reloadPorts.clicked.connect(self.serial_task.get_ports)
        self.comboBox_port.currentIndexChanged.connect(lambda:
                                                       self.serial_task.port_changed(self.comboBox_port.currentIndex()))
        self.pushButton_connect.clicked.connect(self.serial_task.open_port)
        self.pushButton_rdConfig.clicked.connect(self.serial_task.get_settings)
        self.pushButton_wrConfig.clicked.connect(lambda:
                                                 self.serial_task.set_settings(self.get_settings_table_data()))
        self.pushButton_defConfig.clicked.connect(self.serial_task.set_default_settings)

    # @brief Connect signals from other tasks to local functions
    def connect_external_signals(self):
        # Connect serial task signals
        self.serial_task.sig_serial_list_ports.connect(lambda ports: self.add_ports(ports))
        self.serial_task.posit.sig_posit_settings_received.connect(lambda sets: self.serial_received_settings(sets))
        self.serial_task.sig_status_changed.connect(lambda st: self.serial_status_changed(st))
        self.serial_task.sig_add_console_logs.connect(lambda text, color: self.add_console_logs(text, color))

    # @brief Refresh ports at choose port combo-box
    def add_ports(self, ports):
        self.comboBox_port.clear()
        self.comboBox_port.addItems(ports['port_desc'])
        self.comboBox_port.setCurrentIndex(0)
        self.comboBox_port.setCurrentText(ports['port_desc'][0])

    def serial_received_settings(self, sets):
        self.tableWidget_config.setRowCount(0)
        for name, value in sets.items():
            row_position = self.tableWidget_config.rowCount()
            self.tableWidget_config.insertRow(row_position)
            item_name = QTableWidgetItem(str(name))
            item_name.setFlags(Qt.ItemIsEnabled)    # disable editing
            item_value = QTableWidgetItem(str(value))
            self.tableWidget_config.setItem(row_position, 0, item_name)
            self.tableWidget_config.setItem(row_position, 1, item_value)

        pass

    def serial_status_changed(self, st):
        self.label_serialStatus.setText(st)

    def add_console_logs(self, text, color):
        self.textBrowser_console.setTextColor(color)
        self.textBrowser_console.append(str(text))
        self.textBrowser_console.repaint()

    def get_settings_table_data(self):
        table_view = self.tableWidget_config
        data = dict()
        for row in range(table_view.rowCount()):
            key_data = table_view.model().index(row, 0).data()
            value_data = table_view.model().index(row, 1).data()
            data.update({str(key_data): value_data})
        return data

