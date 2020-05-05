import logging as log
import os

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from designs.mainwindow_ui import Ui_MainWindow
from modules.mpl import MplCanvas, MplWidget
from tasks.SerialTask import SerialTask
from tasks.UDPServerTask import UdpServerTask
from tasks.UneTask import UneTask, UneNavMethod
from tasks.PositTask import PositTask

# from matplotlib.backends.backend_qt5agg import (                      # uncomment for mpl toolbar
#         FigureCanvas, NavigationToolbar2QT as NavigationToolbar)      # uncomment for mpl toolbar


#
# @brief:
#   Main Window module: prepare UI, create/start tasks,
#   connect ui/tasks signals/slots. Contains UI slots.
# @classSignals:
#   UI_AddAnchor:   args: list [ip, pos_x, pos_y, pos_z]
#       - signal fired on "Add Anchor" pushButton click
#   UI_AddTag:   args: str (ip)
#       - signal fired on "Add Tag" pushButton click
#   UI_ConnectNodes: args: list[[anchor_node_id,...],[tag_node_id,...]]
#       - signal fired on connect toolButton click
#
class MainWindow(QMainWindow, Ui_MainWindow):

    sig_ui_add_anchor_req = pyqtSignal(list, name='UI_AddAnchor')
    sig_ui_add_tag_req = pyqtSignal(str, name='UI_AddTag')
    sig_ui_connect_nodes_req = pyqtSignal(list, name='UI_ConnectNodes')

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.mpl_layout = None
        self._widget_mpl = None
        self.mpl_canvas = None
        self.treeview_status_model = None
        self.init_ui()

        self.serial_task = SerialTask()
        self.udp_task = UdpServerTask(self)
        self.une_task = UneTask(UneNavMethod.lse)
        self.posit_task = PositTask()

        self.connect_ui_signals_ext_slots()
        self.connect_ext_signals_ui_slots()
        self.connect_ext_signals_ext_slots()

    def init_ui(self):
        # Init matplotlib
        self.mpl_layout = QVBoxLayout(self.widget_mpl)
        self._widget_mpl = MplWidget(self.widget_mpl)
        self.mpl_canvas = MplCanvas()
        self.mpl_layout.addWidget(self.mpl_canvas)
        # self.mpl_toolbar = NavigationToolbar(self.mpl_canvas, self)   # uncomment for mpl toolbar
        # self.mpl_layout.addWidget(self.mpl_toolbar)                   # uncomment for mpl toolbar
        pass

    # @brief: Connect MW ui signals.
    def connect_ui_signals_ext_slots(self):
        # Self UI Slots
        self.pushButton_addAnchor.clicked.connect(self.add_anchor_req)
        self.pushButton_addTag.clicked.connect(self.add_tag_req)
        self.toolButton_connectUne.clicked.connect(self.connect_nodes_req)
        # Serial Task
        self.pushButton_reloadPorts.clicked.connect(self.serial_task.get_ports)
        self.comboBox_port.currentIndexChanged.connect(
            lambda: self.serial_task.port_changed(self.comboBox_port.currentIndex()))
        self.pushButton_connect.clicked.connect(self.serial_task.open_port)
        self.pushButton_disconnect.clicked.connect(self.serial_task.close_port)
        self.pushButton_rdConfig.clicked.connect(self.serial_task.get_settings)
        self.pushButton_wrConfig.clicked.connect(
            lambda: self.serial_task.set_settings(self.serial_get_settings_table_data()))
        self.pushButton_defConfig.clicked.connect(self.serial_task.set_default_settings)
        # Posit Task
        self.sig_ui_add_anchor_req.connect(lambda data: self.posit_task.add_anchor_req(data))
        self.sig_ui_add_tag_req.connect(lambda node_id: self.posit_task.add_tag_req(node_id))
        self.sig_ui_connect_nodes_req.connect(lambda data: self.posit_task.connect_nodes_req(data))

    # @brief: Connect signals from other tasks to local functions.
    def connect_ext_signals_ui_slots(self):
        # Serial Task
        self.serial_task.sig_serial_list_ports.connect(lambda ports: self.serial_add_ports(ports))
        self.serial_task.posit.sig_posit_settings_received.connect(lambda sets: self.serial_received_settings(sets))
        self.serial_task.sig_status_changed.connect(lambda st: self.serial_status_changed(st))
        self.serial_task.sig_add_console_logs.connect(lambda text, color: self.serial_add_console_logs(text, color))
        # Posit Task
        # Network device list table widget signals/slots.
        self.posit_task.sig_update_settings.connect(lambda data: self.network_received_settings(data))
        self.posit_task.sig_add_device.connect(lambda data: self.network_add_device(data))
        self.posit_task.sig_remove_device.connect(lambda ip: self.network_remove_device(ip))
        self.posit_task.sig_update_device.connect(lambda data: self.network_update_device(data))
        self.posit_task.sig_add_anchor_resp.connect(lambda node_id: self.add_anchor_resp(node_id))
        self.posit_task.sig_add_tag_resp.connect(lambda node_id: self.add_tag_resp(node_id))
        self.posit_task.sig_connect_une_resp.connect(lambda tags: self.connect_nodes_resp(tags))

    def connect_ext_signals_ext_slots(self):
        self.udp_task.posit.sig_cmd_hello.connect(lambda ip: self.posit_task.slot_cmd_hello(ip))
        self.udp_task.posit.sig_cmd_get_settings.connect(lambda data: self.posit_task.slot_cmd_get_settings(data))
        self.udp_task.posit.sig_twr_ranging.connect(lambda data: self.posit_task.slot_twr_ranging(data))

    # @brief: Start all application tasks.
    def start_tasks(self):
        self.serial_task.start()
        self.udp_task.start()
        self.posit_task.start()
        # self.une_task.start()

    def stop_tasks(self):
        self.serial_task.stop()
        self.udp_task.stop()
        self.posit_task.stop()
        # self.une_task.stop()

    def closeEvent(self, event):
        # reply = QMessageBox.question(self, 'Window Close', 'Are you sure you want to close the window?',
        #                              QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        # if reply == QMessageBox.Yes:
        #     event.accept()
        # else:
        #     event.ignore()
        self.stop_tasks()
        event.accept()
        os._exit(0)

    # @brief: Refresh ports in "choose port" combo-box.
    def serial_add_ports(self, ports):
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

    @pyqtSlot(str, str)
    def serial_status_changed(self, st):
        self.label_serialStatus.setText(st)

    @pyqtSlot(str, QColor)
    def serial_add_console_logs(self, text, color):
        self.textBrowser_console.setTextColor(color)
        self.textBrowser_console.append(str(text))
        self.textBrowser_console.repaint()

    # @brief Method to get settings dict from serial device configuration table.
    def serial_get_settings_table_data(self):
        table_view = self.tableWidget_config
        data = dict()
        for row in range(table_view.rowCount()):
            key_data = table_view.model().index(row, 0).data()
            value_data = table_view.model().index(row, 1).data()
            data.update({str(key_data): value_data})
        return data

    @pyqtSlot(dict)
    def network_received_settings(self, sets):
        self.tableWidget_configNet.setRowCount(0)
        for name, value in sets.items():
            row_position = self.tableWidget_configNet.rowCount()
            self.tableWidget_configNet.insertRow(row_position)
            item_name = QTableWidgetItem(str(name))
            item_name.setFlags(Qt.ItemIsEnabled)    # disable editing
            item_value = QTableWidgetItem(str(value))
            self.tableWidget_configNet.setItem(row_position, 0, item_name)
            self.tableWidget_configNet.setItem(row_position, 1, item_value)

    # @brief:   Add device to "Network List" table.
    # @args:    list(ip, {settings})
    @pyqtSlot(list)
    def network_add_device(self, data):
        row_position = self.tableWidget_networkList.rowCount()
        self.tableWidget_networkList.insertRow(row_position)
        if data[0] is not None:
            ip = data[0]
            item_node_ip = QTableWidgetItem(str(ip))
            item_node_ip.setFlags(Qt.ItemIsEnabled)  # disable editing
            self.tableWidget_networkList.setItem(row_position, 1, item_node_ip)

        if len(data[1]) == 0:
            return

        settings = data[1]
        for sett, val in settings.items():
            item = None
            pos = 0
            if sett == 'NodeID':
                item = QTableWidgetItem(str(settings['NodeID']))
                pos = 0
            elif sett == 'NodeType':
                pos = 2
            elif sett == 'RTLSMode':
                pos = 3
            if item is None:
                continue
            item = QTableWidgetItem(str(settings['sett']))
            item.setFlags(Qt.ItemIsEnabled)  # disable editing
            self.tableWidget_configNet.setItem(row_position, pos, item)
        return

    @pyqtSlot(str)
    def network_remove_device(self, node_id):
        return

    @pyqtSlot(list)
    def network_update_device(self, device):
        return

    # @brief: Slot called on "Add Anchor" button click.
    #         Sends signal to PositTask to check and add tag to UNE.
    @pyqtSlot()
    def add_anchor_req(self):
        try:
            node_id = self.textEdit_anchorId.toPlainText()
            pos_x = self.textEdit_anchorX.toPlainText()
            pos_y = self.textEdit_anchorY.toPlainText()
            pos_z = self.textEdit_anchorZ.toPlainText()
            if int(node_id) > 0 and float(pos_x) > 0 and float(pos_y) > 0 and float(pos_z) > 0:
                self.sig_ui_add_anchor_req.emit([node_id, [pos_x, pos_y, pos_z]])
            else:
                log.debug('invalid parameters')
        except TypeError:
            log.debug('invalid parameters')
        except ValueError:
            log.debug('invalid parameters')

    # @brief: Slot called on response from PositTask to add_tag_req.
    #         Adds tag ID string to tag list tree-view.
    @pyqtSlot(str)
    def add_anchor_resp(self, node_id):
        self.listWidget_anchor.addItem(node_id)

    # @brief: Slot called on "Add Tag" button click.
    #         Sends signal to PositTask to check and add tag to UNE.
    @pyqtSlot()
    def add_tag_req(self):
        try:
            node_id = self.textEdit_tagId.toPlainText()
            if int(node_id) > 0:
                self.sig_ui_add_tag_req.emit(node_id)
            else:
                log.debug('invalid parameters')
        except TypeError as e:
            log.debug('invalid parameters')

    # @brief:
    #   Slot called on response from PositTask to add_tag_req.
    #   Adds tag ID string to tag list tree-view.
    @pyqtSlot(str)
    def add_tag_resp(self, node_id):
        self.listWidget_tag.addItem(node_id)

    # @brief:
    #   Slot called on "Connect" tool-button click.
    #   Takes all selected NodeIDs from tag and anchor tree-views and sends them to PositTask.
    #         Sends signal to PositTask to make new UNE connections.
    def connect_nodes_req(self):
        selected_tags = [item.text() for item in self.listWidget_tag.selectedItems()]
        selected_anchors = [item.text() for item in self.listWidget_anchor.selectedItems()]
        self.sig_ui_connect_nodes_req.emit([selected_anchors, selected_tags])
        return

    # @brief: Slot called on response from PositTask to connect_nodes_req.
    #         Adds new connection items to UNE status tree-view.
    def connect_nodes_resp(self, tags):
        self.treeWidget_uneStatus.clear()
        for t in tags:
            tag_item = QTreeWidgetItem(self.treeWidget_uneStatus)
            tag_item.setText(0, t[0])
            for an in t[1]:
                an_item = QTreeWidgetItem(tag_item)
                an_item.setText(0, an[0])
                an_item.setText(2, (str('X: ' + str(an[1][0])) +
                                    str(' Y: ' + an[1][1]) +
                                    str(' Z: ' + an[1][2])))
        self.treeWidget_uneStatus.sortByColumn(0, Qt.AscendingOrder)
        self.treeWidget_uneStatus.show()
        return
