import logging as log
import os

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from designs.mainwindow_ui import Ui_MainWindow
from tasks.SerialTask import SerialTask
from tasks.UDPServerTask import UdpServerTask
from tasks.UneTask import UneTask, UneNavMethod, UneTaskTst
from tasks.PositTask import PositTask
from tasks.PositCalibrationTask import PositCalibrationTask
from modules.PositSerial import PositSerial
from modules.GraphWidget import GraphWidget


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
    sig_ui_start_calibration = pyqtSignal(list, float, name='UI_StartCalibration')

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.set_status('Loaded')
        self.treeview_status_model = None
        self.graph = GraphWidget()
        self.verticalLayout_mpl.addWidget(self.graph)

        self.serial_task = SerialTask()
        self.udp_task = UdpServerTask(self)
        self.une_task = UneTask(UneNavMethod.lse)
        self.posit_task = PositTask()
        self.calib_task = PositCalibrationTask()
        # !!! For Test
        self.une_tst_task = UneTaskTst()

        self.connect_ui_signals_ext_slots()
        self.connect_ext_signals_ui_slots()
        self.connect_ext_signals_ext_slots()

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
        # self.pushButton_connect.clicked.connect(self.serial_task.posit.get_settings)
        self.pushButton_disconnect.clicked.connect(self.serial_task.close_port)
        self.pushButton_serialReadConfig.clicked.connect(self.serial_task.posit.get_settings)
        self.pushButton_serialWriteConfig.clicked.connect(
            lambda: self.serial_task.posit.set_settings(self.get_settings_table_data(self.tableWidget_serialConfig)))
        self.pushButton_serialDefConfig.clicked.connect(self.serial_task.posit.set_default_settings)
        self.pushButton_serialReboot.clicked.connect(self.serial_task.posit.reboot)
        # Posit Task
        self.sig_ui_add_anchor_req.connect(lambda data: self.posit_task.add_anchor_req(data))
        self.sig_ui_add_tag_req.connect(lambda node_id: self.posit_task.add_tag_req(node_id))
        self.sig_ui_connect_nodes_req.connect(lambda data: self.posit_task.connect_nodes_req(data))
        self.toolButton_startUne.clicked.connect(self.posit_task.start_une)
        # Network Task
        self.pushButton_netReadConfig.clicked.connect(self.net_get_settings_req)
        self.pushButton_netWriteConfig.clicked.connect(self.net_set_settings_req)
        self.pushButton_netDefConfig.clicked.connect(self.net_set_default_settings_req)
        self.pushButton_netReboot.clicked.connect(self.net_reboot_req)
        # MPL
        self.toolButton_setRoom.clicked.connect(
            lambda: self.graph.set_room(self.textEdit_roomX.toPlainText(), self.textEdit_roomY.toPlainText()))
        # Calibration
        self.toolButton_startCalibration.clicked.connect(self.start_calibration)
        self.sig_ui_start_calibration.connect(lambda nodes, dist: self.calib_task.start_calibration(nodes, dist))

    # @brief: Connect signals from other tasks to local functions.
    def connect_ext_signals_ui_slots(self):
        # SerialTask + SerialTask.PositSerial
        self.serial_task.sig_serial_list_ports.connect(lambda ports: self.serial_add_ports(ports))
        self.serial_task.sig_status_changed.connect(lambda st: self.serial_status_changed(st))
        self.serial_task.sig_add_console_logs.connect(lambda text, color: self.serial_add_console_logs(text, color))
        self.serial_task.posit.sig_posit_settings_received.connect(lambda sets: self.serial_update_settings(sets))
        # UdpTask.PositNetwork
        self.udp_task.posit.sig_ui_update_settings.connect(lambda data: self.network_update_settings(data))
        self.udp_task.posit.sig_ui_update_settings.connect(lambda data: self.network_update_settings(data))
        self.udp_task.posit.sig_ui_add_device.connect(lambda data: self.network_add_device(data))
        self.udp_task.posit.sig_ui_remove_device.connect(lambda ip: self.network_remove_device(ip))
        self.udp_task.posit.sig_ui_update_device.connect(lambda data: self.network_update_device(data))
        # PositTask
        self.posit_task.sig_ui_update_anchor_resp.connect(lambda data: self.add_anchor_resp(data))
        self.posit_task.sig_ui_update_tag_resp.connect(lambda data: self.add_tag_resp(data))
        self.posit_task.sig_ui_connect_une_resp.connect(lambda tags: self.connect_nodes_resp(tags))
        # Calibration task
        self.calib_task.sig_update_status.connect(lambda status: self.set_status(status))

    def connect_ext_signals_ext_slots(self):
        self.serial_task.posit.sig_serial_write.connect(
            lambda data: self.serial_task.serial_write(data))
        self.udp_task.posit.sig_udp_transmit.connect(
            lambda ip, data: self.udp_task.udp_transmit(ip, data))
        self.posit_task.sig_une_add_new_tag.connect(
            lambda tag: self.une_task.api_slot_add_tag(tag))
        self.posit_task.sig_une_upd_tag_meas.connect(
            lambda meas_list: self.une_task.api_slot_tag_upd_meas(meas_list))
        self.une_task.api_sig_new_pvt.connect(
            lambda pvt: self.posit_task.une_new_pvt(pvt))
        self.udp_task.posit.sig_posit_twr_received.connect(
            lambda twr_info: self.posit_task.twr_received(twr_info))
        self.udp_task.posit.sig_posit_twr_received.connect(
            lambda twr_info: self.posit_task.twr_received(twr_info))
        self.udp_task.posit.sig_posit_twr_received.connect(
            lambda twr_info: self.calib_task.twr_received(twr_info))
        self.serial_task.posit.sig_posit_twr_received.connect(
            lambda twr_info: self.calib_task.twr_received(twr_info))
        # Graph Widget
        self.posit_task.sig_ui_update_anchor_resp.connect(
            lambda data: self.graph.update_anchor(data))
        self.posit_task.sig_ui_update_tag_resp.connect(
            lambda data: self.graph.update_tag(data))
        self.posit_task.sig_ui_new_pvt.connect(
            lambda data: self.graph.update_tag(data))

        # !!! For test. Signal to UNE
        self.une_tst_task.sig_add_new_tag.connect(self.une_task.api_slot_add_tag)
        self.une_tst_task.sig_upd_tag_meas.connect(self.une_task.api_slot_tag_upd_meas)
        self.une_tst_task.sig_calib_start.connect(self.une_task.api_slot_calibrate)
        # Signal from UNE
        self.une_task.api_sig_new_pvt.connect(self.une_tst_task.slot_new_pvt)
        self.une_task.api_sig_calib_finished.connect(self.une_tst_task.slot_calib_finished)

    # @brief: Start all application tasks.
    def start_tasks(self):
        self.serial_task.start()
        self.udp_task.start()
        self.posit_task.start()
        self.une_task.start()
        self.calib_task.start()
        # !!! For Test
        self.une_tst_task.start()

    def stop_tasks(self):
        self.serial_task.stop()
        self.udp_task.stop()
        self.posit_task.stop()
        self.calib_task.stop()

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
    @pyqtSlot(dict)
    def serial_add_ports(self, ports):
        self.comboBox_port.clear()
        self.comboBox_port.addItems(ports['port_desc'])
        self.comboBox_port.setCurrentIndex(0)
        self.comboBox_port.setCurrentText(ports['port_desc'][0])

    @pyqtSlot(dict)
    def serial_update_settings(self, sets):
        self.tableWidget_serialConfig.setRowCount(0)
        for name, value in sets.items():
            row_position = self.tableWidget_serialConfig.rowCount()
            self.tableWidget_serialConfig.insertRow(row_position)
            item_name = QTableWidgetItem(str(name))
            item_name.setFlags(Qt.ItemIsEnabled)    # disable editing
            item_value = QTableWidgetItem(str(value))
            self.tableWidget_serialConfig.setItem(row_position, 0, item_name)
            self.tableWidget_serialConfig.setItem(row_position, 1, item_value)

    @pyqtSlot(str, str)
    def serial_status_changed(self, st):
        self.label_serialStatus.setText(st)

    @pyqtSlot(str, QColor)
    def serial_add_console_logs(self, text, color):
        self.textBrowser_console.setTextColor(color)
        self.textBrowser_console.append(str(text))
        self.textBrowser_console.repaint()

    @pyqtSlot(dict)
    def network_update_settings(self, sets):
        self.tableWidget_netConfig.setRowCount(0)
        for name, value in sets.items():
            row_position = self.tableWidget_netConfig.rowCount()
            self.tableWidget_netConfig.insertRow(row_position)
            item_name = QTableWidgetItem(str(name))
            item_name.setFlags(Qt.ItemIsEnabled)    # disable editing
            item_value = QTableWidgetItem(str(value))
            self.tableWidget_netConfig.setItem(row_position, 0, item_name)
            self.tableWidget_netConfig.setItem(row_position, 1, item_value)
        self.tableWidget_netConfig.setSelectionBehavior(QTableView.SelectRows)

    # @brief:   Add device to "Network List" table.
    # @args:    list([NodeID, IP, Type, Mode, Rx/Tx, Error])
    @pyqtSlot(list)
    def network_add_device(self, data):
        row_position = self.tableWidget_networkList.rowCount()
        self.tableWidget_networkList.insertRow(row_position)
        for i, par in enumerate(data):
            item = QTableWidgetItem(str(par))
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)  # disable editing
            self.tableWidget_networkList.setItem(row_position, i, item)
        return

    # @brief:   Remove device from "Network List" table.
    # @args:    list([NodeID, IP, Type, Mode, Rx/Tx, Error])
    @pyqtSlot(list)
    def network_remove_device(self, node_data):
        return

    # @brief:   Update device from "Network List" table.
    # @args:    list([NodeID, IP, Type, Mode, Rx/Tx, Error])
    @pyqtSlot(list)
    def network_update_device(self, device):
        for i in range(self.tableWidget_networkList.rowCount()):
            ip = self.tableWidget_networkList.item(i, 1).text()
            if ip == device[1]:
                row_position = i
                break
        for i, par in enumerate(device):
            item = QTableWidgetItem(str(device[i]))
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)  # disable editing
            self.tableWidget_networkList.setItem(row_position, i, item)
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
    @pyqtSlot(list)
    def add_anchor_resp(self, data):
        node_id = data[0]
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
    def add_tag_resp(self, data):
        node_id = data[0]
        self.listWidget_tag.addItem(node_id)

    # @brief:
    #   Slot called on "Connect" tool-button click.
    #   Takes all selected NodeIDs from tag and anchor tree-views and sends them to PositTask.
    #         Sends signal to PositTask to make new UNE connections.
    @pyqtSlot()
    def connect_nodes_req(self):
        selected_tags = [item.text() for item in self.listWidget_tag.selectedItems()]
        selected_anchors = [item.text() for item in self.listWidget_anchor.selectedItems()]
        self.sig_ui_connect_nodes_req.emit([selected_anchors, selected_tags])
        return

    # @brief: Slot called on response from PositTask to connect_nodes_req.
    #         Adds new connection items to UNE status tree-view.
    @pyqtSlot(list)
    def connect_nodes_resp(self, tags):
        self.treeWidget_uneStatus.clear()
        for t in tags:
            tag_item = QTreeWidgetItem(self.treeWidget_uneStatus)
            tag_item.setText(0, t[0].m_name)
            for an in t[1]:
                an_item = QTreeWidgetItem(tag_item)
                an_item.setText(0, an[0])
                an_item.setText(2, (str('X: ' + str(an[1][0])) +
                                    str(' Y: ' + an[1][1]) +
                                    str(' Z: ' + an[1][2])))
        self.treeWidget_uneStatus.sortByColumn(0, Qt.AscendingOrder)
        self.treeWidget_uneStatus.show()
        return

    # @brief Method to get settings dict from serial device configuration table.
    @staticmethod
    def get_settings_table_data(table_view):
        data = dict()
        for row in range(table_view.rowCount()):
            key_data = table_view.model().index(row, 0).data()
            value_data = table_view.model().index(row, 1).data()
            data.update({str(key_data): value_data})
        for key in PositSerial.ip32_keys:
            if key in data:
                data[key] = PositSerial.str_to_ip32(str(data[key]))
        return data

    @pyqtSlot()
    def net_get_settings_req(self):
        selected_row_i = self.tableWidget_networkList.selectionModel().selectedRows()[0].row()
        ip = self.tableWidget_networkList.item(selected_row_i, 1).text()

        self.udp_task.posit.get_settings_req(ip)

    @pyqtSlot()
    def net_set_settings_req(self):
        selected_row_i = self.tableWidget_networkList.selectionModel().selectedRows()[0].row()
        ip = self.tableWidget_networkList.item(selected_row_i, 1).text()

        sett_dict = self.get_settings_table_data(self.tableWidget_netConfig)
        self.udp_task.posit.set_settings_req(ip, sett_dict)

    @pyqtSlot()
    def net_set_default_settings_req(self):
        selected_row_i = self.tableWidget_networkList.selectionModel().selectedRows()[0].row()
        ip = self.tableWidget_networkList.item(selected_row_i, 1).text()
        self.udp_task.posit.set_default_settings_req(ip)

    @pyqtSlot()
    def net_reboot_req(self):
        selected_row_i = self.tableWidget_networkList.selectionModel().selectedRows()[0].row()
        ip = self.tableWidget_networkList.item(selected_row_i, 1).text()
        self.udp_task.posit.reboot_req(ip)

    @pyqtSlot(str)
    def set_status(self, status):
        self.statusbar.showMessage(status)

    @pyqtSlot()
    def start_calibration(self):
        selected_tags = [item.text() for item in self.listWidget_tag.selectedItems()]
        selected_anchors = [item.text() for item in self.listWidget_anchor.selectedItems()]
        nodes = selected_anchors + selected_tags
        self.sig_ui_start_calibration.emit(nodes, 1.5)
