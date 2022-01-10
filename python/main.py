# from fbs_runtime.application_context.PyQt5 import ApplicationContext
from PyQt5.QtWidgets import QApplication, QWidget
import sys
import logging as log

from design_components.MainWindow import MainWindow

log.basicConfig(level=log.DEBUG,
                format='%(levelname)s:\t%(message)s\t(%(filename)s|%(funcName)s|%(asctime)s.%(msecs)03d)',
                datefmt='%H:%M:%S')
log.getLogger('matplotlib.font_manager').disabled = True

if __name__ == '__main__':

    app = QApplication(sys.argv)

    window = MainWindow()
    window.start_tasks()
    window.show()

    sys.exit(app.exec_())
