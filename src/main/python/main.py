from fbs_runtime.application_context.PyQt5 import ApplicationContext
import sys
import os
import logging as log

from design_components.MainWindow import MainWindow

log.basicConfig(level=log.DEBUG,
                format='%(levelname)s:\t%(message)s\t(%(filename)s|%(funcName)s|%(asctime)s.%(msecs)03d)',
                datefmt='%H:%M:%S')
log.getLogger('matplotlib.font_manager').disabled = True

if __name__ == '__main__':
    app_context = ApplicationContext()       # 1. Instantiate ApplicationContext
    window = MainWindow()
    window.start_tasks()
    window.show()
    exit_code = app_context.app.exec_()      # 2. Invoke appctxt.app.exec_()
    window.stop_tasks()
    os._exit(0)
    sys.exit(exit_code)


