from fbs_runtime.application_context.PyQt5 import ApplicationContext
import sys

from design_components.MainWindow import MainWindow

import logging as log

log.basicConfig(level=log.DEBUG,
                format='%(levelname)s:\t%(message)s\t(%(filename)s|%(funcName)s|%(asctime)s.%(msecs)03d)',
                datefmt='%H:%M:%S')

if __name__ == '__main__':
    app_context = ApplicationContext()       # 1. Instantiate ApplicationContext
    window = MainWindow()
    window.start_tasks()
    window.show()
    exit_code = app_context.app.exec_()      # 2. Invoke appctxt.app.exec_()
    sys.exit(exit_code)
