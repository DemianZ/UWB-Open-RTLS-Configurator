## POSIT.PRO Configurator.

------

#### Python3:

- Switch to virtual environment: `source fbsenv/bin/activate`
- Switch back: `decativate`
- Run project: `fbs run`

#### Qt:

Make python file from .ui Qt form:

- Switch to fbsenv
- `pyuic5 ./src/main/resources/ui_qt/mainwindow.ui -o ./src/main/python/designs/mainwindow_ui.py`

#### PyCharm:

- *Settings->Build, Execution, Deployment->Python Debugger:*
- *Gevent compatible - [True], PyQt compatible- [True, PyQt5]*

