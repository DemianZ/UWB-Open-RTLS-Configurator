## UWB Open-RTLS Configurator

This is a cross-platform PyQT tool for configuration and monitoring nodes of UWB Open-RTLS system.

Firmware for STM32-based RTLS nodes is [here](https://github.com/DemianZ/UWB-Open-RTLS-MCU).

Configuration tool is used setting up and monitor RTLS nodes, connected to your local network. You can set network and RTLS parameters for each node. Tool has a simple monitor window (based on pyqtgraph) with constant anchor positions and live-updating tag positions. You can configure Universal Navigation Engine (UNE) by adding new tags and anchors.

<img src="docs/config_utility_screen.png" alt="config_utility_screen"/>

##### Simple example, 4 anchor, 1 tag, TWR mode, update rate - 10 Hz:

![train_gif](docs/train_gif.gif)

##### Generate .py proto files from proto file using Protoc (Mac OS/Linux)

```bash
brew install protobuf
protoc -I=proto --python_out=proto Settings.proto
protoc -I=proto --python_out=proto Monitoring.proto
```

##### Generating python file from Qt form (.ui):

```bash
pyuic5 ./src/main/resources/ui_qt/mainwindow.ui -o ./src/main/python/designs/mainwindow_ui.py
```

For all questions, please mail me demianzenkov@gmail.com

