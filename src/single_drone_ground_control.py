'''
MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

#!/usr/bin/env python

import sys
from PyQt5 import QtWidgets

import rclpy
from GUI.single_drone_flight import Ui_MultiDroneGroundControlStation
from ROS_Node.ros_single_drone_control import SingleDroneRosThread


if __name__ == "__main__":
    # Initialize ROS2
    rclpy.init(args=None)

    # Create PyQt5 application
    app = QtWidgets.QApplication(sys.argv)
    SingleDroneGroundControlStation = QtWidgets.QTabWidget()
    ui = Ui_MultiDroneGroundControlStation()
    ui.setupUi(SingleDroneGroundControlStation)

    # Create ROS thread
    rosSingleDroneThread = SingleDroneRosThread(ui)
    rosSingleDroneThread.start()

    # Show the window
    SingleDroneGroundControlStation.show()
    print("System Started")

    try:
        sys.exit(app.exec_())
    finally:
        # Cleanup ROS2
        rclpy.shutdown()
