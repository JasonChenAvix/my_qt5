import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QCheckBox
from PyQt5.QtCore import Qt, QTimer
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from avix_msg.msg import GimbalInfo, TrackingUpdate

import matplotlib.pyplot as plt
import numpy as np

class RosSubscriberApp(QWidget, Node):
    def __init__(self):
        super().__init__(node_name="ros_subscriber")
        QWidget.__init__(self)

        self.create_subscription(GimbalInfo, '/gimbal/info', self.listener_callback, 10)
        self.create_subscription(TrackingUpdate, '/object_detection/target_deviation', self.deviation_callback, 10)

        self.tracking_cmd_publisher = self.create_publisher(Bool, '/icp_interface/tracking_cmd', 10)
        self.following_cmd_publisher = self.create_publisher(Int32, '/icp_interface/following_cmd', 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(100)
        
        self.init_ui()

        plt.ion()
        plt.figure(1)
        self.t, self.mx, self.my = [], [], []
        self.count = 0
        self.plot_initialized = False

    def timer_callback(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def deviation_callback(self, msg):
        if self.plot_initialized:
            self.t.append(self.count * 0.1)
            self.mx.append(msg.deviation_x)
            self.my.append(msg.deviation_y)
            plt.clf()
            plt.plot(self.t, self.mx, '-r', label="X Deviation")
            plt.plot(self.t, self.my, '-b', label="Y Deviation")
            plt.legend()
            plt.pause(0.1)
            self.count += 1

    def listener_callback(self, msg):
        # Implement what happens when a GimbalInfo message is received
        pass

    def init_ui(self):
        self.setWindowTitle('ROS Subscriber App')
        layout = QVBoxLayout()

        self.label = QLabel('Waiting for ROS messages...')
        layout.addWidget(self.label)
        
        self.track = QPushButton('Start Tracking')
        layout.addWidget(self.track)
        self.track.clicked.connect(self.track_start)

        self.id_input = QLineEdit()
        layout.addWidget(self.id_input)

        self.id_button = QPushButton('Publish ID')
        layout.addWidget(self.id_button)
        self.id_button.clicked.connect(self.publish_id)
        
        self.checkBox = QCheckBox('Plot Graph')
        layout.addWidget(self.checkBox)
        self.checkBox.stateChanged.connect(self.checkBox_changed)

        self.setLayout(layout)

    def checkBox_changed(self, state):
        if state == Qt.Checked:
            self.plot_initialized = True
        else:
            plt.close()
            self.plot_initialized = False

    def track_start(self):
        msg = Bool()
        msg.data = True
        self.tracking_cmd_publisher.publish(msg)

    def publish_id(self):
        try:
            id_info = int(self.id_input.text())
            msg = Int32()
            msg.data = id_info
            self.following_cmd_publisher.publish(msg)
        except ValueError:
            print("Invalid input for ID. Please enter a valid integer.")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app_instance = RosSubscriberApp()
    app_instance.show()
    exit_code = app.exec_()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
