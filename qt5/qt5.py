import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit ,QCheckBox
from PyQt5.QtCore import Qt, QTimer
from rclpy import init, shutdown
from rclpy.node import Node
from std_msgs.msg import String ,Bool , Int32
from avix_msg.msg import GimbalInfo ,TrackingUpdate

#plot graph
import matplotlib.pyplot as plt
import numpy as np
import time
from math import *

class RosSubscriberApp(QWidget):
    def __init__(self):
        super().__init__()
        init(args=None)
        self.node= Node("ros_subscriber")
        self.node.create_subscription(
            GimbalInfo,
            '/gimbal/info',
            self.listener_callback,
            10
        )
        self.node.create_subscription(
            TrackingUpdate,
            '/object_detection/target_deviation',
            self.deviation_callback,
            10
        )
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(100)
        self.init_ui()  
        
        #plot init 
        self.plt_init=True
        self.t=[]
        self.t_now=0
        self.mx=[]
        self.my=[]
        self.count=0

        
    def timer_callback(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
    def deviation_callback(self, msg):
        if  not self.plt_init :
            plt.clf()
            self.t_now=self.count*0.1
            self.t.append(self.t_now)
            self.mx.append(msg.deviation_x)
            plt.plot(self.t, self.mx,'-r' , label="x")

            self.my.append(msg.deviation_y)
            plt.plot(self.t, self.my, '-b', label="y")

            plt.legend()
            plt.pause(0.1)
            self.count+=1
            self.number=0
        else:
            #plot init 
            self.t=[]
            self.t_now=0
            self.mx=[]
            self.my=[]
            self.count=0
            
    def listener_callback(self, msg):
        pass   
        # if  not self.plt_init :
        #     plt.clf()
        #     self.t_now=self.count*0.1
        #     self.t.append(self.t_now)
        #     self.mx.append(msg.target_distance)
        #     plt.plot(self.t, self.mx,'-r' , label="y")

        #     self.my.append(msg.pitch_angle)
        #     plt.plot(self.t, self.my, '-b', label="x")

        #     plt.legend()
        #     plt.pause(0.1)
        #     self.count+=1
        #     self.number=0
        # else:
        #     #plot init 
        #     self.t=[]
        #     self.t_now=0
        #     self.mx=[]
        #     self.my=[]
        #     self.count=0
            
            
        # self.received_data = msg.target_distance
        # print(f"Received: {self.received_data}")
        # self.handle_received_data(self.received_data)
        
    def init_ui(self):
        self.setWindowTitle('ROS Subscriber App')

        layout = QVBoxLayout()

        self.label = QLabel('Waiting for ROS messages...')
        layout.addWidget(self.label)
        
        self.track = QPushButton('tracking start')
        layout.addWidget(self.track)
        self.track.clicked.connect(self.track_start)

        self.id_input = QLineEdit()
        layout.addWidget(self.id_input)

        self.id_button = QPushButton('Publish ID')
        layout.addWidget(self.id_button)
        self.id_button.clicked.connect(self.publish_id)
        
        self.checkBox_1 = QCheckBox('plot graph')
        layout.addWidget(self.checkBox_1)
        self.checkBox_1.stateChanged.connect(self.checkBox_1_changed)

        self.setLayout(layout)
        
    def checkBox_1_changed(self, state):
        if state == Qt.Checked:
            #plot init 
            plt.ion()
            plt.figure(1)
            self.plt_init=False
        
        else:
            #plot close
            plt.close(1)
            self.plt_init=True

    def handle_received_data(self, received_data):
        self.label.setText(f'Received: {received_data}')
    def track_start(self):
        node = Node('tracking_start')
        id_publisher = node.create_publisher(Bool, '/icp_interface/tracking_cmd', 10)
        msg = Bool()
        msg.data = True
        id_publisher.publish(msg)
        print(f'Tracking: {msg.data}')   

    def publish_id(self):
        node = Node('id_publisher')
        id_publisher = node.create_publisher(Int32, '/icp_interface/following_cmd', 10)
        try:
            id_info = int(self.id_input.text())
            print(type(id_info))
            msg = Int32()
            msg.data = id_info
            id_publisher.publish(msg)
            print(f'Published ID: {id_info}')    
        except:
            pass    



def main(args=None):
    app = QApplication(sys.argv)
    app_instance = RosSubscriberApp()  # Create an instance of RosSubscriberApp
    app_instance.show()
    sys.exit(app.exec_())
    
    
if __name__ == '__main__':
    main()
