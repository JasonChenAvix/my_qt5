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

#for gtk3
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject
from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg as FigureCanvas




class DynamicPlotWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title="Dynamic Plot with GTK3 and Matplotlib")
        self.set_default_size(600, 400)
        self.connect("destroy", Gtk.main_quit)

        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        self.add(self.canvas)
        self.t_data=[]
        self.x_data = []
        self.y_data = []

        self.plot_data()
        self.x=0
        self.y=0
        self.timer_id = GObject.timeout_add(500, self.update_plot)  # Update plot every second

    def plot_data(self):
        self.ax.clear()
        self.ax.plot(self.t_data, self.x_data, color='b')
        self.ax.plot(self.t_data, self.y_data, color='r')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')
        plt.legend(['x', 'y'], loc='upper left')
        self.ax.set_title('Real-Time Data Plot')
        self.canvas.draw()
    def update_data(self,x,y):
        self.x=x
        self.y=y
        print("x:",x,"y:",y)

    def update_plot(self):
        # Generate random data for demonstration
        new_t = len(self.t_data)
        new_x = self.x
        new_y = self.y
        
        self.t_data.append(new_t)
        self.x_data.append(new_x)
        self.y_data.append(new_y)
        
        self.plot_data()

        return True


class RosSubscriberApp(QWidget):
    def __init__(self):
        super().__init__()
        init(args=None)
        self.node= Node("ros_subscriber")
        self.node.create_subscription(
            GimbalInfo,
            '/gimbal/info',
            self.gimbal_callback,
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
            self.win.update_data(msg.deviation_x,msg.deviation_y)
            
        else:
            pass
        
        
    def gimbal_callback(self, msg):
              
    
        #print(f"ranging_flag: {self.received_data}")
        self.gimbal_info.setText(f'ranging_flag: {msg.ranging_flag}, target_distance: {msg.target_distance}')
        
    def init_ui(self):
        self.setWindowTitle('ROS Subscriber App')

        layout = QVBoxLayout()
        
        self.gimbal_info = QLabel("gimbal_info")
        layout.addWidget(self.gimbal_info)

        self.following = QPushButton('drone following start')
        layout.addWidget(self.following)
        self.following.clicked.connect(self.following_start)
        
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
            self.plt_init=False
            # GTK
            self.win = DynamicPlotWindow()
            self.win.connect("destroy", Gtk.main_quit)
            self.win.show_all()
            Gtk.main()
            
        else:
            #plot close
            self.plt_init=True
            self.win.close()
            #Gtk.main_quit()
            
            

    
    def following_start(self):
        node = Node('following_start')
        node_publisher = node.create_publisher(Bool, '/mq3/start_following', 10)
        msg = Bool()
        msg.data = True
        node_publisher.publish(msg)
        print(f'Following: {msg.data}')
     
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
