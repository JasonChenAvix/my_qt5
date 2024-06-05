import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit ,QCheckBox ,QHBoxLayout,QMainWindow , QStackedWidget, QSizePolicy, QShortcut
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt, QTimer
from rclpy import init, shutdown
from rclpy.node import Node
from std_msgs.msg import String ,Bool , Int32 
from sensor_msgs.msg import Image
from avix_msg.msg import GimbalInfo ,TrackingUpdate , InfInfo ,GimbalControl, MavlinkState, ObjectDetections,  FollowCommand, TargetGPS 

#plot graph
import matplotlib.pyplot as plt
import numpy as np
import time
from math import *
# for image show 
import cv2
from PyQt5.QtGui import QImage, QPixmap
from cv_bridge import CvBridge, CvBridgeError

#for gtk3
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class ImageWindow(QMainWindow):
    def __init__(self):
        super(ImageWindow, self).__init__()
        self.initUI()
        self.node= Node("ros_subscriber_image_show")
        self.image_subscriber = self.node.create_subscription(Image, '/for_qt5', self.image_callback, 10)

       
        # Set up a timer to call the update_frame method periodically
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Update every 30 ms (about 33 FPS)

    def initUI(self):
        self.setWindowTitle('Real-time Video Feed')

        # Create a QLabel widget to display the video
        self.videoLabel = QLabel(self)

        # Create a layout and add widgets
        layout = QVBoxLayout()
        layout.addWidget(self.videoLabel)
        self.frame = None

        # Create a button to start/stop the video feed
        self.controlButton = QPushButton('Stop', self)
        self.controlButton.clicked.connect(self.control_video)
        layout.addWidget(self.controlButton)

        # Set up the central widget
        centralWidget = QWidget()
        centralWidget.setLayout(layout)
        self.setCentralWidget(centralWidget)
        self.bridge = CvBridge()

        # Resize the main window
        self.setGeometry(100, 100, 640, 480)

    def image_callback(self, msg):
            self.frame = msg
         

    def update_frame(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.frame is not None:
            # Convert the frame to RGB
            frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Convert the frame to QImage
            
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            q_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Set the QImage to the QLabel
            self.videoLabel.setPixmap(QPixmap.fromImage(q_image))

    def control_video(self):
        if self.timer.isActive():
            self.timer.stop()
            self.controlButton.setText('Start')
        else:
            self.timer.start(500)
            self.controlButton.setText('Stop')

    def closeEvent(self, event):
        #self.cap.release()
        self.timer.stop()
        event.accept()

 


class MessageWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.node= Node("ros_subscriber_fetch_message")
        self.topic = [
                '/gimbal/info', '/object_detection/target_deviation', '/object_detection/detections',
                '/avix_mavros/follow_command', '/object_detection/target_gps', '/ktg_gimbal/control',
                '/inf_interface/info', 'avix_mavros/state'
            ]

        # Create a dictionary to store labels
        self.labels = {}

        # ROS Subscriptions
        self.node.create_subscription(GimbalInfo, '/gimbal/info', self.gimbal_callback, 10)
        self.node.create_subscription(TrackingUpdate, '/object_detection/target_deviation', self.deviation_callback, 10)
        self.node.create_subscription(MavlinkState, 'avix_mavros/state', self.gps_mavlink_callback, 10)
        self.node.create_subscription(InfInfo, '/inf_interface/info', self.gps_inf_callback, 10)
        self.node.create_subscription(GimbalControl, '/ktg_gimbal/control', self.controlGimbal, 10)
        self.node.create_subscription(ObjectDetections, '/object_detection/detections', self.detections_callback, 10)
        self.node.create_subscription(FollowCommand, '/avix_mavros/follow_command', self.follow_command_callback, 10)
        self.node.create_subscription(TargetGPS, '/object_detection/target_gps', self.target_GPS_callback, 10)

        

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.sendmsg = "initialize!!!"
        
        
        #  create image show 
        self.node1= Node("ros_subscriber_image_show")
        self.image_subscriber = self.node1.create_subscription(Image, '/for_qt5', self.image_callback, 10)

       
        # Set up a timer to call the update_frame method periodically
        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.update_frame)
        self.timer1.start(500)  # Update every 500 ms (about 500 FPS)
        

        # create for the pid controller
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        self.t_data=[]
        self.x_data = []
        self.y_data = []

        self.plot_data()
        self.x=0
        self.y=0
        self.timer2 = QTimer(self)
        self.timer2.timeout.connect(self.update_plot)
        self.init_ui()

    # image show 
    def image_callback(self, msg):
        self.frame = msg
         

    def update_frame(self):
        rclpy.spin_once(self.node1, timeout_sec=0.1)
        if self.frame is not None:
            # Convert the frame to RGB
            frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Convert the frame to QImage
            
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            q_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Set the QImage to the QLabel
            self.videoLabel.setPixmap(QPixmap.fromImage(q_image))

    def control_video(self):
        if self.timer1.isActive():
            self.timer1.stop()
            self.controlButton.setText('Start')
        else:
            self.timer1.start(500)
            self.controlButton.setText('Stop')

    
    
    # PID controller 
    def plot_data(self):
        self.ax.clear()
        self.ax.plot(self.t_data, self.x_data, color='r')
        self.ax.plot(self.t_data, self.y_data, color='b')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')
        plt.legend(['x', 'y'], loc='upper left')
        self.ax.set_title('Real-Time  Deviation Data Plot')
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
    def pid_control(self):
        if self.timer2.isActive():
            self.timer2.stop()
            self.data_reset()
            self.PIDButton.setText('Start')
        else:
            self.timer2.start(500)
            self.PIDButton.setText('Stop')

    def data_reset(self):
        self.t_data=[]
        self.x_data = []
        self.y_data = []


    def init_ui(self):
        self.setWindowTitle('Message Window')
        layout = QVBoxLayout()

        self.fetch_button = QPushButton('Fetch Message')
        layout.addWidget(self.fetch_button)
        self.fetch_button.clicked.connect(self.fetch_message)

        self.label = QLabel('Message TEST')
        layout.addWidget(self.label)

        self.stacked_widget = QStackedWidget(self)

        # Create a label for each topic and add to stacked widget
        for topic in self.topic:
            label = QLabel(f'{topic} Content', self)
            self.labels[topic] = label
            self.stacked_widget.addWidget(label)

        # Create buttons to switch between pages
        Hbox=QHBoxLayout()
        self.buttons = {}
        for i, topic in enumerate(self.topic):
            button = QPushButton(topic, self)
            self.buttons[topic] = button
            button.clicked.connect(lambda _, t=topic: self.show_topic_page(t))
            Hbox.addWidget(button)
        layout.addLayout(Hbox)
        # Create shortcuts for switching pages using Tab key
        self.page_count = 0
        shortcut_follow_mode = QShortcut(QKeySequence(Qt.Key_Tab), self)
        shortcut_follow_mode.activated.connect(self.change_page)

        # Add stacked widget and buttons layout to main layout
        layout.addWidget(self.stacked_widget)
        #initialize the label 
        font = self.buttons[self.topic[self.page_count]].font()
        size = font.pointSize()
        font.setPointSize(size +2)
        self.buttons[self.topic[self.page_count]].setFont(font)
        self.buttons[self.topic[self.page_count]].setStyleSheet( "color: blue;" )

        

        allimage=QHBoxLayout()
        # Create a layout and add widgets for the video play 
        #self.videoTitle = QLabel("image show ")
        #self.videoTitle.setAlignment(Qt.AlignCenter)
        self.videoLabel = QLabel(self)
        image = QVBoxLayout()
        #image.addWidget(self.videoTitle)
        image.addWidget(self.videoLabel)
        self.frame = None

        # Create a button to start/stop the video feed
        self.controlButton = QPushButton('Stop', self)
        self.controlButton.clicked.connect(self.control_video)
        image.addWidget(self.controlButton)

        # Create a layout and add widgets for the video play 
        self.PIDcontrollerLabel = self.canvas 
        PIDcontroller = QVBoxLayout()
        PIDcontroller.addWidget(self.PIDcontrollerLabel)
        self.frame = None

        # Create a button to start/stop the video feed
        self.PIDButton = QPushButton('Start', self)
        self.PIDButton.clicked.connect(self.pid_control)
        PIDcontroller.addWidget(self.PIDButton)


        # Set up the central widget

        self.bridge = CvBridge()
        allimage.addLayout(image)
        allimage.addLayout(PIDcontroller)
        layout.addLayout(allimage)

        self.setLayout(layout)
    # message page 
    def timer_callback(self):
        rclpy.spin_once(self.node, timeout_sec=1)
    
    def change_page(self):
        self.page_count=self.stacked_widget.currentIndex()
        self.page_count = (self.page_count + 1) % len(self.topic)
        self.update_button_sizes()
        self.stacked_widget.setCurrentIndex(self.page_count)
        self.label.setText(f'Change Topic {self.topic[self.page_count]}' )

    def show_topic_page(self, topic):
        index = self.topic.index(topic)
        self.page_count=index
        self.update_button_sizes()
        self.stacked_widget.setCurrentIndex(index)
        self.label.setText(f'Change Topic {self.topic[index]}' )
        

    def update_button_sizes(self):
        if self.stacked_widget.currentIndex() == self.page_count:
            return
        font = self.buttons[self.topic[self.page_count]].font()
        size = font.pointSize()

        for topic, button in self.buttons.items():
            font.setPointSize(size if self.page_count != self.topic.index(topic)  else size + 2)
            button.setFont(font)
            button.setStyleSheet("color: black;"  if self.page_count != self.topic.index(topic)  else "color: blue;"  )
        

    def fetch_message(self):
        if self.timer.isActive():
            self.timer.stop()
            self.fetch_button.setText('Start')
        else:
            self.timer.start(100)
            self.fetch_button.setText('Stop')
        

         
    def update_label(self, topic, msg):
        if self.topic[self.page_count] == topic:
            self.labels[topic].setText(msg)

    def target_GPS_callback(self, msg):
        if self.topic[self.page_count] == '/object_detection/target_gps':
            self.sendmsg = (f'''
                target_latitude: {msg.target_latitude},
                target_longitude: {msg.target_longitude},
                target_altitude: {msg.target_altitude},
                estimate_status: {msg.estimate_status},
                estimate_source: {msg.estimate_source},
                heading: {msg.heading},
            ''')
            self.update_label('/object_detection/target_gps', self.sendmsg)

    def gps_mavlink_callback(self, msg):
        if self.topic[self.page_count] == 'avix_mavros/state':
            self.sendmsg = (f'''
                latitude: {msg.latitude},
                longitude: {msg.longitude},
                altitude: {msg.altitude},
                relative_altitude: {msg.relative_altitude},
                heading: {msg.heading},
                flight_mode: {msg.flight_mode},
                roll: {msg.roll},
                yaw: {msg.yaw},
                pitch: {msg.pitch},
            ''')
            self.update_label('avix_mavros/state', self.sendmsg)

    def follow_command_callback(self, msg):
        if self.topic[self.page_count] == '/avix_mavros/follow_command':
            self.sendmsg = (f'''
                latitude: {msg.latitude},
                longitude: {msg.longitude},
                altitude: {msg.altitude},
                heading: {msg.heading},
                estimate_status: {msg.estimate_status},
                estimate_source: {msg.estimate_source},
            ''')
            self.update_label('/avix_mavros/follow_command', self.sendmsg)

    def detections_callback(self, msg):
        if self.topic[self.page_count] == '/object_detection/detections':
            self.sendmsg = (f'''
                detections: {msg.detections},
                num_detections: {msg.num_detections},
            ''')
            self.update_label('/object_detection/detections', self.sendmsg)

    def controlGimbal(self, msg):
        if self.topic[self.page_count] == '/ktg_gimbal/control':
            self.sendmsg = (f'''
                header: {msg.header},
                pan_velocity: {msg.pan_velocity},
                tilt_velocity: {msg.tilt_velocity},
                control_type: {msg.ranging_flag},
                trackbox_x_center: {msg.trackbox_x_center},
                trackbox_y_center: {msg.trackbox_y_center},
                trackbox_width: {msg.trackbox_width},
                trackbox_height: {msg.trackbox_height},
            ''')
            self.update_label('/ktg_gimbal/control', self.sendmsg)

    def gps_inf_callback(self, msg):
        if self.topic[self.page_count] == '/inf_interface/info':
            self.sendmsg = (f'''
                longitude: {msg.longitude},
                latitude: {msg.latitude},
                altitude: {msg.altitude},
                relative_altitude: {msg.relative_altitude},
                heading: {msg.heading},
                home_altitude: {msg.home_altitude},
            ''')
            self.update_label('/inf_interface/info', self.sendmsg)

    def deviation_callback(self, msg):
        self.update_data(msg.deviation_x,msg.deviation_y)
        if self.topic[self.page_count] == '/object_detection/target_deviation':
            self.sendmsg = (f'''
                header: {msg.header},
                deviation_x: {msg.deviation_x},
                deviation_y: {msg.deviation_y},
                resolution_x: {msg.resolution_x},
                resolution_y: {msg.resolution_y},
                size_x: {msg.size_x},
                size_y: {msg.size_y},
            ''')
            self.update_label('/object_detection/target_deviation', self.sendmsg)

    def gimbal_callback(self, msg):
        if self.topic[self.page_count] == '/gimbal/info':
            self.sendmsg = (f'''
                pitch_angle: {msg.pitch_angle},
                yaw_angle: {msg.yaw_angle},
                roll_angle: {msg.roll_angle},
                target_distance: {msg.target_distance},
                ranging_flag: {msg.ranging_flag},
                eo_zoom: {msg.eo_zoom},
            ''')
            self.update_label('/gimbal/info', self.sendmsg)

    def closeEvent(self, event):
        #self.cap.release()
        self.timer.stop()
        self.timer1.stop()
        event.accept()

class DynamicPlotWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title="tracking derviation with PID controller")
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


        self.init_ui()  

        
        
    def init_ui(self):
        #self.setWindowTitle('ROS Subscriber App')

        layout = QVBoxLayout()
        self.msg_button = QPushButton(' msg  windows')
        layout.addWidget(self.msg_button)
        self.msg_button.clicked.connect(self.show_the_msg)
        self.init_windows1=True
        self.init_windows2=True
    
        # self.gimbal_info = QLabel("gimbal_info")
        # layout.addWidget(self.gimbal_info)


        self.img_show = QPushButton('image_show')
        layout.addWidget(self.img_show)
        self.img_show.clicked.connect(self.img_show_btm)

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
        self.window1 = MessageWindow()
        self.window2 = ImageWindow()

        

    def show_the_msg(self):
        if not self.window1.isVisible():
            self.init_windows1=False
            self.window1.show()
        else:
            self.window1.close()
            self.init_windows1=True


    def img_show_btm(self):
        if  not self.window2.isVisible():
            self.init_windows2=False
            self.window2.show()
        else:
            self.window2.close()
            self.init_windows2=True

    
     
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
