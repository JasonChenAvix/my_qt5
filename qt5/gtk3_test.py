import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit ,QCheckBox ,QMainWindow
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
from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg as FigureCanvas

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
        self.topic=[ '/gimbal/info','/object_detection/target_deviation','/object_detection/detections', '/avix_mavros/follow_command', '/object_detection/target_gps' , '/ktg_gimbal/control', '/inf_interface/info' ,'avix_mavros/state']
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
        self.mav_subscriber = self.node.create_subscription(MavlinkState, 'avix_mavros/state', self.gps_mavlink_callback, 10)
        self.inf_subscriber = self.node.create_subscription(InfInfo, '/inf_interface/info', self.gps_inf_callback, 10)
        self.subscription = self.node.create_subscription(
            GimbalControl,
            '/ktg_gimbal/control',
            self.controlGimbal,
            10)
        self.detections_subscriber = self.node.create_subscription(
            ObjectDetections,
            '/object_detection/detections',
            self.detections_callback,
            10
        )
        self.cmd_subscription = self.node.create_subscription(
            FollowCommand,
            '/avix_mavros/follow_command',
            self.follow_command_callback,
            10)
        self.target_GPS_subscriber = self.node.create_subscription(
            TargetGPS,
            '/object_detection/target_gps',
            self.target_GPS_callback,
            10
        )
        self.init_ui()  
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        
        self.topic_count = 0 
        self.sendmsg="initialize!!!"
        
        
    def timer_callback(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        

    def init_ui(self):
        self.setWindowTitle(' Message Window ')
        layout = QVBoxLayout()
        self.fetch_button = QPushButton('fetch message')
        layout.addWidget(self.fetch_button)
        self.fetch_button.clicked.connect(self.fetch_message)
        self.init_fetch= True
        self.label = QLabel('message TEST')
        layout.addWidget(self.label)
        

        self.setLayout(layout)
        self.count=0

    def fetch_message(self):
        if self.timer.isActive():
            self.timer.stop()
            self.fetch_button.setText('Start')
        else:
            self.timer.start(100)
            self.fetch_button.setText('Stop')
        
    def ChangeTopic(self):
        self.topic_count += 1 
        self.topic_count=self.topic_count %len(self.topic)
        print(self.topic_count) 
        print(self.topic[self.topic_count])

        

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Tab:
            # self.label.setText(f'ranging_flag:{self.count}')
            # self.count+=1
            self.ChangeTopic()
            self.label.setText(f'Change Topic {self.topic[self.topic_count]}' )
            pass

    def target_GPS_callback(self, msg):
        if  self.topic[self.topic_count]== '/object_detection/target_gps' : 
            self.sendmsg = ( f'''
                            target_latitude :{msg.target_latitude},
                            target_longitude:{msg.target_longitude},
                            target_altitude :{msg.target_altitude},
                            estimate_status :{msg.estimate_status},
                            estimate_source :{msg.estimate_source},
                            heading :{msg.heading},
                            delta_north : {msg.delta_north}
                            delta_east : {msg.delta_east}
                            ''')
            self.label.setText(self.sendmsg)


    def gps_mavlink_callback(self , msg ):
        if  self.topic[self.topic_count]== 'avix_mavros/state' : 
            self.sendmsg = (f'''
                            latitude :{msg.latitude},
                            longitude :{msg.longitude},
                            altitude :{msg.altitude},
                            relative_altitude :{msg.relative_altitude},
                            heading :{msg.heading},
                            flight_mode :{msg.flight_mode},
                            roll :{msg.roll},
                            yaw :{msg.yaw},
                            pitch :{msg.pitch},
                            ''')
            self.label.setText(self.sendmsg)


    def follow_command_callback(self, msg):
        if  self.topic[self.topic_count]== '/avix_mavros/follow_command' : 
            self.sendmsg = ( f'''
                            latitude :{msg.latitude},
                            longitude :{msg.longitude},
                            altitude :{msg.altitude},
                            heading :{msg.heading},
                            estimate_status  :{msg.estimate_status},
                            estimate_source  :{msg.estimate_source},

                            ''')
            self.label.setText(self.sendmsg)


    def detections_callback(self, msg ):
        if  self.topic[self.topic_count]== '/object_detection/detections': 
            self.sendmsg = ( f'''
                            detections  :{msg.detections},
                            num_detections:{msg.num_detections},
                            ''')
            self.label.setText(self.sendmsg)


    def controlGimbal(self, msg):
        if self.topic[self.topic_count]== '/ktg_gimbal/control' : 
        
            self.sendmsg = ( f''' 
                            header:{msg.header} 
                            pan_velocity:{msg.pan_velocity} 
                            tilt_velocity:{msg.tilt_velocity} 
                            control_type :{msg.ranging_flag} 
                            trackbox_x_center:{msg.trackbox_x_center} 
                            trackbox_y_center :{msg.trackbox_y_center} 
                            trackbox_width :{msg.trackbox_width} 
                            trackbox_height :{msg.trackbox_height} 
                            ''')
            self.label.setText(self.sendmsg)


    def gps_inf_callback(self, msg ):
        if  self.topic[self.topic_count]== '/inf_interface/info'  : 
            self.sendmsg = (f'''
                            longitude:{msg.longitude} 
                            latitude:{msg.latitude} 
                            altitude:{msg.altitude} 
                            relative_altitude:{msg.relative_altitude} 
                            heading:{msg.heading} 
                            home_altitude:{msg.home_altitude} 
                                ''')
            self.label.setText(self.sendmsg)


    def deviation_callback(self, msg):
        if  self.topic[self.topic_count]== '/object_detection/target_deviation' : 
            self.sendmsg = ( f'''
                            header :{msg.header} 
                            deviation_x :{msg.deviation_x} 
                            deviation_y :{msg.deviation_y} 
                            resolution_x :{msg.resolution_x} 
                            resolution_y :{msg.resolution_y} 
                            size_x :{msg.size_x} 
                            size_y :{msg.size_y} 
                            ''')
            self.label.setText(self.sendmsg)

        
    def gimbal_callback(self, msg):
        #print(f"ranging_flag: {self.received_data}")
        if  self.topic[self.topic_count]== '/gimbal/info' : 
            self.sendmsg = ( f'''
                            pitch_angle :{msg.pitch_angle} 
                            yaw_angle :{msg.yaw_angle} 
                            roll_angle :{msg.roll_angle} 
                            target_distance :{msg.target_distance} 
                            ranging_flag :{msg.ranging_flag} 
                            eo_zoom :{msg.eo_zoom} ''')
            self.label.setText(self.sendmsg)
            

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
            TrackingUpdate,
            '/object_detection/target_deviation',
            self.deviation_callback,
            10
        )
   

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(1000)
        self.init_ui()  
        
        #plot init 
        self.plt_init=True
        self.t=[]
        self.t_now=0
        self.mx=[]
        self.my=[]
        self.count=0

        
        
    def timer_callback(self):
        rclpy.spin_once(self.node, timeout_sec=1)
        
    def deviation_callback(self, msg):
        if  not self.plt_init :
            self.win.update_data(msg.deviation_x,msg.deviation_y)           
        else:
            pass     

        
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

    def show_the_msg(self):
        if self.init_windows1:
            self.init_windows1=False
            self.window1 = MessageWindow()
            self.window1.show()
        else:
            self.window1.close()
            self.init_windows1=True


    def img_show_btm(self):
        if self.init_windows2:
            self.init_windows2=False
            self.window2 = ImageWindow()
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
