import sys
import rclpy
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QKeySequence ,QIcon , QGuiApplication, QMouseEvent
from PyQt5.QtCore import Qt, QTimer
from rclpy import init, shutdown
from rclpy.node import Node
from std_msgs.msg import String ,Bool , Int32 
from sensor_msgs.msg import Image
from avix_utils.msg import MQ3State, GimbalInfo ,TrackingUpdate , InfInfo ,GimbalControl, MavlinkInfo, ObjectDetections,  FollowCommand, TargetGPS  ,VisualTrackingCommand
from avix_utils_py import avix_common
from avix_utils.srv import EnableFunction,  ObjectDetectionStatus, DroneFollowingStatus, GimbalTrackingStatus
from avix_utils_py.avix_error_codes import get_error_message
from avix_utils_py.avix_enums import ErrorMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup ,ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


import os
os.environ['LD_PRELOAD']="/home/nvidia/.local/lib/python3.8/site-packages/scikit_learn.libs/libgomp-d22c30c5.so.1.0.0"

import pandas as pd 

#plot graph
import matplotlib.pyplot as plt
import numpy as np
import time
from math import *
# for image show 
import cv2
from PyQt5.QtGui import QImage, QPixmap
from cv_bridge import CvBridge, CvBridgeError

#for plot the pid image 
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

#for plot the image show for the detection 
from copy import deepcopy
from ultralytics.utils.plotting import Annotator, colors
#basic setting
line_width=None
font_size=None
font='Arial.ttf'
pil = False
rtsp_url = "mot4.mp4"
# for gimbal controller 
from avix_controllers.avix_controllers import PIDController



class mainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        init(args=None)
        self.node= Node("ros_subscriber_fetch_message")
        
        # ROS Subscriptions
        self.node.create_subscription(GimbalInfo, avix_common.KTG_INFO, self.gimbal_callback, 10)
        self.node.create_subscription(ObjectDetections, avix_common.OBJECT_DETECTIONS, self.detections_callback,10)
        self.node.create_subscription(MavlinkInfo, avix_common.MAVLINK_INFO, self.gps_mavlink_callback, 10)
        self.node.create_subscription(InfInfo, avix_common.INF_INFO, self.gps_inf_callback, 10)
        self.node.create_subscription(GimbalControl, avix_common.KTG_CONTROL, self.controlGimbal, 10)
        self.node.create_subscription(FollowCommand, avix_common.MAVLINK_FOLLOW_CMD, self.follow_command_callback, 10)     
        self.node.create_subscription(MQ3State, avix_common.MQ3_STATUS, self.mq3_status_callback, 10)
        
        #  create image show 
        self.image_subscriber = self.node.create_subscription(Image,avix_common.KTG_EO_IMG , self.video_callback, 10)

        #create the publisher 
        #self.follow_publisher = self.node.create_publisher(Bool, '/mq3/start_following', 10)
        #self.track_start_publisher = self.node.create_publisher(Bool, '/icp_interface/tracking_cmd', 10)
        #self.id_publisher = self.node.create_publisher(Int32, '/icp_interface/following_cmd', 10)
        

        self.mq3_status_publisher = self.node.create_publisher(MQ3State, avix_common.MQ3_STATUS, 10)
        self.id_publisher = self.node.create_publisher(Int32, avix_common.ICP_TARGET_ID_CMD, 10)
        self.control_publisher = self.node.create_publisher(GimbalControl, avix_common.KTG_CONTROL, 10)
        self.vis_publisher = self.node.create_publisher(
            VisualTrackingCommand, 
            avix_common.MQ3_VISUAL_TRACKING_CMD, 
            10
        )

       
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.sendmsg = "initialize!!!"
        self.init_ui()

        # Initialize CV bridge
        self.bridge = CvBridge()
        self.video_num=1
        self.fps= 10.0
        
        # record data init
        self.video_data=[]
        self.yolo_data=[]
        self.botsort_data=[]
        self.avix_data=[]
        self.target_data=[]
        self.gimbal_data=[]
        self.filename=1
        self.video_init= True
        self.timestamp=0
        
        
        self.mode = 'init'
    def init_ui(self):
        self.setWindowTitle('AVIX Detection System V1.0.1')
        self.setGeometry(100, 100, 800, 800)
        
        layout=QHBoxLayout()
        
        # # command layout 
        # ## fetch message button
        # commandlayout=QVBoxLayout()
        # self.fetch_button = QPushButton('Start detection system')
        # self.fetch_button.setFixedSize(170, 50)
        # commandlayout.addWidget(self.fetch_button)
        # self.fetch_button.clicked.connect(self.fetch_message)
        
        # ## Drone following enable button
        # self.following = QPushButton('Drone following start')
        # self.following.setFixedSize(170, 50)
        # commandlayout.addWidget(self.following)
        # self.following.clicked.connect(self.following_start)
        # self.follow_init = False
        
        # ## Tracking enable button
        # self.track = QPushButton('Tracking start')
        # self.track.setFixedSize(170, 50)
        # commandlayout.addWidget(self.track)
        # self.track.clicked.connect(self.track_start)
        # self.track_init = False
        
        # # Tracking enable button
        # self.vis_track = QPushButton('Visual Tracking start')
        # self.vis_track.setFixedSize(170, 50)
        # commandlayout.addWidget(self.vis_track)
        # self.vis_track.clicked.connect(self.visual_track_start)
        # self.visual_track_init = False
        
        
        #  ## Publish ID button
        # input_widget= QFrame()
        # input_widget.setFrameStyle(QFrame.Box)
        # input_widget.setFixedWidth(170)
        # input_layout=QVBoxLayout()
        # input_label= QLabel('Input the tracking id:')
        # input_layout.addWidget(input_label)
        # self.id_input = QLineEdit()
        # self.id_input.setFixedSize(150, 30)
        # input_layout.addWidget(self.id_input)
        # self.id_button = QPushButton('Publish ID')
        # self.id_button.setFixedSize(150, 50)
        # input_layout.addWidget(self.id_button)
        # self.id_button.clicked.connect(self.publish_id)
        # input_widget.setLayout(input_layout)
        # commandlayout.addWidget(input_widget)

        # ## Record data button
        # self.record_button = QPushButton('Record data')
        # self.record_button.setFixedSize(170, 50)
        # commandlayout.addWidget(self.record_button)
        # self.record_button.clicked.connect(self.record_start)
        # self.record_init = True
        
        # ## Command Status Box
        # # 创建文本框
        # self.textbox = QTextEdit(self)
        # self.textbox.setFixedSize(170, 300)
        # self.textbox.setReadOnly(True)  # 设置文本框为只读
        # self.textbox.setPlaceholderText('Waiting for command...')
        # commandlayout.addWidget(self.textbox)


        # commandlayout.addStretch()
        # # Create a grid layout for the arrow buttons
        # grid = QGridLayout()

        # # Button and their positions in the grid
        # path = '/home/nvidia/avix/arrow_icon/'
        # buttons = {
        #     (0, 1): ("up", path+"up.png"),
        #     (1, 0): ("left", path+"left.png"),
        #     (1, 1): ("center", path+"center.png"),
        #     (1, 2): ("right", path+"right.png"),
        #     (2, 1): ("down", path+"down.png"),
        #     (0, 0): ("left-up", path+"left_up.png"),
        #     (0, 2): ("right-up", path+"right_up.png"),
        #     (2, 0): ("left-down", path+"left_down.png"),
        #     (2, 2): ("right-down", path+"right_down.png")
        # }

        # # Create buttons and place them in the grid
        # for position, (name, icon) in buttons.items():
        #     button = QPushButton()
        #     button.setIcon(QIcon(icon))
        #     button.setIconSize(button.sizeHint())
        #     button.clicked.connect(lambda _, name=name: self.control_clicked(name))
        #     grid.addWidget(button, *position)

        # commandlayout.addLayout(grid)
        # self.set_center=False

        # #Content Layout

        center = QWidget()
        contentlayout = QVBoxLayout(center)
        
        # Messge windows
        # self.message =MessageLayout()
        # contentlayout.addWidget(self.message)
        

        # allimage=QHBoxLayout()
        
        # # Image show windows
        # self.image = ImageShowLayout()

        # # PID deviation image 
        # self.PIDcontroller=PIDContorllerImage()
        # self.PIDcontroller.setFixedSize(740,580)


        # allimage.addWidget(self.image)
        # allimage.addWidget(self.PIDcontroller)
        # contentlayout.addLayout(allimage)
        # add the status layout 
        test = ServiceStatus()
        contentlayout.addWidget(test)

        # layout.addLayout(commandlayout)
        # layout.addLayout(contentlayout)
        

        
        self.setCentralWidget(center)
       

    def timer_callback(self):
        #rclpy.spin_once(self.node, timeout_sec=0.1)
        self.executor.spin_once( timeout_sec=0.5 )

    #update  the command label 
    def update_command_label(self,str):
         # 按钮点击事件处理函数，记录按钮点击的历史
        current_text = self.textbox.toPlainText()
        new_text = str
        self.textbox.setPlainText(new_text + '\n'+'-----'+'\n' + current_text )
    
    # buttom click function 
    def control_clicked(self, name):
        if name == 'up':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(0.0)
            control_msg.tilt_velocity = float(2000.0)

            self.control_publisher.publish(control_msg)
            print('up')
        elif name == 'down':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(0.0)
            control_msg.tilt_velocity = float(-2000.0)

            self.control_publisher.publish(control_msg)
            print('down')
        elif name == 'left':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(-2000.0)
            control_msg.tilt_velocity = float(0.0)

            self.control_publisher.publish(control_msg)
            print('left')
        elif name == 'right':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(2000.0)
            control_msg.tilt_velocity = float(0.0)

            self.control_publisher.publish(control_msg)
            print('right')
        elif name == 'left-up':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(-2000.0)
            control_msg.tilt_velocity = float(2000.0)

            self.control_publisher.publish(control_msg)
            print('left-up')
        elif name == 'left-down':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(-2000.0)
            control_msg.tilt_velocity = float(-2000.0)

            self.control_publisher.publish(control_msg)
            print('left-down')
        elif name == 'right-up':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(2000.0)
            control_msg.tilt_velocity = float(2000.0)

            self.control_publisher.publish(control_msg)
            print('right-up')
        elif name == 'right-down':
            control_msg = GimbalControl()
            control_msg.pan_velocity = float(2000.0)
            control_msg.tilt_velocity = float(-2000.0)

            self.control_publisher.publish(control_msg)
            print('right-down')
        elif name == 'center':
            #TODO: move to center
            self.set_center=True
            print("center")
           
    def fetch_message(self):
        if self.timer.isActive():
            self.timer.stop()
            self.fetch_button.setText('Start to fetch data')
            self.update_command_label("Stop fetching ")
        else:
            self.timer.start(50)
            self.fetch_button.setText('Stop to fetch data')
            self.update_command_label(f"Start fetching ros \n message ")
    
    def record_start(self):
        if self.record_init:
            self.record_init=False
            self.record_button.setStyleSheet("background-color: red")
            self.record_button.setText('Stop to record data')
            self.update_command_label(f'Start recording the \n data')
        else:  
            self.record_init=True
            self.record_button.setText('Start to record data')
            self.record_button.setStyleSheet("background-color: white")
            self.filename+=1
            self.video_init= True
            self.video_num=0
            self.update_command_label(f'Stop recording the \n data')
            self.out.release()

            pass 
         
    def following_start(self):
        # msg = Bool()
        # msg.data = True
        # self.follow_publisher.publish(msg)
        # self.update_command_label(f'Following: {msg.data}')
        # print(f'Following: {msg.data}')

        # self.mq3_status_publisher.publish(MQ3State(following_enabled=True))
        # self.update_command_label(f'Following: True')
        # print(f'Following: True') 
        if not self.follow_init:
            temp_request = EnableFunction.Request()
            temp_request.enable = True
            future = self.cli_drone_following_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future, timeout_sec= 5)
            future_check_response = future.result()
            print(future_check_response)
            if future_check_response.success:
                self.service["Following System"].setStyleSheet("background-color: green")
            self.update_command_label(f'Following: True')
            self.following.setText(f'Following False')
            print(f'Following: True') 
            self.follow_init = True
        else:
            temp_request = EnableFunction.Request()
            temp_request.enable = False
            future = self.cli_drone_following_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future, timeout_sec= 5)
            future_check_response = future.result()
            print(future_check_response)
            if future_check_response.success:
                self.service["Following System"].setStyleSheet("background-color: red")
            self.update_command_label(f'Following: False')
            self.following.setText(f'Following True')
            print(f'Following: False')
            self.follow_init = False
     
    def track_start(self):
        # msg = Bool()
        # msg.data = True
        # self.track_start_publisher.publish(msg)
        # self.mq3_status_publisher.publish(MQ3State(tracking_enabled=True, detection_enabled=True))
        

        if not self.track_init:
            temp_request = EnableFunction.Request()
            temp_request.enable = True
            future = self.cli_gimbal_tracking_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future, timeout_sec= 3)
            future_check_response = future.result()
            print(future_check_response)
            if future_check_response.success :
                self.update_command_label(f'Open Tracking: True')
            else:
                self.update_command_label(f'Open Tracking: False \n reason: {future_check_response.message}')
        

            future1 = self.cli_object_detection_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future1, timeout_sec= 3)
            future1_check_response = future1.result()
            print(future1_check_response)
            if future1_check_response.success :
                self.update_command_label(f'Open Detection: True')
            else:
                self.update_command_label(f'Open Detection: False \n reason: {future1_check_response.message}')
        
            #self.update_command_label(f'Tracking: True')
            self.track.setText(f'Tracking False')
            print(f'Tracking: True') 
            self.track_init = True

        else:
            temp_request = EnableFunction.Request()
            temp_request.enable = False
            future = self.cli_gimbal_tracking_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future, timeout_sec= 3)
            future_check_response = future.result()
            print(future_check_response)
            if future_check_response.success :
                self.update_command_label(f'Close Tracking: True')
            else:
                self.update_command_label(f'Close Tracking: False \n reason: {future_check_response.message}')
        
            future1= self.cli_object_detection_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future1, timeout_sec= 3)
            future1_check_response = future1.result()
            print(future1_check_response)
            if future1_check_response.success :
                self.update_command_label(f'Close Detection: True')
            else:
                self.update_command_label(f'Close Detection: False \n reason: {future1_check_response.message}')
        
            #self.update_command_label(f'Tracking: False')
            self.track.setText(f'Tracking True')
            print(f'Tracking: False')
            self.track_init = False
            self.image.detection_init=False

    def visual_track_start(self):
        # msg = Bool()
        # msg.data = True
        # self.track_start_publisher.publish(msg)
        # self.mq3_status_publisher.publish(MQ3State(tracking_enabled=True, detection_enabled=True))
        

        if not self.track_init:
            temp_request = EnableFunction.Request()
            temp_request.enable = True
            future = self.cli_gimbal_tracking_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future, timeout_sec= 3)
            future_check_response = future.result()
            print(future_check_response)
            if future_check_response.success :
                self.update_command_label(f'Open  visual Tracking: True')
            else:
                self.update_command_label(f'Open  visual Tracking: False \n reason: {future_check_response.message}')
        

            future1 = self.cli_visual_detection_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future1, timeout_sec= 3)
            future1_check_response = future1.result()
            print(future1_check_response)
            if future1_check_response.success :
                self.update_command_label(f'Open visual  Detection: True')
            else:
                self.update_command_label(f'Open visual Detection: False \n reason: {future1_check_response.message}')
        
            #self.update_command_label(f'Tracking: True')
            self.vis_track.setText(f'visual Tracking False')
            print(f'visual Tracking: True') 
            self.vis_track_init = True

        else:
            temp_request = EnableFunction.Request()
            temp_request.enable = False
            future = self.cli_gimbal_tracking_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future, timeout_sec= 3)
            future_check_response = future.result()
            print(future_check_response)
            if future_check_response.success :
                self.update_command_label(f'Close visual Tracking: True')
            else:
                self.update_command_label(f'Close visual Tracking: False \n reason: {future_check_response.message}')
        
            future1= self.cli_visual_detection_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future1, timeout_sec= 3)
            future1_check_response = future1.result()
            print(future1_check_response)
            if future1_check_response.success :
                self.update_command_label(f'Close visual Detection: True')
            else:
                self.update_command_label(f'Close visual Detection: False \n reason: {future1_check_response.message}')
        
            #self.update_command_label(f'Tracking: False')
            self.vis_track.setText(f'visual Tracking True')
            print(f'visual Tracking: False')
            self.vis_track_init = False
           

    def publish_id(self):
        try:
            id_info = int(self.id_input.text())
            self.PIDcontroller.ID_init=id_info
            msg = Int32()
            msg.data = id_info %128
            self.id_publisher.publish(msg)
            self.update_command_label(f'Published ID: {id_info}')
            print(f'Published ID: {id_info}')    
        except:
            pass   

    # ros callback function
    def mq3_status_callback(self,msg):
     
        if msg.detection_enabled:
            self.service["Detection System"].setStyleSheet("background-color: green")
        else: 
            self.service["Detection System"].setStyleSheet("background-color: red")

        if msg.tracking_enabled:
            self.service["Tracking System"].setStyleSheet("background-color: green")
        else :
            self.service["Tracking System"].setStyleSheet("background-color: red")
            
        if msg.following_enabled:
            self.service["Following System"].setStyleSheet("background-color: green")
        else:
            self.service["Following System"].setStyleSheet("background-color: red")

        if msg.boot_up_sucess:
            self.service["System Bootup"].setStyleSheet("background-color: green")
        else:
            self.service["System Bootup"].setStyleSheet("background-color: red")

    
    def video_callback(self, msg):
        
        self.image.frame = msg
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.node.get_logger().error(f'Error converting ROS Image to OpenCV: {e}')
            return
           
        cv2.imshow(self.display_name, cv_image) 

        #record video
        if self.record_init: 
            return
        
        
        self.video_num+=1
        cap=cv_image
        width = cv_image.shape[1]    # 取得影像寬度
        height = cv_image.shape[0]   # 取得影像高度
           # 取得影像FPS
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') # 建立儲存影片的物件
        
        if self.video_init:
            self.out = cv2.VideoWriter(f'output{self.filename}.mp4', fourcc, self.fps, (width,  height))
            self.video_init=False
            print ("start video")
        # Display the resulting frame   

      
        self.out.write(cap)     # 將取得的每一幀圖像寫入空的影片
        #cv2.imshow("frame",cap)
        #cv2.waitKey(1)
        if self.video_num > self.fps*200:  # 每200秒存一次
            self.video_num=0
            self.filename+=1
            self.video_data=[]
            self.yolo_data=[]
            self.botsort_data=[]
            self.out = cv2.VideoWriter(f'output{self.filename}.mp4', fourcc, self.fps, (width,  height))
            print ("change video")
            time.sleep(1)
        pass


    def gps_mavlink_callback(self, msg):
        if self.message.topic[self.message.page_count] == avix_common.MAVLINK_INFO:
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
            self.message.update_label(avix_common.MAVLINK_INFO, self.sendmsg)
        if not self.record_init:
            self.record_avix_data(msg)

    def follow_command_callback(self, msg):
        if self.message.topic[self.message.page_count] ==  avix_common.MAVLINK_FOLLOW_CMD:
            self.sendmsg = (f'''
                latitude: {msg.latitude},
                longitude: {msg.longitude},
                altitude: {msg.altitude},
                heading: {msg.heading},
                estimate_status: {msg.estimate_status},
                estimate_source: {msg.estimate_source},
            ''')
            self.message.update_label(avix_common.MAVLINK_FOLLOW_CMD, self.sendmsg)

    def controlGimbal(self, msg):
        if self.message.topic[self.message.page_count] == avix_common.KTG_CONTROL :
            self.sendmsg = (f'''
                header: {msg.header},
                pan_velocity: {msg.pan_velocity},
                tilt_velocity: {msg.tilt_velocity},
                control_type: {msg.control_type},
                trackbox_x_center: {msg.trackbox_x_center},
                trackbox_y_center: {msg.trackbox_y_center},
                trackbox_width: {msg.trackbox_width},
                trackbox_height: {msg.trackbox_height},
            ''')
            self.message.update_label(avix_common.KTG_CONTROL,  self.sendmsg)

    def gps_inf_callback(self, msg):
        if self.message.topic[self.message.page_count] == avix_common.INF_INFO:
            self.sendmsg = (f'''
                longitude: {msg.longitude},
                latitude: {msg.latitude},
                altitude: {msg.altitude},
                relative_altitude: {msg.relative_altitude},
                heading: {msg.heading},
                home_altitude: {msg.home_altitude},
            ''')
            self.message.update_label(avix_common.INF_INFO,  self.sendmsg)

    def detections_callback(self, msg):
        self.image.detection_init=True
        self.image.detections =msg.detections
        self.PIDcontroller.detections=msg.detections 
        if self.message.topic[self.message.page_count] == avix_common.OBJECT_DETECTIONS:
            self.sendmsg = (f'''
                detections: {msg.detections}
                num_detections: {msg.num_detections}
            ''')
            self.message.update_label(avix_common.OBJECT_DETECTIONS,  self.sendmsg) 
            
    def gimbal_callback(self, msg):
        # save for the control gimbal to center
        if self.set_center:
            pitch_angle = msg.pitch_angle
            yaw_angle = msg.yaw_angle
            self.roll_angle = msg.roll_angle
            control_msg = GimbalControl()
            ctl_pan, ctl_tilt = self.calculate_control_command(yaw_angle, pitch_angle, 180, 90)
            control_msg.pan_velocity = float(ctl_pan)
            control_msg.tilt_velocity = float(ctl_tilt)
            self.control_publisher.publish(control_msg)
            if pitch_angle<2.5 and yaw_angle<2.5:
                self.set_center = False

        
        if self.message.topic[self.message.page_count] == avix_common.KTG_INFO:
            self.sendmsg = (f'''
                pitch_angle: {msg.pitch_angle},
                yaw_angle: {msg.yaw_angle},
                roll_angle: {msg.roll_angle},
                target_distance: {msg.target_distance},
                ranging_flag: {msg.ranging_flag},
                eo_zoom: {msg.eo_zoom},
            ''')
            self.message.update_label(avix_common.KTG_INFO, self.sendmsg)
        if  not self.record_init:
            self.record_gimbal_data(msg)

    # to calculate the speed to the center
    def calculate_control_command(self, yaw_angle,pitch_angle,max_yaw_angle,max_pitch_angle):
        # Placeholder for your control algorithm
        point_cx_dif = (yaw_angle)/max_yaw_angle
        point_cy_dif = (pitch_angle)/max_pitch_angle
        # if point_cx_dif>1 or point_cy_dif>1:
        #     self.node.get_logger().info(f'point_cx_dif {point_cx_dif} , point_cy_dif {point_cy_dif}')
            

        # plug in the controller
        speedx , X_p , X_i, X_d= self.pidx.update_with_output(point_cx_dif)
        speedy , Y_p , Y_i, Y_d= self.pidy.update_with_output(point_cy_dif)
        
        #if speedx>1 or speedy>1:
        # self.node.get_logger().info(f'speedxraw {speedx} , speedyraw {speedy}')
        
        # cap the speed to required format in KTG
        speed_x =min(max(speedx * 4000,-8000),8000) 
        speed_y =min(max(speedy * 4000,-8000),8000) 
        self.node.get_logger().info(f'speed_x {speed_x}, speed_y {speed_y}')
        
        # 600 is threshold
        if abs(speed_x)<600 and abs(speed_x)>200:
            if speed_x>0:
                speed_x=600
            else:
                speed_x=-600
        
        if abs(speed_y)<600 and abs(speed_y)>200:
            if speed_y>0:
                speed_y=600
            else:
                speed_y=-600
           
        return speed_x, speed_y 
    #record the data
    def record_gimbal_data(self,msg):
        objects_data3=[]
        video_time = self.video_num/self.fps
        objects_data3.append(self.timestamp)
        objects_data3.append(video_time)
        objects_data3.append(msg.pitch_angle)
        objects_data3.append(msg.yaw_angle)
        objects_data3.append(msg.target_distance)
        objects_data3.append(msg.ranging_flag)
        objects_data3.append(msg.eo_zoom)
        
        self.gimbal_data.append(objects_data3)
       
        if len(self.gimbal_data)>0:
            self.gimbal_df=pd.DataFrame(self.gimbal_data , columns=[ "timestamp","video time", "pitch_angle","yaw_angle","target_distance" ,"ranging_flag", "eo_zoom"])
            self.gimbal_df.to_csv(f"gimbal{self.filename}.csv", index=False)
            #print("save")

    def record_avix_data(self, msg ):
        objects_data1=[]
        
        self.timestamp+=1
        video_time = self.video_num/self.fps
        objects_data1.append(self.timestamp)
        objects_data1.append(video_time)
        objects_data1.append(msg.latitude)
        objects_data1.append(msg.longitude)
        objects_data1.append(msg.altitude)
        objects_data1.append(msg.relative_altitude)
        objects_data1.append(msg.heading)
        objects_data1.append(msg.flight_mode)
        objects_data1.append(msg.roll)
        objects_data1.append(msg.yaw)
        objects_data1.append(msg.pitch)
        
        
        self.avix_data.append(objects_data1)
       
        if len(self.avix_data)>0:
            self.avix_df=pd.DataFrame(self.avix_data , columns=[ "timestamp","video time ","latitude","longitude","altitude" ,"relative_altitude","heading","flight_mode" , "roll","yaw","pitch" ])
            self.avix_df.to_csv(f"avix{self.filename}.csv", index=False)
            #print("save")

    def record_object_data(self, msg):
        objects_data2=[]
    
        video_time = self.video_num/self.fps
        objects_data2.append(self.timestamp)
        objects_data2.append(video_time)
        objects_data2.append(msg.target_latitude)
        objects_data2.append(msg.target_longitude)
        objects_data2.append(msg.target_altitude)
        objects_data2.append(msg.estimate_status)
        objects_data2.append(msg.estimate_source)
        objects_data2.append(msg.heading)
        objects_data2.append(msg.delta_north)
        objects_data2.append(msg.delta_east)
        
        self.target_data.append(objects_data2)

        time.sleep(0.5)
       
        if len(self.target_data)>0:
            self.target_df=pd.DataFrame(self.target_data , columns=[ "timestamp", "video time ", "target_latitude","target_longitude","target_altitude" ,"estimate_status", "estimate_source","heading","delta_north", "delta_east"])
            self.target_df.to_csv(f"target{self.filename}.csv", index=False)
            #print("save")

    #close windows
    def closeEvent(self, event):
        #self.cap.release()
        self.timer.stop()
        event.accept()


class ServiceStatus(QWidget):
    def __init__(self):
        super().__init__()  
        
        self.node=Node('Service_Status')
        #set timer to check the status
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(1000)

        self.object_detection_response = None
        self.gimbal_tracking_response = None
        self.visual_tracking_response = None
        self.drone_following_response = None
        
        ##change to client
        self.cli_object_detection_enable = self.node.create_client(EnableFunction, avix_common.MQ3_ENABLE_OBJECT_DETECTION_SRV)
        self.cli_gimbal_tracking_enable = self.node.create_client(EnableFunction, avix_common.MQ3_ENABLE_GIMBAL_TRACKING_SRV)
        self.cli_drone_following_enable = self.node.create_client(EnableFunction, avix_common.MQ3_ENABLE_DRONE_FOLLOWING_SRV)
        self.cli_visual_tracking_enable = self.node.create_client(EnableFunction,avix_common.ENABLE_VISUAL_TRACKING)
        # create the mutli thread executor
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.init_ui()
        print('All service are ready. MQ3 service success')
        


    def init_ui(self):
        
        service_list = [
            "Object Detection System","Gimbal Tracking System", "Visual Tracking System","Drone Following System" 
       ]
        self.service = {}
        self.sub_service = {
            "Object Detection System": ["enabled: " , "is_intializing: " , "is_detecting: ", "detection_mode: "],
            "Gimbal Tracking System": ["enabled: " , "is_tracking: " , "target_id: "],
            "Visual Tracking System": ["enabled: " , "is_intializing: " , "is_tracking: " , "tracking_mode: " , "tracking_number: "],
            "Drone Following System": ["enabled: "  , "is_following: " , "target_id: " , "heading_locked: " , "following_mode: "],
  

        }
        self.button={}
        statuslayout = QGridLayout()
        count=0      
        for  system_name  in service_list:
            temp_widget = QWidget()
            main_layout = QVBoxLayout(temp_widget) 
            temp_widget.setStyleSheet("""
                QWidget#container {
                    border: 2px solid black;
                }
            """)
            temp_widget.setObjectName("container")  
            
            label = QHBoxLayout()
            self.service[system_name] = QLabel()
           
            self.service[system_name].setStyleSheet("background-color: red")
            self.service[system_name].setFixedSize(20, 20)
            systemlabelname = QLabel()
            systemlabelname.setText(system_name)
            label.addWidget(self.service[system_name])
            label.addWidget(systemlabelname)
            main_layout.addLayout(label)
            for sub_name in self.sub_service[system_name]:
                temp_layout = QHBoxLayout()
                sublabel = QLabel()
                sublabel.setText(sub_name)
                self.service[system_name, sub_name] = QLabel()
                temp_layout.addWidget(sublabel)
                temp_layout.addWidget(self.service[system_name, sub_name])
                main_layout.addLayout(temp_layout)

            temp_layout = QHBoxLayout()
            self.button[system_name,1] = QPushButton('ON')
            temp_layout.addWidget(self.button[system_name,1])
            self.button[system_name,1].clicked.connect(self.service_calling(system_name,1))
            self.button[system_name,0] = QPushButton('OFF')
            temp_layout.addWidget(self.button[system_name,0])
            self.button[system_name,0].clicked.connect(self.service_calling(system_name,0))
            main_layout.addLayout(temp_layout)

            statuslayout.addWidget(temp_widget, count // 2, (count)%2)
            count+=1
        self.init_detection = False
        self.init_tracking =False
        self.init_following =False
        self.init_visual= False
       
        self.setLayout(statuslayout)
        
    def service_calling(self, system_name, status):
        if system_name == "Object Detection System":
            if status == 1:
                #close visual tracking 
                temp1_request = EnableFunction.Request()
                temp1_request.enable = False
                future = self.cli_visual_tracking_enable.call_async(temp1_request)
                self.executor.spin_until_future_complete(future, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)

                #open object detection 
                temp_request = EnableFunction.Request()
                temp_request.enable = True
                future1 = self.cli_object_detection_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future1, timeout_sec= 3)
                future1_check_response = future1.result()
                print(future1_check_response)
                # if future1_check_response.success :
                #     self.update_command_label(f'Open Detection: True')
                # else:
                #     self.update_command_label(f'Open Detection: False \n reason: {future1_check_response.message}')
            else:
                temp_request = EnableFunction.Request()
                temp_request.enable = False
                future1 = self.cli_object_detection_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future1, timeout_sec= 3)
                future1_check_response = future1.result()
                print(future1_check_response)
                # if future1_check_response.success :
                #     self.update_command_label(f'Open Detection: True')
                # else:
                #     self.update_command_label(f'Open Detection: False \n reason: {future1_check_response.message}')
        elif system_name == "Gimbal Tracking System":
            if status == 1:
                temp_request = EnableFunction.Request()
                temp_request.enable = True
                future = self.cli_gimbal_tracking_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)
                # if future_check_response.success :
                #     self.update_command_label(f'Open Tracking: True')
                # else:
                #     self.update_command_label(f'Open Tracking: False \n reason: {future_check_response.message}')
            else:
                temp_request = EnableFunction.Request()
                temp_request.enable = False
                future = self.cli_gimbal_tracking_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)
        elif system_name == "Drone Following System":
            if status == 1:
                temp_request = EnableFunction.Request()
                temp_request.enable = True
                future = self.cli_drone_following_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)
            else:
                temp_request = EnableFunction.Request()
                temp_request.enable = False
                future = self.cli_drone_following_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)

        elif system_name == "Visual Tracking System":
            if status == 1:
                #close object detection 
                temp1_request = EnableFunction.Request()
                temp1_request.enable = False
                future = self.cli_object_detection_enable.call_async(temp1_request)
                self.executor.spin_until_future_complete(future1, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)

                #open visual tracking
                temp_request = EnableFunction.Request()
                temp_request.enable = True
                future = self.cli_visual_tracking_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)
            else:
                temp_request = EnableFunction.Request()
                temp_request.enable = False
                future = self.cli_visual_tracking_enable.call_async(temp_request)
                self.executor.spin_until_future_complete(future, timeout_sec= 3)
                future_check_response = future.result()
                print(future_check_response)

            
               



    def timer_callback(self):
        if self.init_detection:
            self.service["Object Detection System"].setStyleSheet("background-color: green")
            
            self.update_label("Object Detection System", "enabled: ", self.object_detection_response.enabled)
            self.update_label("Object Detection System", "is_intializing: ", self.object_detection_response.is_intializing)
            self.update_label("Object Detection System", "is_detecting: ", self.object_detection_response.is_detecting)
            self.update_label("Object Detection System", "detection_mode: ", self.object_detection_response.detection_mode)
           
        else: 
            self.service["Object Detection System"].setStyleSheet("background-color: red") 
            for sub_name in self.sub_service["Object Detection System"]:
                self.service["Object Detection System", sub_name].setText("False")
               

        if self.init_tracking:
            self.service["Gimbal Tracking System"].setStyleSheet("background-color: green")
            self.update_label("Gimbal Tracking System", "enabled: ", self.gimbal_tracking_response.enabled)
            self.update_label("Gimbal Tracking System", "is_tracking: ", self.gimbal_tracking_response.is_tracking)
            self.update_label("Gimbal Tracking System", "target_id: ", self.gimbal_tracking_response.target_id)
           
        else:
            self.service["Gimbal Tracking System"].setStyleSheet("background-color: red")
            for sub_name in self.sub_service["Gimbal Tracking System"]:
                self.service["Gimbal Tracking System", sub_name].setText("False")

        if self.init_following:
            self.service["Drone Following System"].setStyleSheet("background-color: green")
            self.update_label("Drone Following System", "enabled: ", self.drone_following_response.enabled)
            self.update_label("Drone Following System", "is_following: ", self.drone_following_response.is_following)
            self.update_label("Drone Following System", "target_id: ", self.drone_following_response.target_id)
            self.update_label("Drone Following System", "heading_locked: ", self.drone_following_response.heading_locked)
            self.update_label("Drone Following System", "following_mode: ", self.drone_following_response.following_mode)
        else:
            self.service["Drone Following System"].setStyleSheet("background-color: red")
            for sub_name in self.sub_service["Drone Following System"]:
                self.service["Drone Following System", sub_name].setText("False")

        if self.init_visual:
            self.service["Visual Tracking System"].setStyleSheet("background-color: green")
            self.update_label("Visual Tracking System", "enabled: ", self.visual_tracking_response.enabled)
            self.update_label("Visual Tracking System", "is_intializing: ", self.visual_tracking_response.is_intializing)
            self.update_label("Visual Tracking System", "is_tracking: ", self.visual_tracking_response.is_tracking)
            self.update_label("Visual Tracking System", "tracking_mode: ", self.visual_tracking_response.tracking_mode)
            self.update_label("Visual Tracking System", "tracking_number: ", self.visual_tracking_response.tracking_number)

        else:
            self.service["Visual Tracking System"].setStyleSheet("background-color: red")
            for sub_name in self.sub_service["Visual Tracking System"]:
                self.service["Visual Tracking System", sub_name].setText("False")
    #update the Qlabel
    def update_label(self, system_name, sub_name, msg):
        self.service[system_name, sub_name].setText(str(msg))
        

        
   

class MessageLayout(QWidget):
    def __init__(self):
        super().__init__()
        self.node= Node("ros_subscriber_fetch_message")
        # self.topic = [
        #         '/gimbal/info', '/object_detection/target_deviation', 
        #         '/avix_mavros/follow_command', '/object_detection/target_gps', '/ktg_gimbal/control',
        #         '/inf_interface/info', 'avix_mavros/state'
        #     ]

        self.topic = [
            avix_common.KTG_INFO, avix_common.OBJECT_DETECTIONS, 
            avix_common.MAVLINK_INFO, avix_common.INF_INFO, 
            avix_common.KTG_CONTROL, avix_common.MAVLINK_FOLLOW_CMD

        ]

        # Create a dictionary to store labels
        self.labels = {}
        self.sendmsg = "initialize!!!"
        self.init_ui()
    

    def init_ui(self):
        #self.setWindowTitle('Message Window')
        layout = QVBoxLayout()

        self.label = QLabel('Message TEST')
        layout.addWidget(self.label)

        # self.stacked_widget = QStackedWidget(self)
        self.tab_message =QTabWidget()
        self.tab_message.tabBarClicked.connect(self.tabBarClicked)

        # Create a label for each topic and add to stacked widget
        for topic in self.topic:
            
            label = QLabel(f'{topic} Content', self)
            self.labels[topic] = label
            self.labels[topic].setWordWrap(True)
            # self.stacked_widget.addWidget(label)
            self.tab_message.addTab(label, topic)
        self.tab_message.setStyleSheet("""
            QTabBar::tab:selected { color: blue; }
        """)
       
        # Create shortcuts for switching pages using Tab key
        self.page_count = 0
        shortcut_follow_mode = QShortcut(QKeySequence(Qt.Key_Tab), self)
        shortcut_follow_mode.activated.connect(self.change_page)

        # Add stacked widget and buttons layout to main layout
        layout.addWidget(self.tab_message)   
        self.setLayout(layout)

    def tabBarClicked(self, index):
        self.page_count=index


    def change_page(self):
    
        current_index = self.tab_message.currentIndex()
        next_index = (current_index + 1) % self.tab_message.count()
        self.tab_message.setCurrentIndex(next_index)
        self.page_count=next_index
         
    def update_label(self, topic, msg):
        if self.topic[self.page_count] == topic:
            self.labels[topic].setText(msg)


class ImageShowLayout(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()
        # # Set up a timer to call the update_frame method periodically
        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.update_frame)
        self.timer1.start(100)  # Update every 100 ms .

         # # Set up a timer to call the update_frame method periodically
        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.check_frame)
        self.timer1.start(3000)  # Update every 100 ms .
        
        
        # for judge  if the detection is open
        self.detection_init = False
        self.detections=None
        self.display_name = 'Display: ' 
        cv2.namedWindow(self.display_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(self.display_name, 640, 480)
        
        cv2.setMouseCallback(self.display_name, self.get_pixel_coordinates)
        

    def init_ui(self):
        #self.setWindowTitle('Message Window')
        layout = QVBoxLayout()
        # Create a layout and add widgets for the video play 
        self.videoTitle = QLabel("image show ")
        self.videoTitle.setAlignment(Qt.AlignCenter)
        self.videoLabel = QLabel(self)

        self.frame = np.zeros((480, 640, 3), np.uint8)
        # Convert the black image to QImage
        height, width, channel = self.frame.shape
        bytes_per_line = channel * width
        q_img = QImage(self.frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
        # Create a QLabel and set the QImage to it
        self.videoLabel.setPixmap(QPixmap.fromImage(q_img))
        
        image = QVBoxLayout()
        image.addWidget(self.videoTitle)
        image.addStretch()
        image.addWidget(self.videoLabel)
        self.frame = None

        # Create a button to start/stop the video feed
        self.controlButton = QPushButton('Stop', self)
        self.controlButton.clicked.connect(self.control_video)
        image.addWidget(self.controlButton)

        # Set up the central widget

        self.bridge = CvBridge()
        
        layout.addLayout(image)

        self.setLayout(layout)
    # message page 
      # image show       

    # TODO : find the way to solve the problem of between the detection and the image 
    def check_frame(self):
        self.detection_init = False


    def update_frame(self):
        if self.frame is not None:
            
            # Convert the frame to RGB
            frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')
            height, width = frame.shape[:2]  
            # judge if the button is on  
            if self.detection_init  and self.detections is not None:
                #frame = cv2.resize(frame, (640, 480))
                frame = self.plot_annotator(self.detections, frame)
            else:
                frame = self.plot_annotator(None, frame)
            if width != 640 or height != 480:
                # resize the image
                #print(f"Received image of dimensions ({width}, {height}), which does not match expected dimensions ({640}, {480}). Resizing image.")
                frame = cv2.resize(frame, (640, 480))
            
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)   
              
            

            # Convert the frame to QImage
            
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            q_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Set the QImage to the QLabel
            self.videoLabel.setPixmap(QPixmap.fromImage(q_image))
            
    
    def get_pixel_coordinates(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.mode == 'init':
            self.target_tl = (x, y)
            self.target_br = (x, y)
            print(x,y)
            self.mode = 'select'
       
        elif event == cv2.EVENT_LBUTTONDOWN and self.mode == 'select':
            self.target_br = (x, y)
            print(x,y)
            
            self.mode = 'init'
            message = VisualTrackingCommand()
            message.tl_x = float(self.target_tl[0])
            message.tl_y = float(self.target_tl[1])
            message.br_x = float(self.target_br[0])
            message.br_y = float(self.target_br[1])
            message.reset = False
            print(f"publish  {message}")

            self.vis_publisher.publish(message)
  

    def plot_annotator(self, detection, frame):
        # resize the image shape


        if detection is None:
            return frame
        if not self.timer1.isActive():
            return

        annotator = Annotator(
        deepcopy(frame),
            line_width,
            font_size,
            font,
            pil,  # Classify tasks default to pil=True
            #example=results[0].names
        )
       
        for t in detection:
            tlbr = t.bbox
            tid = t.id
            tcls = t.class_type    
            confidence = round(t.confidence,4)
            c,  id = int(tcls), int(tid)
            label =  ('' if id is None else f'id:{id} ') +(f"class:{c}, conf: {confidence}")
            annotator.box_label(tlbr, label, color=colors(c, True))  
            
        annotated_frame = annotator.result()
        annotated_frame = cv2.resize(annotated_frame,(640,480))
        
        return  annotated_frame

    def control_video(self):
        if self.timer1.isActive():
            self.timer1.stop()
            self.controlButton.setText('Start')
        else:
            self.timer1.start(3000)
            self.controlButton.setText('Stop')


class PIDContorllerImage(QWidget):
    def __init__(self):
        super().__init__()

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
        # PID controller

        # wait the image input first
        self.initialize =  False
        self.ID_init =0
        self.detections= None
        self.image_width = 1280
        self.image_height= 720

    
    def init_ui(self):
        #self.setWindowTitle('Message Window')
        
        layout = QVBoxLayout()



        # Create a layout and add widgets for the video play 
        self.PIDcontrollerLabel = self.canvas 
        PIDcontroller = QVBoxLayout()
        PIDcontroller.addWidget(self.PIDcontrollerLabel)
        
        
        

        self.frame = None

        # Create a button to start/stop the video feed
        self.PIDButton = QPushButton('Start', self)
        self.PIDButton.clicked.connect(self.pid_control)
        PIDcontroller.addWidget(self.PIDButton)

        layout.addLayout(PIDcontroller)

        self.setLayout(layout)
    # message page 
     
    def plot_data(self):
        self.ax.clear()
        self.ax.plot(self.t_data, self.x_data, color='r')
        self.ax.plot(self.t_data, self.y_data, color='b')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')
        plt.legend(['x', 'y'], loc='upper left')
        self.ax.set_title('Real-Time Deviation Data Plot')
        self.canvas.draw()
    def update_data(self,detections , ID):
       
        for t in detections:
            tid = t.id
            if tid == ID:
                tlbr = t.bbox
                x1,y1,x2,y2=tlbr[0],tlbr[1],tlbr[2],tlbr[3]
                cx = (x1+x2)/2
                cy = (y1+y2)/2
                dev_x= cx - self.image_width/2
                dev_y = cy - self.image_height/2
                return dev_x , dev_y
        return 0.0 , 0.0

        
    def update_plot(self):
        # Generate random data for demonstration
        new_t = len(self.t_data)
        # new_x = self.x
        # new_y = self.y
        if self.detections is None :
            return
        new_x , new_y = self.update_data(self.detections, self.ID_init)
        
        self.t_data.append(new_t)
        self.x_data.append(new_x)
        self.y_data.append(new_y)
        
        self.plot_data()

        return True
    def pid_control(self):
        if self.timer2.isActive():
            self.timer2.stop()
            self.data_reset()
            self.plot_data()
            self.PIDButton.setText('Start')
        else:
            self.timer2.start(300)
            self.PIDButton.setText('Stop')

    def data_reset(self):
        self.t_data=[]
        self.x_data = []
        self.y_data = []


def main(args=None):
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    print('Screen: %s' % screen.name())
    size = screen.size()
    print('Size: %d x %d' % (size.width(), size.height()))
    rect = screen.availableGeometry()
    print('Available: %d x %d' % (rect.width(), rect.height()))

    app_instance = mainWindow()  # Create an instance of RosSubscriberApp
    screen = app.primaryScreen()
    rect = screen.availableGeometry()
    app_instance.resize(rect.width(), rect.height())
    app_instance.show()
    
    
    
    sys.exit(app.exec_())
    
    
if __name__ == '__main__':
    main()
