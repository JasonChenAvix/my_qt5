import sys
import rclpy
from PyQt5.QtWidgets import QGridLayout, QFrame,QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit ,QCheckBox ,QHBoxLayout,QMainWindow , QStackedWidget, QSizePolicy, QShortcut
from PyQt5.QtGui import QKeySequence ,QIcon
from PyQt5.QtCore import Qt, QTimer
from rclpy import init, shutdown
from rclpy.node import Node
from std_msgs.msg import String ,Bool , Int32 
from sensor_msgs.msg import Image
from avix_utils.msg import MQ3State, GimbalInfo ,TrackingUpdate , InfInfo ,GimbalControl, MavlinkInfo, ObjectDetections,  FollowCommand, TargetGPS 
from avix_utils import avix_common
from avix_utils.srv import EnableFunction,  ObjectDetectionStatus, DroneFollowingStatus, GimbalTrackingStatus
from avix_utils.avix_error_codes import get_error_message
from avix_utils.avix_enums import ErrorMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup ,ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


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




class FollowWindow(QWidget):
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
        
        #  create image show 
        #TODO: make two subscriber into one 
        self.image_subscriber = self.node.create_subscription(Image, avix_common.KTG_EO_IMG, self.image_callback, 10)
        self.image_subscriber = self.node.create_subscription(Image,avix_common.KTG_EO_IMG , self.video_callback, 10)

        #create the publisher 
        #self.follow_publisher = self.node.create_publisher(Bool, '/mq3/start_following', 10)
        #self.track_start_publisher = self.node.create_publisher(Bool, '/icp_interface/tracking_cmd', 10)
        #self.id_publisher = self.node.create_publisher(Int32, '/icp_interface/following_cmd', 10)
        

        self.mq3_status_publisher = self.node.create_publisher(MQ3State, avix_common.MQ3_STATUS, 10)
        self.id_publisher = self.node.create_publisher(Int32, avix_common.ICP_TARGET_ID_CMD, 10)
        self.control_publisher = self.node.create_publisher(GimbalControl, avix_common.KTG_CONTROL, 10)

        ##change to client
        self.cli_object_detection_enable = self.node.create_client(EnableFunction, avix_common.MQ3_ENABLE_OBJECT_DETECTION_SRV)
        self.cli_gimbal_tracking_enable = self.node.create_client(EnableFunction, avix_common.MQ3_ENABLE_GIMBAL_TRACKING_SRV)
        self.cli_drone_following_enable = self.node.create_client(EnableFunction, avix_common.MQ3_ENABLE_DRONE_FOLLOWING_SRV)

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
        #self.wait_for_services()
        # create the mutli thread executor
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        print('All service are ready. MQ3 service success')

    def wait_for_services(self):
        while not self.cli_object_detection_enable.wait_for_service(timeout_sec=3.0):
            print('[Object Detection Enabling] Service not available, waiting again... Please boot it up if have not!')
            #time.sleep(0.5)
        
        while not self.cli_drone_following_enable.wait_for_service(timeout_sec=3.0):
            print('[Drone Following Enabling] Service not available, waiting again... Please boot it up if have not!')
            #time.sleep(0.5)

        while not self.cli_gimbal_tracking_enable.wait_for_service(timeout_sec=3.0):
            print('[Gimbal Tracking Enabling] Service not available, waiting again... Please boot it up if have not!')
            #time.sleep(0.5)

    #UI interface layout
    def init_ui(self):
        self.setWindowTitle('AVIX Detection System V1.0.1')
        self.setGeometry(100, 100, 600, 600)
        
        layout=QHBoxLayout()
        
        # command layout 
        ## fetch message button
        commandlayout=QVBoxLayout()
        self.fetch_button = QPushButton('Start detection system')
        self.fetch_button.setFixedSize(170, 50)
        commandlayout.addWidget(self.fetch_button)
        self.fetch_button.clicked.connect(self.fetch_message)
        
        ## Drone following enable button
        self.following = QPushButton('Drone following start')
        self.following.setFixedSize(170, 50)
        commandlayout.addWidget(self.following)
        self.following.clicked.connect(self.following_start)
        self.follow_init = False
        
        ## Tracking enable button
        self.track = QPushButton('Tracking start')
        self.track.setFixedSize(170, 50)
        commandlayout.addWidget(self.track)
        self.track.clicked.connect(self.track_start)
        self.track_init = False
        
         ## Publish ID button
        input_widget= QFrame()
        input_widget.setFrameStyle(QFrame.Box)
        input_widget.setFixedWidth(170)
        input_layout=QVBoxLayout()
        input_label= QLabel('Input the tracking id:')
        input_layout.addWidget(input_label)
        self.id_input = QLineEdit()
        self.id_input.setFixedSize(150, 30)
        input_layout.addWidget(self.id_input)
        self.id_button = QPushButton('Publish ID')
        self.id_button.setFixedSize(150, 50)
        input_layout.addWidget(self.id_button)
        self.id_button.clicked.connect(self.publish_id)
        input_widget.setLayout(input_layout)
        commandlayout.addWidget(input_widget)

        ## Record data button
        self.record_button = QPushButton('Record data')
        self.record_button.setFixedSize(170, 50)
        commandlayout.addWidget(self.record_button)
        self.record_button.clicked.connect(self.record_start)
        self.record_init = True
        
        ## Command Status Box
        # 创建文本框
        self.textbox = QTextEdit(self)
        self.textbox.setFixedSize(170, 300)
        self.textbox.setReadOnly(True)  # 设置文本框为只读
        self.textbox.setPlaceholderText('Waiting for command...')
        commandlayout.addWidget(self.textbox)


        commandlayout.addStretch()
        # Create a grid layout for the arrow buttons
        grid = QGridLayout()

        # Button and their positions in the grid
        path = '/home/nvidia/avix/arrow_icon/'
        buttons = {
            (0, 1): ("up", path+"up.png"),
            (1, 0): ("left", path+"left.png"),
            (1, 1): ("center", path+"center.png"),
            (1, 2): ("right", path+"right.png"),
            (2, 1): ("down", path+"down.png"),
            (0, 0): ("left-up", path+"left_up.png"),
            (0, 2): ("right-up", path+"right_up.png"),
            (2, 0): ("left-down", path+"left_down.png"),
            (2, 2): ("right-down", path+"right_down.png")
        }

        # Create buttons and place them in the grid
        for position, (name, icon) in buttons.items():
            button = QPushButton()
            button.setIcon(QIcon(icon))
            button.setIconSize(button.sizeHint())
            button.clicked.connect(lambda _, name=name: self.control_clicked(name))
            grid.addWidget(button, *position)

        commandlayout.addLayout(grid)
        self.set_center=False

        #Content Layout
        contentlayout = QVBoxLayout()

        # Messge windows
        self.message =MessageLayout()
        contentlayout.addWidget(self.message)

        allimage=QHBoxLayout()
        
        # Image show windows
        self.image = ImageShowLayout()

        # PID deviation image 
        self.PIDcontroller=PIDContorllerImage()

        allimage.addWidget(self.image)
        allimage.addWidget(self.PIDcontroller)
        contentlayout.addLayout(allimage)
        layout.addLayout(commandlayout)
        layout.addLayout(contentlayout)
        self.setLayout(layout)

    def timer_callback(self):
        #rclpy.spin_once(self.node, timeout_sec=0.1)
        self.executor.spin_once( timeout_sec=0.1 )

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
            pass # do nothing
            #TODO: move to center
            self.set_center=True
           
    def fetch_message(self):
        if self.timer.isActive():
            self.timer.stop()
            self.fetch_button.setText('Start to fetch data')
            self.update_command_label("Stop fetching ")
        else:
            self.timer.start(100)
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
            self.update_command_label(f'Following: True')
            print(f'Following: True') 
            self.follow_init = True
        else:
            temp_request = EnableFunction.Request()
            temp_request.enable = False
            future = self.cli_drone_following_enable.call_async(temp_request)
            self.update_command_label(f'Following: False')
            print(f'Following: False')
            self.follow_init = False
     
    def track_start(self):
        # msg = Bool()
        # msg.data = True
        # self.track_start_publisher.publish(msg)
        # self.mq3_status_publisher.publish(MQ3State(tracking_enabled=True, detection_enabled=True))
        if  not self.cli_object_detection_enable.wait_for_service(timeout_sec=1.0):
            return

        if not self.track_init:
            temp_request = EnableFunction.Request()
            temp_request.enable = True
            future = self.cli_gimbal_tracking_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future)
            future_check_response = future.result()
            print(future_check_response)

            future1 = self.cli_object_detection_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future1)
            future1_check_response = future1.result()
            print(future1_check_response)
            self.update_command_label(f'Tracking: True')
            print(f'Tracking: True') 
            self.track_init = True

        else:
            temp_request = EnableFunction.Request()
            temp_request.enable = False
            future = self.cli_gimbal_tracking_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future)
            future_check_response = future.result()
            print(future_check_response)
            future1= self.cli_object_detection_enable.call_async(temp_request)
            self.executor.spin_until_future_complete(future1)
            future1_check_response = future1.result()
            print(future1_check_response)
            self.update_command_label(f'Tracking: False')
            print(f'Tracking: False')
            self.track_init = False
            self.image.detection_init=False

    def publish_id(self):
        try:
            id_info = int(self.id_input.text())

            msg = Int32()
            msg.data = id_info %128
            self.id_publisher.publish(msg)
            self.update_command_label(f'Published ID: {id_info}')
            print(f'Published ID: {id_info}')    
        except:
            pass   

    # ros callback function
    def video_callback(self, msg):
        if self.record_init: 
            return
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting ROS Image to OpenCV: {e}')
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

    def image_callback(self, msg):
        self.image.frame = msg

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
        if self.message.topic[self.message.page_count] == avix_common.OBJECT_DETECTIONS:
            self.sendmsg = (f'''
                detections: {msg.detections}
                num_detections: {msg.num_detections}
            ''')
            self.message.update_label(avix_common.OBJECT_DETECTIONS,  self.sendmsg) 
    
    def gimbal_callback(self, msg):
        # save for the control gimbal to center
        if self.set_center:
            self.pitch_angle = msg.pitch_angle
            self.yaw_angle = msg.yaw_angle
            self.roll_angle = msg.roll_angle
            control_msg = GimbalControl()
            if abs(msg.pitch_angle) > 5 :
                if msg.pitch_angle > 0:
                    control_msg.tilt_velocity = float(-2000.0)
                else:
                    control_msg.tilt_velocity = float(2000.0)
            if abs(msg.yaw_angle) > 5:
                if msg.yaw_angle > 0:
                    control_msg.pan_velocity = float(-2000.0)
                else:
                    control_msg.pan_velocity = float(2000.0)
            else:
                self.set_center = False

            self.control_publisher.publish(control_msg)
        
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
        self.setLayout(layout)

    
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
        
         
    def update_label(self, topic, msg):
        if self.topic[self.page_count] == topic:
            self.labels[topic].setText(msg)


class ImageShowLayout(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()
        # Set up a timer to call the update_frame method periodically
        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.update_frame)
        self.timer1.start(500)  # Update every 500 ms (about 500 FPS)
        
        # for judge  if the detection is open
        self.detection_init = False
        self.detections=None

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

    def update_frame(self):
        if self.frame is not None:
            # Convert the frame to RGB
            frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')
            height, width = frame.shape[:2]
            if self.detection_init:
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

    def plot_annotator(self, detection, frame):
        if detection is None:
            return frame
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
            confidence = t.confidence
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
            self.timer1.start(500)
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
            self.plot_data()
            self.PIDButton.setText('Start')
        else:
            self.timer2.start(500)
            self.PIDButton.setText('Stop')

    def data_reset(self):
        self.t_data=[]
        self.x_data = []
        self.y_data = []


def main(args=None):
    app = QApplication(sys.argv)
    app_instance = FollowWindow()  # Create an instance of RosSubscriberApp
    app_instance.show()
    sys.exit(app.exec_())
    
    
if __name__ == '__main__':
    main()
