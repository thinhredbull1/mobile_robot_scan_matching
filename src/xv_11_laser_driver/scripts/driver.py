#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from actionlib_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import numpy as np
import serial

class TwistSerialBridge:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('twist_serial_bridge')

        # Khởi tạo cổng serial
        self.serial_port_name = rospy.get_param('port', '/dev/robot')
        self.baud = rospy.get_param('baud', 57600)
        
        self.serial_port = serial.Serial( self.serial_port_name,  self.baud)  # Thay đổi '/dev/ttyUSB0' thành cổng serial thực tế
        rospy.loginfo("Serial port opened")
        self.imu_pub=rospy.Publisher('/imu/data', Imu, queue_size=10)
        # Đăng ký subscriber vào topic '/cmd_vel'
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/speak', Bool, self.speak_callback)
        # rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # Khai báo publisher cho topic /odom
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.last_state=0
        self.goal_set=False
        self.publish_speak=False
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.move_base_goal_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        self.odom_transform = None
        self.base_link_transform = None
        self.publish_odom_flag=0
        self.odom_msg = Odometry()
        self.speak_pub=rospy.Publisher('/speak', Bool, queue_size=10)
        self.last_yaw=0
        self.last_x=0
        self.vx=0
        self.vz=0
        self.dt=rospy.Time.now()
    def quaternion_multiply(self,q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([-x2*x1 - y2*y1 - z2*z1 + w2*w1,
                        x2*w1 + y2*z1 - z2*y1 + w2*x1,
                        -x2*z1 + y2*w1 + z2*x1 + w2*y1,
                        x2*y1 - y2*x1 + z2*w1 + w2*z1])
    def publish_odom(self,pose_):   
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.position = pose_.transform.translation
        base_link_in_odom_x=pose_.transform.translation.x
        dt_now=(rospy.Time.now()-self.dt).to_sec()
        self.dt=rospy.Time.now()

        self.odom_msg.twist.twist.linear.x=(-(base_link_in_odom_x-self.last_x))/10.0
        
        self.last_x=base_link_in_odom_x
        # Tính toán quaternion từ rotation
        
        
        self.odom_msg.pose.pose.orientation=pose_.transform.rotation
        
        (roll, pitch, yaw) = euler_from_quaternion([self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w])    
        d_theta=yaw-self.last_yaw
        pi=3.14159265359
        if(d_theta<-5.5):
            d_theta=(yaw+pi)+(pi-self.last_yaw) # last_yaw = 3.14 yaw = -3
        elif(d_theta>5.5): # yaw= 3 last_yaw =-3
            d_theta=(yaw-pi)-(pi+self.last_yaw)
        self.odom_msg.twist.twist.angular.z =d_theta/10.0
        self.last_yaw=yaw
        if(self.vx==0 and self.vz==0):
        
            self.odom_msg.pose.covariance[0] = 1e-9
            self.odom_msg.pose.covariance[7] = 1e-3
            self.odom_msg.pose.covariance[8] = 1e-9
            self.odom_msg.pose.covariance[14] = 1e6
            self.odom_msg.pose.covariance[21] = 1e6
            self.odom_msg.pose.covariance[28] = 1e6
            self.odom_msg.pose.covariance[35] = 1e-9
            self.odom_msg.twist.covariance[0] = 1e-9
            self.odom_msg.twist.covariance[7] = 1e-3
            self.odom_msg.twist.covariance[8] = 1e-9
            self.odom_msg.twist.covariance[14] = 1e6
            self.odom_msg.twist.covariance[21] = 1e6
            self.odom_msg.twist.covariance[28] = 1e6
            self.odom_msg.twist.covariance[35] = 1e-9
        
        else:
            self.odom_msg.pose.covariance[0] = 1e-3
            self.odom_msg.pose.covariance[7] = 1e-3
            self.odom_msg.pose.covariance[8] = 0.0
            self.odom_msg.pose.covariance[14] = 1e6
            self.odom_msg.pose.covariance[21] = 1e6
            self.odom_msg.pose.covariance[28] = 1e6
            self.odom_msg.pose.covariance[35] = 1e3
            self.odom_msg.twist.covariance[0] = 1e-3
            self.odom_msg.twist.covariance[7] = 1e-3
            self.odom_msg.twist.covariance[8] = 0.0
            self.odom_msg.twist.covariance[14] = 1e6
            self.odom_msg.twist.covariance[21] = 1e6
            self.odom_msg.twist.covariance[28] = 1e6
            self.odom_msg.twist.covariance[35] = 1e3
            
        self.odom_pub.publish(self.odom_msg)
        self.publish_odom_flag=1
            
    def move_base_goal_callback(self,goal_msg):
        rospy.loginfo("Goal is set!")
        self.goal_set = True
        self.speak_pub.publish(1)
        self.publish_speak=True
    def move_base_status_callback(self,status_msg):
        if self.goal_set and len(status_msg.status_list) > 0:  # Kiểm tra xem goal đã đạt được hay chưa
            if status_msg.status_list[-1].status == 3:
                rospy.loginfo("Goal reached!")
                self.speak_pub.publish(1)
                self.publish_speak=True
                # Đặt lại biến cờ để không publish nữa khi goal mới được cài đặt
                self.goal_set = False
    def speak_callback(self,msg):
        if msg.data == True and self.last_state==0:
            print("Speak command received: True")
            self.last_state=1
            serial_data_2 = "1s;"
            self.serial_port.write(serial_data_2.encode())
            # Thực hiện các hành động khi nhận được True từ topic /speak
        elif msg.data==False and self.last_state==1:
            serial_data_2 = "0s;"
            self.last_state=0
            print("Speak command received: False")
            # Thực hiện các hành động khi nhận được False từ topic /speak
            self.serial_port.write(serial_data_2.encode())
    def cmd_vel_callback(self, msg):
        # Lấy thông tin vận tốc tuyến tính và vận tốc góc từ message
        self.vx = msg.linear.x
        self.vz = msg.angular.z
        min_x=0.1
        if(self.vx<=min_x and self.vx>0):
            self.vx=min_x
        elif(self.vx>=-min_x and self.vx<0):
            self.vx=-min_x
        serial_data = "{}/{};".format(self.vx, self.vz)
        # Gửi dữ liệu xuống serial
        self.serial_port.write(serial_data.encode())
        rospy.loginfo("Serial data sent: {}".format(serial_data.strip()))

    def run(self):
        # Chạy vòng lặp ROS
        speak_delay=1.4
        rate_hz=5
        rate=rospy.Rate(rate_hz)
        imu_msg = Imu()
        count_delay=rate_hz*speak_delay
   
        while not rospy.is_shutdown():
          
            try:
        # Chuyển đổi tọa độ từ 'odom' sang 'base_link'
                pose_base_link = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.1))
                # rospy.loginfo("Base link transform: %s", pose_base_link)
                # pose_odom.header.stamp = rospy.Time.now() 
                self.publish_odom(pose_base_link)
                # rospy.loginfo("Base link pose: %s", pose_base_link.pose)
            except tf2_ros.LookupException:
                rospy.logwarn_throttle(1.0, "LookupException: Transform not available yet, retrying...")
            except tf2_ros.ExtrapolationException:
                rospy.logwarn_throttle(1.0, "ExtrapolationException: Transform not available yet, retrying...")
            if self.publish_speak:
                count_delay-=1
                if(count_delay<=0):
                    count_delay=rate_hz*speak_delay
                    self.speak_pub.publish(0)
            # if self.serial_port.in_waiting > 0:
            #     data=self.serial_port.readline().decode().strip()
            #     if ',' in data:
            #         try:
            #             value1, value2,value3,value4= [float(x) for x in data.split(',')]
                        
            #             imu_msg.header = Header()
            #             imu_msg.header.stamp = rospy.Time.now()
            #             imu_msg.header.frame_id = "imu_link"

            #             # Đặt giá trị cho quaternion (orientation)
            #             quaternion = Quaternion()
            #             quaternion.x = value1
            #             quaternion.y = value2
            #             quaternion.z = value3
            #             quaternion.w = value4
            #             imu_msg.orientation = quaternion

            #             # Đặt giá trị cho linear acceleration
            #             # imu_msg.linear_acceleration.x = 0.0
            #             # imu_msg.linear_acceleration.y = 0.0
            #             # imu_msg.linear_acceleration.z = 9.81  # Giả sử gia tốc z = 9.81 m/s^2 (gia tốc trọng trường)
            #             # # Đặt giá trị cho angular velocity
            #             # imu_msg.angular_velocity.x = 0.0
            #             # imu_msg.angular_velocity.y = 0.0
            #             # imu_msg.angular_velocity.z = 0.0
            #             # Xuất bản message Imu
            # #             self.imu_pub.publish(imu_msg)
            #         except Exception as e:
            #             print(e)
          
              
              
            rate.sleep()

if __name__ == '__main__':
    try:
        twist_serial_bridge = TwistSerialBridge()
        twist_serial_bridge.run()
    except rospy.ROSInterruptException:
        pass
