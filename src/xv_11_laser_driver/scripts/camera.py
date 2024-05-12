#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import threading
raspberry_pi_ip = 'raspberrypi.local'
tcp_port = 8888
video_stream_url = f'tcp://{raspberry_pi_ip}:{tcp_port}'
def start_video_stream():
    subprocess.run(["libcamera-vid", "-t", "0", "--inline", "--listen", "-o", "tcp://0.0.0.0:8888"])
def read_video_stream():
    cap = cv2.VideoCapture(video_stream_url)
    if not cap.isOpened():
        rospy.logerr("cannot open video.")
        return
    image_pub = rospy.Publisher('camera_image', Image, queue_size=10)
    bridge = CvBridge()
    # Vòng lặp để đọc và xuất dữ liệu hình ảnh
    while not rospy.is_shutdown():
        ret, frame = cap.read()
      
        if not ret:
            rospy.logerr("Không thể đọc khung hình.")
            break

        # Chuyển đổi khung hình thành dạng ROS Image
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

        # Xuất dữ liệu hình ảnh qua topic 'camera_image'
        image_pub.publish(ros_image)
    # Giải phóng tài nguyên khi kết thúc
    cap.release()
def main():
    # Khởi chạy lệnh libcamera-vid trong một luồng riêng
    video_stream_thread = threading.Thread(target=start_video_stream)
    video_stream_thread.start()
    rospy.sleep(2)
    # Khởi tạo node ROS
    rospy.init_node('video_to_image_topic', anonymous=True)
    # Bắt đầu đọc và xuất dữ liệu hình ảnh
    read_video_stream()
    rospy.spin()
if __name__ == '__main__':
    main()
