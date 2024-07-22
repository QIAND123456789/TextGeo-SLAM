# coding:utf-8

#!/usr/bin/python

import os
import roslib
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

path = 'textloop/pictures/'  # 存放图片的位置
if not os.path.exists(path):
    os.makedirs(path)

class ImageCreator:
    def __init__(self):
        self.bridge = CvBridge()
        try:
            with rosbag.Bag('2024-07-05-15-18-03.bag', 'r') as bag:  # 要读取的bag文件
                print("开始")
                image_count = 1
                for topic, msg, t in bag.read_messages():
                    if topic.strip() == '/h1/huarui_cam_node/image_raw':  # 确保topic名称没有多余空格
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print(e)
                            continue
                        timeimg = msg.header.stamp.to_sec()  # 使用时间戳作为图像名称
                        image_name = str(timeimg) + ".jpg"
                        print(f"输出第{image_count}个图像数据")
                        image_count += 1
                        cv2.imwrite(path + image_name, cv_image)
        except Exception as e:
            print(f"读取bag文件时出错: {e}")

if __name__ == '__main__':
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
