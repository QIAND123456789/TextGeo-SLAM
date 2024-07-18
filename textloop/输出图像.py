#coding:utf-8

#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'

import roslib;  
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

path='/media/lcy/lcy_szu_private_disk/IR-D论文/数据集/M2DGR/street1/rgb图像/' #存放图片的位置
class ImageCreator():
    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('street1.bag', 'r') as bag:   #要读取的bag文件；
            time=0
            print("开始")
            a=1
            b=1
            for topic,msg,t in bag.read_messages():
                
                if topic == '/camera/color/image_raw' : 
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    except CvBridgeError as e:
                        print(e)
                    timeimg= msg.header.stamp.to_sec()#之前时间同步之后，用这个方法输出图像，bag里有1542个图像，解出来同名覆盖之后有1534，证明只有几个时间戳一样的
                    #timeimg= "{:.6f}".format(time)
                    image_name = str(timeimg)+ ".jpg"
                    print("输出第"+str(b)+"个图像数据")
                    b=b+1
                    cv2.imwrite(path+image_name, cv_image)
                    continue
        #outbag.write(topic, msg, msg.header.stamp)生成bag文件
if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
