import rospy
import numpy as np
import os
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud

'''
Real time semantic map generator engine, including publisher and subscriper port for ros.
Haven't tested yet.
'''
class map_engine:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._sub3 = rospy.Subscriber('categorymap', Image, self.callback, queue_size = 1000)
        self._sub1 = rospy.Subscriber('KeyPoints', PointCloud, self.callback1, queue_size = 1000)
        self._sub2 = rospy.Subscriber('MapPoints', PointCloud, self.callback2, queue_size = 1000)
        self._pub = rospy.Publisher('Semantic_Map', PointCloud, queue_size=1)
        self._currentframe = Image()
        self._mp = PointCloud()
        self._kp = PointCloud()
        self.smp = PointCloud()


    def callback(self, image_msg):
        self._currentframe = image_msg

    def callback1(self, pointcloud_msg):
        self._kp = pointcloud_msg
        for i in range(self._kp):
            if self._currentframe[i.points.x,i.points.y] == 11:
                self.smp.points.append(i)
            self.smp.header = self._mp.header
        self._pub.publish(self.smp)
    
    def callback2(self, pointcloud_msg):
        self._mp = pointcloud_msg
    
    def main(self):
        rospy.spin()
    
