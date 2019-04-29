# coding: utf-8
'''
This file is part of Semantic SLAM
License: MIT
Author: Zirui Zhao
Email: ryan_zzr@outlook.com
Web: https://1989Ryan.github.io/
'''
import rospy
import numpy as np
import os
import sys
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud, ChannelFloat32
from map_generator.msg import mp
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

'''
Real time semantic map generator engine, including publisher and subscriper port for ros.
'''

class map_engine:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._sub = rospy.Subscriber('/map', mp, self.callback, queue_size = 1000)
        self._pub = rospy.Publisher('Semantic_Map', PointCloud, queue_size=1)
        self._currentframe = Image()
        self._framequeue = []
        self._kp = PointCloud()
        self.smp = PointCloud()
        self._mpqueue = []
        self._kpqueue = []
        self._labelqueue = []
        self.smp.channels = [ChannelFloat32(),ChannelFloat32()]
        self.smp.channels[0].name = 'category'
        self.smp.channels[1].name = 'rgb'
        self.temp = 0
    
    def process(self, queue, ele):
        if np.size(queue)>60:
            queue.remove(queue[0])
        queue.append(ele)
        return queue
    
    def update(self, queue, newele, id):
        if id in self._mp.channels[0].values:
            index = self._mp.channels[0].values.index(id)
            newele = np.array(newele, dtype = float)
            if np.size(queue[index]) == 1:
                queue[index] = newele
            else:
                queue[index] = queue[index]*newele / np.sum(queue[index]*newele)
        else:
            return
        

    def callback(self, pointcloud_msg):
        self._kp = pointcloud_msg.kpt
        self._mp = pointcloud_msg.mpt
        self._currentframe = pointcloud_msg.currentframe
        self._kpqueue = self.process(self._kpqueue, self._kp)
        self._framequeue = self.process(self._framequeue, self._currentframe)
        if np.size(self._framequeue)<=30:
            return
        self.smp.channels[0].values += [0]*(np.size(self._mp.channels[0].values)-np.size(self.smp.channels[0].values))
        self.smp.channels[1].values += [0]*(np.size(self._mp.channels[0].values)-np.size(self.smp.channels[1].values))
        self._labelqueue += [0]*(np.size(self._mp.channels[0].values)-np.size(self._labelqueue))
        self.smp.points = self._mp.points
        cv_image = self._cv_bridge.imgmsg_to_cv2(self._framequeue[-29])
        cv_image = np.array(cv_image/255, dtype = float)
        m = 0
        for i in self._kpqueue[-29].points:
            cate = cv_image[int(i.y),int(i.x)]
            self.update(self._labelqueue, cate, i.z)
            if i.z in  self._mp.channels[0].values:
                ii = self._mp.channels[0].values.index(i.z)
                self.smp.channels[0].values[ii] = 100 * np.argmax(self._labelqueue[ii])
                self.smp.channels[1].values[ii] = self._kpqueue[-29].channels[0].values[m]
            m+=1
        cv_image = self._cv_bridge.imgmsg_to_cv2(self._framequeue[1])
        m = 0
        for i in self._kpqueue[1].points:
            cate = cv_image[int(i.y),int(i.x)]
            self.update(self._labelqueue, cate, i.z)
            if i.z in  self._mp.channels[0].values:
                ii = self._mp.channels[0].values.index(i.z)
                self.smp.channels[0].values[ii] = 100*np.argmax(self._labelqueue[ii])
                self.smp.channels[1].values[ii] = self._kpqueue[1].channels[0].values[m]
            m+=1
        self.smp.header = self._mp.header
        self._pub.publish(self.smp)
        rospy.loginfo("semantic map published")
        
    
    def sorting(self, msg, value):
        return (msg.index(value))
    
    def main(self):
        rospy.spin()
        def myhook():
            f = open("semantic_map.txt", "w")
            for i in range(np.size(self.smp.points)):
                f.write("%f " %self.smp.points[i].x+"%f " %self.smp.points[i].y+"%f " %self.smp.points[i].z+" %d" %self.smp.channels[0].values[i]+"\n")
            f.close()
        rospy.on_shutdown(myhook)
        
        
    
if __name__ == '__main__':
    rospy.init_node('map_engine')
    engine = map_engine()
    engine.main()
    
