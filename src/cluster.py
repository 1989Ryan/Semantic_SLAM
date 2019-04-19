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
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud, ChannelFloat32
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

'''
real time clustering.
'''
class cluster:
    def __init__(self):
        self._sub = rospy.Subscriber('/Semantic_Map', PointCloud, self.callback, queue_size=1)
        self._sub2 = rospy.Subscriber('/trajectory', Path,  self.callback2, queue_size=1)
        self._pub = rospy.Publisher('/location', Marker, queue_size=1000)
        self._smp = [[],[]]
        self._trj = Path()
        self._model = DBSCAN(eps = 0.035, min_samples=35)
        self.marker_scale = 0.2
        self.marker_lifetime = 3 # 0 is forever
        self.marker_id = 0
        self.marker_ns = 'building'+str(self.marker_id) 
        self.marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        #self.marker_init()
    
    def marker_init(self, markers):
        self.marker_ns = 'building'+str(self.marker_id) 
        markers.ns = self.marker_ns
        markers.id = self.marker_id
        markers.type = Marker.CUBE
        markers.action = Marker.ADD
        markers.lifetime = rospy.Duration(self.marker_lifetime)
        markers.scale.x = self.marker_scale
        markers.scale.y = self.marker_scale
        markers.scale.z = self.marker_scale
        markers.color.r = self.marker_color['r']
        markers.color.g = self.marker_color['g']
        markers.color.b = self.marker_color['b']
        markers.color.a = self.marker_color['a']
        markers.header.frame_id = "sensor_frame"
        markers.header.stamp = rospy.Time.now()
        markers.frame_locked = True
        #markers.points = list()
        return markers
        

    def callback(self, pcmsg):
        self._smp = [[],[]]
        for i in range(np.size(pcmsg.channels[0].values)):
            if pcmsg.channels[0].values[i] == 200:
                self._smp[0].append(pcmsg.points[i].x)
                self._smp[1].append(pcmsg.points[i].z)
        data = np.array(self._smp)
        data = data.T
        Data = data
        data = StandardScaler().fit_transform(data)
        print(np.shape(data))
        self._model.fit(data)
        y_hat = self._model.labels_
        core_indices = np.zeros_like(y_hat, dtype=bool)
        core_indices[self._model.core_sample_indices_] = True
        y_unique = np.unique(y_hat)
        n_clusters = y_unique.size - (1 if -1 in y_hat else 0)## y_hat=-1为聚类后的噪声类
        rospy.loginfo('cluster number：'+ str(n_clusters))
        #TODO Real time visualization, including trajectorys
        k = 0
        for i in y_unique:
            self.marker_id = k
            k += 1
            marker = Marker()
            marker = self.marker_init(marker)
            if i == -1:
                continue
            marker.pose.position.x = np.mean(Data[(y_hat == i), 0])
            marker.pose.position.z = np.mean(Data[(y_hat == i), 1])
            marker.pose.position.y = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            self._pub.publish(marker)
        rospy.sleep(1)
            
    
    def callback2(self, tjymsg):
        self._trj = tjymsg

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cluster')
    cluster = cluster()
    cluster.main()
