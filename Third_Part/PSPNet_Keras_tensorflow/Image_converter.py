import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

class Image_converter:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size = 1)
        self._pub = rospy.Publisher('encode_image', String, queue_size = 1)

    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_data = cv2.imencode('.jpg',cv_image)[1].tostring()
        self._pub.publish(image_data)
    
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Image_converter')
    converter = Image_converter()
    converter.main()