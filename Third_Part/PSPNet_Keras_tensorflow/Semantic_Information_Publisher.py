'''
This file is part of Semantic SLAM
License: MIT
Author: Zirui Zhao
Email: ryan_zzr@outlook.com
Web: https://1989Ryan.github.io/
'''

from pspnet import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from map_generator.msg import frame
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf
import rospy
import os
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

'''
Real time image segmentation, using PSPNet101. Dummy publisher which is Really Really Slow.
'''

class Semantic_Imformation_Publisher():
    '''
    To publish the semantic infomation which contains the categories of each pixel.
    '''
    def __init__(self):
        '''
        node initialization
        '''
        self._cv_bridge = CvBridge()
        #self._session = tf.Session()
        self.pspnet = PSPNet101(nb_classes=19, input_shape=(713, 713),
                                       weights='pspnet101_cityscapes')
        #init = tf.global_variables_initializer()
        #self._session.run(init)
        self.graph = tf.get_default_graph()
        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size = 1000)
        self._pub = rospy.Publisher('/result', frame, queue_size = 1)
    
    def callback(self, image_msg):
        '''call back funcion, which will send the image and category of each pixel'''
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
        h_ori, w_ori = cv_image.shape[:2]
        with self.graph.as_default():
            probs = self.pspnet.model.predict(self.img_proc(cv_image))[0]
        if cv_image.shape[0:1] != (713,713):  # upscale prediction if necessary
            h, w = probs.shape[:2]
            probs = ndimage.zoom(probs, (1. * h_ori / h, 1. * w_ori / w, 1.),
                                    order=1, prefilter=False)
        rospy.loginfo("running")
        cm = np.argmax(probs, axis=2).astype(np.uint8)
        #print(probs)
        #print(cm)
        category = self._cv_bridge.cv2_to_imgmsg(cm)
        probs = self._cv_bridge.cv2_to_imgmsg(probs)
        f = frame()
        f.image = image_msg
        f.category = category
        self._pub.publish(f)

    def img_proc(self, img):
        # Preprocess the image
        img = misc.imresize(img, (713,713))
        img = img - DATA_MEAN
        img = img[:, :, ::-1]  # RGB => BGR
        img = img.astype('float32')
        data = np.expand_dims(img, 0)
        return data

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Semantic_Information_Publisher')
    tensor = Semantic_Imformation_Publisher()
    tensor.main()

