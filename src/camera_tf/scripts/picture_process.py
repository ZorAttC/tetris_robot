#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf
import math
import numpy as np
from numpy.linalg import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from numpy.linalg import *
class Pixel:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Camera_tf():
 def __init__(self):
    self._cvb=CvBridge()
    self.sub=rospy.Subscriber('/camera/color/image_raw',Image,self.callback)
    self.pub=rospy.Publisher('result',String,queue_size=1)
 def main(self):
    rospy.spin()
    
 def callback(self,image_msg):
    try:
        cv_image=self._cvb.imgmsg_to_cv2(image_msg,desired_encoding="passthrough")
        #the camera inner parameters K 
        #906.6024169921875, 0.0, 654.727783203125, 0.0, 906.7268676757812, 372.0168151855469, 0.0, 0.0, 1.0
        K=np.matrix([[906.6024169921875, 0.0, 654.727783203125],[0.0, 906.7268676757812, 372.0168151855469],[0.0, 0.0, 1.0]])
        pix = Pixel(0, 0)
        uv1=np.array([[pix.x],[pix.y],[1]])
        #Z is the distance from camera to object
        z=100
        result=z*np.dot(inv(K),uv1)

        #look up link_0 to 
        rospy.loginfo(result)
       
        
    except CvBridgeError as e:
        print(e)
    


if __name__ == '__main__':
    rospy.init_node('camera_test')
    camer=Camera_tf()
    camer.main()