#!/usr/bin/env python  
import sys
sys.path.append('/home/cyh/teris_robot/devel/lib/python3/dist-packages')

import rospy
import tf
import numpy as np
from numpy.linalg import *
from geometry_msgs.msg import PointStamped
from numpy.linalg import *
from teris_msgs.srv import *
import jaka_msgs.srv



class Pixel:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def transform(x,y):
    try:
        #the camera inner parameters K 
        #906.6024169921875, 0.0, 654.727783203125, 0.0, 906.7268676757812, 372.0168151855469, 0.0, 0.0, 1.0
        #K=np.matrix([[906.6024169921875, 0.0, 654.727783203125],[0.0, 906.7268676757812, 372.0168151855469],[0.0, 0.0, 1.0]])
        #pix = Pixel(x, y)
        #uv1=np.array([[pix.x],[pix.y],[1]])
        #Z is the distance from camera to object(the unit is meter)
        #z=0.7
        #result=z*np.dot(inv(K),uv1)
        #mylst=result.tolist()
        #look up Transform
        #ls=tf.TransformListener()
        #ls.waitForTransform("Link_0","camera_color_frame",rospy.Time(),rospy.Duration(10.0))
        #point=PointStamped()
        #point.header.frame_id="camera_color_frame"
        #point.point.x=mylst[0][0]
        #point.point.y=mylst[1][0]
        #point.point.z=mylst[2][0]
        #finalrs=ls.transformPoint("Link_0",point)
        #s=real_xyz_srvResponse(finalrs.point.x, finalrs.point.y, finalrs.point.z)
        #move to the corresponsive point
      
   
        a1=-0.03421
        b1=-0.7954
        c1=-119.9
        a2=-0.7926
        b2=0.04183
        c2=482.5
        finalx=a1*x+b1*y+c1
        finaly=a2*x+b2*y+c2
        finalz=100
        s=real_xyz_srvResponse(finalx, finaly, finalz)
        rospy.loginfo(s)
        return s
        
         
    except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException) as e:
        rospy.logerr(e)
    
def handler(req):
    result=transform(req.x,req.y)
    return result

    


if __name__ == '__main__':
    rospy.init_node('camera_tf')
    s=rospy.Service('get_real_xyz',real_xyz_srv,handler)
    rospy.loginfo('im running,service is on')
    rospy.spin()
    
    
