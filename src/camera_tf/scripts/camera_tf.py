#!/usr/bin/env python  
import sys
from typing import final
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
        K=np.matrix([[906.6024169921875, 0.0, 654.727783203125],[0.0, 906.7268676757812, 372.0168151855469],[0.0, 0.0, 1.0]])
        pix = Pixel(x, y)
        uv1=np.array([[pix.x],[pix.y],[1]])
        #Z is the distance from camera to object(the unit is meter)
        z=0.7
        result=z*np.dot(inv(K),uv1)
        mylst=result.tolist()
        #look up Transform
        ls=tf.TransformListener()
        ls.waitForTransform("Link_0","camera_color_frame",rospy.Time(),rospy.Duration(10.0))
        point=PointStamped()
        point.header.frame_id="camera_color_frame"
        point.point.x=mylst[0][0]
        point.point.y=mylst[1][0]
        point.point.z=mylst[2][0]
        finalrs=ls.transformPoint("Link_0",point)
        s=real_xyz_srvResponse(finalrs.point.x, finalrs.point.y, finalrs.point.z)
        #move to the corresponsive point
        rospy.wait_for_service("/jaka_driver/linear_move")
        move=rospy.ServiceProxy("/jaka_driver/linear_move",jaka_msgs.srv.Move)
        result=move(pose=[finalrs.point.x*1000,finalrs.point.y*1000,186,0.93,-2.98,0.0096],mvvelo=30.0,mvacc=30.0)
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
    rospy.spin()
    
    