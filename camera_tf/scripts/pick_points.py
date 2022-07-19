#!/usr/bin/env python 
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped

#图片路径
img = cv2.imread('/home/cyh/jaka_robot/jaka_robot/src/cv_process/scripts/teris.png')
a =[]
cnt=0
def on_EVENT_LBUTTONDOWN(event, x, y,flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:

        xy = "%d,%d" % (x, y)
        a.append((x,y))
        cv2.circle(img, (x, y), 4, (0, 0, 255), thickness=1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    3.0, (125, 255, 255), thickness=2)
        cv2.imshow("image", img)
    global cnt
    if event == cv2.EVENT_RBUTTONDOWN:
     cnt=10
   

        
def callback(res):
	global cnt
	if cnt!=0:
	 cnt-=1
	 if cnt==1:
	  x=round(res.twist.linear.x,2)
	  y=round(res.twist.linear.y,2)
	  print(a[-1][0],a[-1][1],x,y)#pixel(x,y) real(x,y)
         
if __name__ == '__main__':
	rospy.init_node('pick_points')
	sub=rospy.Subscriber('/jaka_driver/tool_position',TwistStamped,callback,queue_size=5)
	cv2.namedWindow("image")
	cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
	cv2.imshow("image", img)
	cv2.waitKey(0)
	rospy.spin()
