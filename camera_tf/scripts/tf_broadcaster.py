#!/usr/bin/env python  

import rospy
import tf
import jaka_msgs.srv


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #static transform (world->camera)
#        translation: 
#  x: -0.39034117044128325
# y: 0.0020440573586200106
 # z: 0.6893540776140117
#rotation: 
 # x: 0.7467675586271404
 # y: -0.626779689834514
  #z: -0.04319727386677629
  #w: 0.21821876482898792
        #br.sendTransform((-0.47487990120962564,0.023059983839086362, 0.7018997292562712),
         #                (0.6891192288252586, -0.7237846459381347, -0.017663639117351984, 0.030634467852760243),
          #               rospy.Time.now(),
           #              "camera_color_frame",
            #             "Link_0")
        #br.sendTransform((-0.499,-0.0462, 0.707),
         #                (-0.682,0.724, -0.0139, 0.097),
          #               rospy.Time.now(),
           #              "camera_color_frame",
            #             "Link_0")
        br.sendTransform((-0.499,-0.0462, 0.707),
                         (-0.707,0.707, 0, 0),
                         rospy.Time.now(),
                         "camera_color_frame",
                         "Link_0")
        rospy.loginfo("successfully send the transform")
        rate.sleep()
