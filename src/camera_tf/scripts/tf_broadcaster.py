#!/usr/bin/env python  

import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #static transform (world->camera)
#         translation: 
#   x: -0.4532940462982372
#   y: 0.006566744148824074
#   z: 0.7340297813180068
# rotation: 
#   x: -0.7072054874287077
#   y: 0.7067211934449428
#   z: 0.009115163257658534
#   w: 0.01795736855070792
        br.sendTransform((-0.4532940462982372, 0.006566744148824074, 0.7340297813180068),
                         (-0.7072054874287077, 0.7067211934449428, 0.009115163257658534, 0.01795736855070792),
                         rospy.Time.now(),
                         "camera_color_frame",
                         "Link_0")
        rospy.loginfo("successfully send the transform")
        rate.sleep()