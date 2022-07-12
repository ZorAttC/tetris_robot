
import jaka_msgs.srv
import rospy
import sys



if __name__=="__main__":
 rospy.init_node("movement_contrl")
 rospy.wait_for_service("/jaka_driver/linear_move")
 move=rospy.ServiceProxy("/jaka_driver/linear_move",jaka_msgs.srv.Move)
 result=move(pose=[-325,-207,186,0.93,-2.98,0.0096],mvvelo=30.0,mvacc=30.0)
 rospy.loginfo(result)
 rospy.spin()
 