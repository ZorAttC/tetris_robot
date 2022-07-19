
import rospy
import jaka_msgs.srv 




name=jaka_msgs.srv.fetch_drop
print(name)


if __name__ =='__main__':
    rospy.init_node('sucker_controller')
    