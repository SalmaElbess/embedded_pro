#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from robot_rotate.msg import move_motors

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.en_rt)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.in_rt_2)

    
    #print('cmd_vel1 is' + str(data.data))
     
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
 
    rospy.Subscriber("mot_vel", move_motors, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    listener()