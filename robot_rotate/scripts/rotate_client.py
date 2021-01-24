#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from robot_rotate.srv import *
 
def rotate_client(x):
    rospy.wait_for_service('rotate_robot')
    try:
        rotate = rospy.ServiceProxy('rotate_robot', Rotate)
        resp1 = rotate(x)
        return resp1#.rotated
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
 
def usage():
    return "%s [x y]"%sys.argv[0]
 
if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s"%(x))
    print("%s = %s"%(x, rotate_client(x)))