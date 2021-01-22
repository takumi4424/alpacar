#!/usr/bin/python3

import rospy, sys
from geometry_msgs.msg import Twist

def cmdVelCallback(msg: Twist):
    rospy.loginfo("HOGEHOGE %s" % msg)

if __name__ == '__main__':
    rospy.init_node('alpacar_driver_node', sys.argv)
    rospy.Subscriber('/cmd_vel', Twist, cmdVelCallback)

    rospy.spin()
