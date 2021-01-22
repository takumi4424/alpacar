#!/usr/bin/python3

import rospy, sys
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('test_publisher', sys.argv)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)

    for i in range(30):
        rospy.loginfo('pub[{}]'.format(i))
        msg = Twist()
        msg.angular.x = 1
        msg.angular.y = 2
        msg.angular.z = i
        msg.linear.x = 3
        msg.linear.y = 4
        msg.linear.z = i

        pub.publish(msg)
        rate.sleep()
