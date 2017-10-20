#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
stop_command = Twist()


def callback(data):
    range_min = data.range_min
    range_max = data.range_max

    ranges = data.ranges
    number_readings = len(ranges)

    front_reading = round(ranges[0], 2)
    right_reading = round(ranges[number_readings//4], 2)
    rear_reading = round(ranges[(number_readings//4) * 2], 2)
    left_reading = round(ranges[(number_readings/4) * 3], 2)

    overview_readings = [front_reading, left_reading, right_reading, rear_reading]

    errorData = False
    for rId, reading in enumerate(overview_readings):
        if (reading > range_max) or (reading < range_min):
            overview_readings[rId] = "ERR"
            cmd_vel_pub.publish(stop_command)
            errorData = True


    rospy.loginfo('--------------------------------------------')
    rospy.loginfo('                     %s                     ', overview_readings[0])
    rospy.loginfo('%s                                        %s', overview_readings[1], overview_readings[2])
    rospy.loginfo('                     %s                     ', overview_readings[3])
    if errorData :
        rospy.loginfo('-----ROBOT STOPPED DUE TO ERROR IN DATA-----')


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laserListener', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
