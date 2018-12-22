#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Polygon, Point32

def callback(target):
    """
    callback method triggered everytime pose_estimation_node publishes a new set of target coordinates.
    Note that these coordinates are the X,Y pixel locations with the origin in the upper left hand corner
    this is the default for OpenCV3
    :param target: 2d array of Polygon[Point32] see https://wiki.ros.org/common_msgs
    :return:
    """

if __name__ == '__main__':
    """
    where ROS lives
    """

    #make a subscriber for the target data coming from pose_estimation_node.py
    rospy.Subscriber("custom/platform_coords",data_class=Polygon,queue_size=1)
    #create gimbal_control node
    rospy.init_node("gimbal_control")

    hz = 50
    rate = rospy.Rate(hz=hz)


