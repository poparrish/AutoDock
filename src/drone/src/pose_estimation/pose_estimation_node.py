#!/usr/bin/env python2
import rospy
import pose_estimation

default = {
    'h': (0, 185),
    's': (0, 125),
    'v': (216, 255),
}


def read_calibration():
    return {
        'h': (rospy.get_param("/h_min", default['h'][0]), rospy.get_param("/h_max", default['h'][1])),
        's': (rospy.get_param("/s_min", default['s'][0]), rospy.get_param("/s_max", default['s'][1])),
        'v': (rospy.get_param("/v_min", default['v'][0]), rospy.get_param("/v_max", default['v'][1])),
    }


if __name__ == '__main__':
    calibration = read_calibration()
    src = rospy.get_param("/capture_device", 0)

    pose_estimation.main(src=src, calibration=calibration, resize=False)
