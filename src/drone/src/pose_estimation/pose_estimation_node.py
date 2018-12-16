#!/usr/bin/env python2
import rospy
import pose_estimation
from pose_estimation import *
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest, StreamRate

default = {
    'h': (0, 185),
    's': (0, 125),
    'v': (216, 255),
}

current_state = State()

def read_calibration():
    """
    reads calibration settings in the launch file
    :return:muchos
    """
    return {
        'h': (rospy.get_param("/h_min", default['h'][0]), rospy.get_param("/h_max", default['h'][1])),
        's': (rospy.get_param("/s_min", default['s'][0]), rospy.get_param("/s_max", default['s'][1])),
        'v': (rospy.get_param("/v_min", default['v'][0]), rospy.get_param("/v_max", default['v'][1])),
    }

def state_changed(msg):
    """
    State msg displays data such as whether drone is armed, system time, and if it has a heartbeat
    :param msg: see mavros_msgs.msg state   https://docs.ros.org/api/mavros_msgs/html/msg/State.html
    :return:nada
    """
    print msg
    global current_state
    current_state = msg


if __name__ == '__main__':
    """
    where ROS lives
    """
    calibration = read_calibration()
    rospy.Subscriber("mavros/state", State, state_changed)
    src = rospy.get_param("/capture_device", 0)
    rospy.init_node("Vision_Pose")
    p = rospy.Publisher("/custom/vision_pose", data_class=TwistStamped, queue_size=1)
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
    stream_rate = rospy.ServiceProxy("mavros/set_stream_rate",StreamRate)

    hz = 50
    sr = 100
    rate = rospy.Rate(hz=hz)#setpoint publishing must be faster than the px4 heartbeat of 2hz

    print "arming..."
    set_mode(custom_mode="OFFBOARD")
    arm(value=True)
    stream_rate(stream_id=0,message_rate=sr,on_off=1)
    cap = start_capture(src, False)

    while not rospy.is_shutdown():
        try:
            ret, frame = cap.read()

            try:
                pose_ret = estimate_pose(frame, calibration)

                if pose_ret is None:
                    print 'no image lock'
                else:
                    # convert to quaternion
                    pose = Twist(linear=Vector3(x=pose_ret[0][0], y=pose_ret[0][1], z=pose_ret[0][2]),
                                 angular=Vector3(x=pose_ret[1][0], y=pose_ret[1][1], z=pose_ret[1][2]))

                    # this throws the std_msgs/header at the beginning of each message being published. px4 requires it
                    pose_msg = TwistStamped(twist=pose)

                    p.publish(pose_msg)

            except Exception as e:
                print 'failed to reconstruct target %s' % e
                print e
        except Exception as e:
            print 'failed to publish to /custom/vision_pose %s' % e
        rate.sleep()
    rospy.spin()
