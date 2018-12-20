#!/usr/bin/env python2
# This is the node in the ros graph which will send velocity commands directly to px4 through mavros

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TwistStamped, Vector3, Twist
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest, ParamSet, ParamGet

current_state = State()


def state_changed(msg):
    """
    State msg displays data such as whether drone is armed, system time, and if it has a heartbeat
    :param msg: see mavros_msgs.msg state   https://docs.ros.org/api/mavros_msgs/html/msg/State.html
    :return:nada
    """
    print msg
    global current_state
    current_state = msg

def setup():
    """
    Setup to run once at ./ros.sh launch.
    Currently just gets the drone into position.
    the platform starts moving on its own on a 10 second timed delay(configured in the animated_box.cc)
    :param hoverTime: roughly how long drone should hover at start position. depends on Hz
    :return:
    """



def callback(data):
    global imageLock
    imageLock = True
    print "IMAGELOCK"



if __name__ == "__main__":
    rospy.init_node("control")
    rospy.Subscriber("mavros/state",State,state_changed)
    rospy.Subscriber("custom/vision_pose",data_class=TwistStamped)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", data_class=PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", data_class=TwistStamped, queue_size=1)
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(hz=50)

    pose = Pose(position=Point(x=0, y=0, z=4), orientation=Quaternion(x=0, y=0, z=.55, w=1))
    pose_msg = PoseStamped(pose=pose)
    print "warming up..."
    for i in range(50):
        local_pos_pub.publish(pose_msg)
        rate.sleep()

    print "arming..."
    set_mode(custom_mode="OFFBOARD")  # sets the mode to "offboard control"
    arm(value=True)  # sets the motors to armed



    print "flying..."
    while not rospy.is_shutdown():
        local_pos_pub.publish(pose_msg)

        rate.sleep()
    rospy.spin()


