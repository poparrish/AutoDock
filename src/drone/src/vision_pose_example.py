import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest, ParamSet, ParamGet

current_state = State()


def state_changed(msg):
    # print msg
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("example")

    rospy.Subscriber("mavros/state", State, state_changed)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", data_class=PoseStamped, queue_size=10)
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(hz=20)

    # wait for FCU connection
    print "waiting for connection..."
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    # inside simulator, enable ekf2 vision pose https://dev.px4.io/en/advanced/switching_state_estimators.html
    # https://github.com/PX4/Firmware/issues/7408
    # param set EKF2_AID_MASK 24 # no gps

    print "connected"
    p = rospy.Publisher('/mavros/vision_pose/pose', data_class=PoseStamped, queue_size=10)

    # generate fake pose data
    i = 0
    while not rospy.is_shutdown():
        pose = Pose(position=Point(x=i, y=5, z=5), orientation=Quaternion(x=0, y=0, z=0, w=0))
        pose_msg = PoseStamped(pose=pose)

        p.publish(pose_msg)
        rate.sleep()
        i += .1
        print i