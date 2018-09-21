# Based on MAVROS Offboard control example
# https://dev.px4.io/en/ros/mavros_offboard.html

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TwistStamped, Twist, Vector3
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest, CommandTOL
from std_msgs.msg import String

current_state = State()


def state_changed(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("example")

    rospy.Subscriber("mavros/state", State, state_changed)
    # local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", data_class=PoseStamped, queue_size=10)
    setpoint_attitude = rospy.Publisher("mavros/setpoint_attitude/thrust", data_class=Thrust, queue_size=10)
    cmd_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", data_class=TwistStamped, queue_size=10)
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
    takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(hz=20)

    # wait for FCU connection
    print "waiting for connection..."
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = Pose(position=Point(x=0, y=0, z=3), orientation=Quaternion(x=0, y=0, z=0, w=0))
    pose_msg = PoseStamped(pose=pose)

    # send a few setpoints before starting
    # print "warming up..."
    # for i in xrange(20):
    #     local_pos_pub.publish(pose_msg)
    #     rate.sleep()

    print "arming..."
    set_mode(custom_mode="OFFBOARD")
    arm(value=True)

    # print "flying..."
    # for i in xrange(20):
    #     local_pos_pub.publish(pose_msg)
    #     rate.sleep()

    ret = takeoff(altitude=5)

    print "thrust..."
    while not rospy.is_shutdown():
        cmd_vel.publish(TwistStamped(twist=Twist(linear=Vector3(x=0, y=0, z=0))))
        # setpoint_attitude.publish(Thrust(thrust=1))
        rate.sleep()
