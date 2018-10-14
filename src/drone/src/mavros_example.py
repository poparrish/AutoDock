# Based on MAVROS Offboard control example
# https://dev.px4.io/en/ros/mavros_offboard.html
# Arms the Quad and flies to a pre-defined location

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest

current_state = State()


def state_changed(msg):
    print msg
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

    pose = Pose(position=Point(x=3, y=6, z=3), orientation=Quaternion(x=0, y=0, z=0, w=0))
    pose_msg = PoseStamped(pose=pose)

    # send a few setpoints before starting
    print "warming up..."
    for i in xrange(50):
        local_pos_pub.publish(pose_msg)
        rate.sleep()

    print "arming..."
    set_mode(custom_mode="OFFBOARD")
    arm(value=True)

    print "flying..."
    while not rospy.is_shutdown():
        local_pos_pub.publish(pose_msg)
        rate.sleep()
