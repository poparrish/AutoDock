# Based on MAVROS Offboard control example
# https://dev.px4.io/en/ros/mavros_offboard.html
# Arms the Quad and flies to a pre-defined location

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


if __name__ == "__main__":

    rospy.init_node("example")#initializes a node called 'example'
    # subscribes to the 'mavros/state' topic. This has information vital to px4 function like heartbeat, arming, and flight mode
    rospy.Subscriber("mavros/state", State, state_changed)
    #creates publisher local_pos_pub. it is publishing to an EXISTING topic mavros/setpoint_position/local
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", data_class=PoseStamped, queue_size=10)
    #creates publisher local_vel_pub. it is publishing to an EXISTING topic mavros/setpoint_velocity/cmd_vel
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", data_class=TwistStamped, queue_size=1)
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)  # proxy service to arm the drones motors
    set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)  # proxy service to set the mode

    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(hz=50)

    #create a pose message
    pose = Pose(position=Point(x=0, y=-2, z=2), orientation=Quaternion(x=0, y=0, z=0, w=0))
    #create a velocity message
    velocity = Twist(linear=Vector3(x=0,y=0,z=0), angular=Vector3(x=0,y=0,z=0))
    # this throws the std_msgs/header at the beginning of each message being published.
    pose_msg = PoseStamped(pose=pose)
    # this throws the std_msgs/header at the beginning of each message being published.
    vel_msg=TwistStamped(twist=velocity)

    # send a few setpoints before starting
    #NO idea why but for some fucked up reason it stays in AUTO.LOITER mode if we don't run this...again no idea but for the
    #time being its needed to switch modes...may be a delay thing
    print "warming up..."
    for i in xrange(50):
        #local_pos_pub.publish(pose_msg)
        #local_vel_pub.publish(vel_msg)
        local_vel_pub.publish(vel_msg)
        rate.sleep()

    print "arming..."
    set_mode(custom_mode="OFFBOARD")  # sets the mode to "offboard control"
    arm(value=True)  # sets the motors to armed

    print "flying..."
    while not rospy.is_shutdown():
        #publish either the velocity or pose
        local_pos_pub.publish(pose_msg)
        #local_vel_pub.publish(vel_msg)

        #set the rate for the publishers
        rate.sleep()
    rospy.spin()


