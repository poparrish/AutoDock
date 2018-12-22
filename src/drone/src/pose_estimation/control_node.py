#!/usr/bin/env python2
# This is the node in the ros graph which will send velocity commands directly to px4 through mavros
#overall control logic is setup so that we initialize with a desired pose. There is a timer called count
#that serves to keep track of when we have an image lock and when we dont. When we have an image
#lock we should switch over to the data coming in from vector_field_node. When we don't have a lock
#we should default to the initial desired pose sent at startup
import rospy
import time
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


def update_vector(vector):
    """
    this updates the vectors that we want to publish
    :param vector: callback data from the vector_field node
    :return: nada
    """
    global count
    count = 0#we have image lock so reset count
    global pubVectors
    pubVectors=[vector.x,vector.y, vector.z]



if __name__ == "__main__":
    rospy.init_node("control")
    rospy.Subscriber("mavros/state",State,state_changed)
    rospy.Subscriber("custom/vector_field",Vector3,update_vector)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", data_class=PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", data_class=TwistStamped, queue_size=1)
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

    count = 0
    pubVectors=[0,0,0]
    hz = 50
    rate = rospy.Rate(hz=hz)
    pose = Pose(position=Point(x=0, y=0, z=3), orientation=Quaternion(x=0, y=0, z=.55, w=1))
    pose_msg = PoseStamped(pose=pose)

    #NOTE not sure why but drone will not take off without this short loop included...I get that its a bit pointless
    #but its a necessary warmup sequence for some reason
    print "warming up..."
    for i in range(50):
        local_pos_pub.publish(pose_msg)
        rate.sleep()

    print "arming..."
    set_mode(custom_mode="OFFBOARD")  # sets the mode to "offboard control"
    arm(value=True)  # sets the motors to armed



    print "flying..."
    while not rospy.is_shutdown():
        global pubVectors
        if count > hz:#lost image lock for hz cycles so set desired pose back to origin (always equates to 1 second)
            local_pos_pub.publish(pose_msg)
            print"GOINGHOME"
        else:#we have an image lock and we should be relying on data from vector field
            vel = Twist(linear=Vector3(x=pubVectors[0],y=pubVectors[1],z=pubVectors[2]), angular=Vector3(x=0,y=0,z=0))
            vel_msg = TwistStamped(twist=vel)
            local_vel_pub.publish(vel_msg)
            print"IM GOING ON AN ADVENTURE!    -bilbo"
        global count
        count =count+1#always add 1 to count each time through
        rate.sleep()#delay hz
    rospy.spin()


