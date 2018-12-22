#!/usr/bin/env python2
import rospy
import pose_estimation
from pose_estimation import *
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Polygon, Point32
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
    calibration = read_calibration()#updates camera HSV calibration. Useless functionality really with gazebo lag
    # subscribes to the 'mavros/state' topic. This has information vital to px4 function like heartbeat, arming, and flight mode
    rospy.Subscriber("mavros/state", State, state_changed)
    src = rospy.get_param("/capture_device", 0)#configures the simulated camera
    rospy.init_node("pose_estimation")#creates the node we will publish our /custom/vision_pose topic from
    # creates publisher p with topic "/custom/vision_pose" of datatype TwistStamped
    vision_pose = rospy.Publisher("/custom/vision_pose", data_class=Twist, queue_size=1)
    platform_coords= rospy.Publisher("/custom/platform_coords", data_class=Polygon, queue_size=1)

    hz = 50
    rate = rospy.Rate(hz=hz)#setpoint publishing must be faster than the px4 heartbeat of 2hz


    cap = start_capture(src, False)#starts the simulated camera

    while not rospy.is_shutdown():
        try:
            ret, frame = cap.read()#captures one frame from simulated camera

            try:
                pose_ret = estimate_pose(frame, calibration)#estimates the pose based on last frame from camera

                if pose_ret is None:#if there was no pose data then assume we don't have a target lock on the target
                    print 'no image lock'
                else:
                    #twist message type keeps everything as euler angles...for the time being, may switch to quaternion later
                    pose = Twist(linear=Vector3(x=pose_ret[0][0], y=pose_ret[0][1], z=pose_ret[0][2]),
                                 angular=Vector3(x=pose_ret[1][0], y=pose_ret[1][1], z=pose_ret[1][2]))
                    #no existing 2dpoint data type and its a pain in teh arse to get custom msg types going in sim...so we use 3d points
                    #and omit the zdimension
                    platform_msg = Polygon(points = [Point32(x=pose_ret[2][0][0],y=pose_ret[2][0][1],z = 0),
                                                     Point32(x=pose_ret[2][1][0], y=pose_ret[2][1][1], z=0),
                                                     Point32(x=pose_ret[2][2][0], y=pose_ret[2][2][1], z=0),
                                                     Point32(x=pose_ret[2][3][0], y=pose_ret[2][3][1], z=0),
                                                     Point32(x=pose_ret[2][4][0], y=pose_ret[2][4][1], z=0),
                                                     Point32(x=pose_ret[2][5][0], y=pose_ret[2][5][1], z=0),
                                                     Point32(x=pose_ret[2][6][0], y=pose_ret[2][6][1], z=0),])


                    #publish our messages
                    vision_pose.publish(pose)
                    platform_coords.publish(platform_msg)

            except Exception as e:
                print 'failed to return pose %s' % e
                print e
        except Exception as e:
            print 'failed to publish to /custom/vision_pose %s' % e
        rate.sleep()#this is where we control the frequency this ros node runs at.
    rospy.spin()#this simply keeps python from exiting until the node is stopped.