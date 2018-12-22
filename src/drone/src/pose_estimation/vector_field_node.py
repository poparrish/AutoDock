#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Vector3,Twist


def get_field(pose):
    """
    builds a vector field and publishes xyz magnitudes under /custom/vector_field topic
    pose is the position
    :return:nada
    """
    print "TRUEPOSE",pose

    #rn its setup so that xyz are the magnitudes that the vector field should be imposing on the drone as if the
    #drone were the origin
    x = 0
    y = 0
    z = 3

    try:
        #where x,y,z are magnitudes with drone as origin
        vector_field.publish(Vector3(x=x,y=y,z=z))
    except Exception as e:
        print 'fuckdup %s' % e

if __name__ == '__main__':
    """
    where ROS lives
    """
    rospy.Subscriber("/custom/vision_pose",Twist,get_field)
    rospy.init_node("vector_field")
    vector_field = rospy.Publisher("/custom/vector_field",data_class=Vector3,queue_size=1)
    rospy.spin()
