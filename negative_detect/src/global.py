#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32


def callback(data):

    if (negative_obstacle ==1 or float(pothole)==1):
        rospy.loginfo(rospy.get_caller_id() + "Attention !!!! il existe un obstacle negative ", data.data)  
    else:
        rospy.loginfo(rospy.get_caller_id() + "Avancer ===========> ", data.data)  

      
def fun():

    rospy.init_node('gloabal', anonymous=True)
    rospy.Subscriber("detect_negative", Float32, callback)
    rospy.Subscriber("topic2", type?, callback)
    rospy.spin()

if __name__ == '__main__':
     fun()
 