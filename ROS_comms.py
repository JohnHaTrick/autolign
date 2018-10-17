#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    print data.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    
    rospy.init_node('listener', anonymous+True)

    # subscribe to auto_bridge's "from_autobox" topic / String type msgs
    rospy.Subscriber("from_autobox", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
