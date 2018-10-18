#!/usr/bin/env python

import rospy
#from std_msgs.msg import String

# This script should listen to the from_autobox auto_bridge msgs,
#   prompt an autolign calculation,
#   then send out a to_autobox msg

def callback(data):
    print "from_autobox:"
    print data.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    
    rospy.init_node('autolign', anonymous+True)

    # subscribe to auto_bridge's "from_autobox" topic / String type msgs
    rospy.Subscriber("from_autobox", from_autobox, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
