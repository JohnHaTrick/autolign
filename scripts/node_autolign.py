#!/usr/bin/env python

import rospy
from auto_bridge.msg import from_autobox
#from auto_bridge import from_autobox

# This script should listen to the from_autobox auto_bridge msgs,
#   prompt an autolign calculation,
#   then send out a to_autobox msg

def callback(data):
    print "from_autobox:"
    rospy.loginfo(rospy.get_caller_id() + " I heard seq %s", str(data.header.seq))
    print "X1 is at (%f East, %f North)" % (data.E_m,data.N_m)

def listener():
    
    rospy.init_node('autolign', anonymous=True)

    # subscribe to auto_bridge's 'from_autobox' topic, from_autobox msg type
    rospy.Subscriber('from_autobox', from_autobox, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
