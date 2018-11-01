#!/usr/bin/env python

import rospy
#from auto_bridge.msg import from_autobox
from auto_messages.msg import from_autobox
from auto_messages.msg import to_autobox

# This script listens to from_autobox auto_bridge msgs,
#   prompts an autolign calculation,
#   then sends a to_autobox msg

def talker(data):
    # publish to auto_bridge's 'to_autobox' topic
    pub = rospy.Publisher('to_autobox', to_autobox, queue_size=10)
    # increment heartbeat
    heartbeat = data.heartbeat+1
    print "heartbeat = %s" % str(heartbeat)
    # pack ao_autobox message
    msg = to_autobox()
    msg.heartbeat = heartbeat
    # publish message
    pub.publish(msg)

def callback(data):
    # unpack message
    rospy.loginfo(rospy.get_caller_id() + " I heard seq %s", str(data.header.seq))
    # send message back to autobox
    talker(data)

def listener():
    # bring node 'autolign' up
    rospy.init_node('autolign', anonymous=True)
    # subscribe to auto_bridge's 'from_autobox' topic, from_autobox msg type
    rospy.Subscriber('from_autobox', from_autobox, callback)
    # loop
    rospy.spin()

if __name__ == '__main__':
    print "Starting node autolign"
    listener()
