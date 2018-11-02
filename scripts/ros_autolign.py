#!/usr/bin/env python

import os
import rospy
import rosbag
from auto_messages.msg import from_autobox
from auto_messages.msg import to_autobox

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

'''
def bagger(topic,data):
    # try recoring stuff
    bag = rosbag.Bag('bagfiles/test.bag','a')
    bag.write(topic,data)
    bag.close()
    print "from_autobox bagged"
'''  
