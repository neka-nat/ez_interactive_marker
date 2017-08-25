#!/usr/bin/env python
import sys
import rospy
import std_msgs.msg as sm

rospy.init_node('example')
with open(sys.argv[1]) as f:
    data = sm.String(data=f.read())
    pub = rospy.Publisher('/ez_interactive_marker/update_config',
                          sm.String, queue_size=1, latch=True)
    pub.publish(data)
rospy.sleep(5.0)
