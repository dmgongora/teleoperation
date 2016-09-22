#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float32


robot_collision = False
collision_cnt = 0

def callback(data):
    print 'Here!!!'
    global collision_cnt
    '''
    Convert uint16_t to float (Volts)
    '''
    #ADC_SCALE_FACTOR = 3.3/(2**16 - 1)
    #r = ADC_SCALE_FACTOR * data.adc4
    #l = ADC_SCALE_FACTOR * data.adc5
    '''
    Obtain the estimate of the impact point
    '''
    position = 1.84*((r - l)/(r + l))

    '''
    Compute the current
    '''
    current = (r + l)/1000
    if current < 250e-6:
        position = 0
        robot_collision = False
        collision_cnt = 0
    else:
        robot_collision = True
        collision_cnt = collision_cnt + 1

    if robot_collision:
        if collision_cnt == 1:
            pub.publish(position)
    '''
    Print data (publish data to a new topic)
    '''
    rospy.loginfo('Position %f', position)

    #rospy.loginfo('Current %f', current)
    #rospy.loginfo('left %f', r)

def listener():
    print 'Here!!'
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('adc', Float32, callback)  # Topic name, Message type

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print 'Here!'
    pub = rospy.Publisher('position', Float32, queue_size = 1)
    listener()
