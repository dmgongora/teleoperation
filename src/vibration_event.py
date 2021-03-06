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
import serial
import serial.tools.list_ports
import json
from std_msgs.msg import Float32
import time
#from rospy.numpy_msg import numpy_msg

def connect_to_uc():
    """ Connect to uc set in JSON file and return handler.

    Args:
    none.

    Returns: 
    Handler to serial port connection.

    Raises:
    IOError: The device described in the JSON file was not found. 
    """
    dev_desc = {}
    print 'Opening file...'
    with open('devices.json', 'r') as f:
	dev_desc = json.load(f)
    connectedDevices = []
    for portcandidate in serial.tools.list_ports.comports():
	port_type = portcandidate[2]
        dev_pid = dev_desc["frdm"]["serial"]["pid"]
        dev_vid = dev_desc["frdm"]["serial"]["vid"]
	if port_type.find('USB VID:PID={}:{}'.format(dev_vid, dev_pid)) >= 0:
	    print "Found %s"%(portcandidate[0],)
	    connectedDevices.append(portcandidate[0])
	else:
	    print port_type
    #Connect to first device found
    uc = None
    if connectedDevices:
	portname = connectedDevices[0]
        dev_baudrate = dev_desc["frdm"]["serial"]["baud"]
	uc = serial.Serial(port=portname,
                           baudrate=dev_baudrate, writeTimeout = 0.05)
    else:
	raise IOError("%s not detected."%(dev_desc["name"],))
    return uc


def callback(data):
    global collision_time
    '''
    position range is [-1, 1]
    '''
    position = 200 * ((data.data + 1.0)/2.0)
    if position <= 255 and position >= 0:
        command = chr(100)
        '''
        Print data (publish data to a new topic)
        '''
        rospy.loginfo('Position %f', position)
        # Collision have to be X seconds apart 
        if time.time() - collision_time > 5:
            uc.write(command)
            collision_time = time.time()

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('vibration')

    rospy.Subscriber('adc', Float32, callback)  # Topic name, Message type
    #numpy_msg(Float32)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


'''
Connect to uc
(GLOBAL)
'''
uc = connect_to_uc()
collision_time = 0
print uc

if __name__ == '__main__':
    listener()
