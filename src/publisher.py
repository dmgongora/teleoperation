#!/usr/bin/env python
import rospy
import serial
import serial.tools.list_ports
import json

from std_msgs.msg import Int32


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
    with open('./devices.json', 'r') as f:
	dev_desc = json.load(f)
        print dev_desc
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

uc = connect_to_uc()
print uc

rospy.init_node('publisher')

pub = rospy.Publisher('counter', Int32, queue_size = 10)

rate = rospy.Rate(20)

count = 0
while not rospy.is_shutdown():
    pub.publish(count)
    count +=1
    rate.sleep()
