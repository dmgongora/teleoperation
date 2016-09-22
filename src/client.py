from socket import *
import serial
import serial.tools.list_ports

import json
import os

def connect_to_pc():
    """ Create socket object and load pc info.

    Args:
    none.

    Returns:
    - Socket object
    - PC address (tuple)

    Raises:
    none
    """
    with open('devices.json', 'r') as f:
	dev_desc = json.load(f)
    host = dev_desc["pc"]["ip"]
    port = dev_desc["pc"]["port"]
    addr = (host, port)
    return socket(AF_INET, SOCK_DGRAM), addr

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
    with open('devices.json', 'r') as f:
	dev_desc = json.load(f)
    connectedDevices = []
    for portcandidate in serial.tools.list_ports.comports():
	port_type = portcandidate[2]
        dev_pid = dev_desc["teensy"]["serial"]["pid"]
        dev_vid = dev_desc["teensy"]["serial"]["vid"]
	if port_type.find('USB VID:PID={}:{}'.format(dev_vid, dev_pid)) >= 0:
	    print "Found %s"%(portcandidate[0],)
	    connectedDevices.append(portcandidate[0])
	else:
	    print port_type
    #Connect to first device found
    uc = None
    if connectedDevices:
	portname = connectedDevices[0]
        dev_baudrate = dev_desc["teensy"]["serial"]["baud"]
	uc = serial.Serial(port=portname,
                           baudrate=dev_baudrate, writeTimeout = 0.05)
    else:
	raise IOError("%s not detected."%(dev_desc["name"],))
    return uc

if __name__ == '__main__':
    uc = connect_to_uc()
    udp_sock, pc_addr = connect_to_pc()
    uc.flushInput()

    DATA_AVAILABLE = False
    try:
        while True:
            if (uc.inWaiting()) > 0:
                data = uc.read(uc.inWaiting())
                DATA_AVAILABLE = True
            if DATA_AVAILABLE:
                # Process data here
                udp_sock.sendto(data, pc_addr)
                DATA_AVAILABLE = False
    except KeyboardInterrupt:
        data = 'exit'
        udp_sock.sendto(data, pc_addr)
        udp_sock.close()
        os._exit(0)
