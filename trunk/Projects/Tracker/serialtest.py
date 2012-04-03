#!/usr/bin/env python

import sys
import serial
import struct

class Tracker(object):
    def __init__(self, port):
        self.END = chr(0xC0)
        self.ESC = chr(0xDB)
        self.ESC_END = chr(0xDC)
        self.ESC_ESC = chr(0xDD)
        self.ser = serial.Serial(port=port)
        self.ser.open()
        self.ser.flushInput()
        # read out any partially complete packet
        self.read()
        
    def read(self):
        line = []
    
        while True:
            c = self.ser.read(1)
            if c == self.END and len(line) != 0:
                break
            elif c == self.ESC:
                c = self.ser.read(1)
                if c == self.ESC_END:
                    line.append(self.END)
                elif c == self.ESC_ESC:
                    line.append(self.ESC)
            else:
                line.append(c)
                
        return ''.join(line)
        
    def quaternion(self):
        line = self.read()
        return struct.unpack('!Bffff', line)

port = '/dev/ttyACM0'
if (len(sys.argv) > 1):
    port = sys.argv[1]
tracker = Tracker(port)
while True:
    print tracker.quaternion()

