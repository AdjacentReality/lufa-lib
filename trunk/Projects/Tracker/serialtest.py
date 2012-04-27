#!/usr/bin/env python

import sys
import serial
import struct

PACKET_QUAT = 0
PACKET_ACC = 1
PACKET_GYRO = 2
PACKET_MAG = 3
PACKET_COLOR = 4
PACKET_BLINK = 5
PACKET_IR = 6
PACKET_VERSION = 7
PACKET_MAX = 8

class Tracker(object):
    def __init__(self, port):
        self.END = chr(0xC0)
        self.ESC = chr(0xDB)
        self.ESC_END = chr(0xDC)
        self.ESC_ESC = chr(0xDD)
        self.ser = serial.Serial(port, 38400)
        self.ser.open()
        self.ser.flushInput()
        # read out any partially complete packet
        self.read_packet()
        
    def read_packet(self):
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
        
    def write_packet(self, packet):
        slipped = [];
        
        for c in packet:
            if c == self.END:
                slipped.append(self.ESC)
                slipped.append(self.ESC_END)
            elif c == self.ESC:
                slipped.append(self.ESC)
                slipped.append(self.ESC_ESC)
            else:
                slipped.append(c)
                
        slipped.append(self.END)
        self.ser.write(''.join(slipped))
        
    def set_color(self, rgb):
        packed = struct.pack('!BBBB', PACKET_COLOR, rgb[0], rgb[1], rgb[2])
        self.write_packet(packed)
        
    def quaternion(self):
        packet = self.read_packet()
        return struct.unpack('!Bffff', packet)

if __name__ == '__main__':
    port = '/dev/ttyACM0'
    if (len(sys.argv) > 1):
        port = sys.argv[1]
    tracker = Tracker(port)
    tracker.set_color((255, 0, 255))
    while True:
        tracker.set_color((255, 0, 255))
        print tracker.quaternion()

