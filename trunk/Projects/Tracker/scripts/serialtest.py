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
PACKET_STREAM = 7
PACKET_VERSION = 8
PACKET_ID = 9
PACKET_MAX = 10

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
        self.read_serial()
    
    def parse_packet(self, packet):
        t = ord(packet[0])
    
        if t == PACKET_QUAT:
            return struct.unpack('!Bffff', packet)
        elif t == PACKET_ACC or t == PACKET_GYRO or t == PACKET_MAG:
            return struct.unpack('!Bhhh', packet)
        elif t == PACKET_COLOR or t == PACKET_BLINK:
            return struct.unpack('!BBBB', packet)
        elif t == PACKET_IR:
            return struct.unpack('!BB', packet)
        elif t == PACKET_VERSION or t == PACKET_ID:
            return struct.unpack('!BI', packet)
        else:
            print "Unknown packet type %d" % t
            return None
        
    def read_serial(self):
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
                
        return line
        
    def read_packet(self):
        line = self.read_serial()
                
        return self.parse_packet(''.join(line))
        
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
        
    def set_streaming_mode(self, quat, acc, gyro, mag):
        mask = (quat << PACKET_QUAT) | (acc << PACKET_ACC) | (gyro << PACKET_GYRO) |\
                (mag << PACKET_MAG)
        packed = struct.pack('!BB', PACKET_STREAM, mask)
        self.write_packet(packed)

if __name__ == '__main__':
    port = '/dev/ttyACM0'
    if (len(sys.argv) > 1):
        port = sys.argv[1]
    tracker = Tracker(port)
    tracker.set_color((255, 0, 255))
    tracker.set_streaming_mode(1, 0, 0, 0)
    while True:
        print tracker.read_packet()

