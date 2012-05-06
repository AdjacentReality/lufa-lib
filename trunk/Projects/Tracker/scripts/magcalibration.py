"""
Magnetometer calibration routine for Tracker.  Run it whenever you change the
enclosure or mounting location, in whatever environment it will
be used in.  It eliminates hard iron and sensor offset errors.  It doesn't do
full soft iron calibration, but it should some day.

Run the script and rotate the Tracker through full spherical orientations until
it collects enough samples to calculate the calibration.  The new calibration
is automatically uploaded to the Tracker.

Ellipse fitting is ported from:
http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit

Copyright (c) 2009, Yury Petrov
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

    * Redistributions of source code must retain the above copyright 
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in 
      the documentation and/or other materials provided with the distribution
      
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
"""

import getopt
import sys
import numpy
from numpy import linalg
from serialtest import *

port = '/dev/ttyACM0'
num_samples = 50000
should_plot = False

def usage():
    print "Tracker Magnetometer Calibration"
    print ""
    print "Magnetometer calibration routine for Tracker.  Run it whenever you change the"
    print "enclosure or mounting location, in whatever environment it will"
    print "be used in.  It eliminates hard iron and sensor offset errors.  It doesn't do"
    print "full soft iron calibration, but it should some day."
    print ""
    print "Options:"
    print "  -h or --help        displays this helpful text"
    print "  -p or --plot        plot the samples, requires mpl_toolkits"
    print "  -n or --numsamples  number of samples to grab (%d default)" % num_samples
    print ""
    print "Usage:"
    print "  python magcalibration.py %s" % port
    print ""
    print "Run the script and rotate the Tracker through full spherical orientations until"
    print "it collects enough samples to calculate the calibration.  The new calibration"
    print "is automatically uploaded to the Tracker."

def calibrate(x, y, z):
    Dt = numpy.array([x*x, y*y, z*z, 2*x, 2*y, 2*z])
    D = numpy.transpose(Dt)
    a = numpy.dot(Dt, D)
    ones = numpy.ones([len(x), 1])
    b = numpy.dot(Dt, ones)
    v = linalg.solve(a, b)
    center = -1.0*v[3:6]/v[0:3]
    gam = 1.0 + (v[3]**2 / v[0] + v[4]**2 / v[1] + v[5]**2 / v[2]);
    radii = numpy.sqrt(gam/ v[0:3]);
    return (center, radii)

def plot(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d', aspect='equal')
    ax.scatter(x, y, z)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

if __name__ == '__main__':
    try:
        opts,args = getopt.gnu_getopt(sys.argv[1:], "pn:", ["plot", "numsamples="])
    except getopt.GetoptError, err:
        print str(err)
        usage()
        sys.exit(2)

    for o,a in opts:
        if o in ("-h", "--help"):
            usage()
            sys.exit()
        elif o in ("-p", "--plot"):
            should_plot = True
            from mpl_toolkits.mplot3d import Axes3D
            import matplotlib.pyplot as plt
        elif o in ("-n", "--numsamples"):
            num_samples = int(a)

    if len(args) > 0:
        port = args[0]

    tracker = Tracker(port)
    x = []
    y = []
    z = []

    # only get magnetometer data
    tracker.set_streaming_mode(0, 0, 0, 1)

    while len(x) < num_samples:
        packet = tracker.read_packet()
        if packet[0] == PACKET_MAG:
            x.append(packet[1])
            y.append(packet[2])
            z.append(packet[3])
        if not len(x) % 100:
            print "%f%%" % (100.0*(float(len(x))/float(num_samples)))

    [center, radii] = calibrate(numpy.array(x), numpy.array(y), numpy.array(z))
    
    # set the tracker back into quaternion mode
    tracker.set_streaming_mode(1, 0, 0, 0)
    
    print "Center of ellipse:\n %s" % str(center)
    print "Ellipse radii:\n %s\n" % str(radii)
    
    if should_plot:
        plot(x, y, z)
    
    while 1:
        print "Send to Tracker? [y/n]"
        choice = raw_input().lower()

        if len(choice) and choice[0] == 'y':
            print "Sending..."
            tracker.set_calibration(center[0], center[1], center[2], radii[0], radii[1], radii[2])
            break
        elif len(choice) and choice[0] == 'n':
            print "Not sending."
            break
            
