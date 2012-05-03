import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from serialtest import *

port = '/dev/ttyACM0'
if (len(sys.argv) > 1):
    port = sys.argv[1]
tracker = Tracker(port)
x = []
y = []
z = []
num_samples = 10000
while len(x) < num_samples:
    packet = tracker.read_packet()
    if packet[0] == PACKET_MAG:
        x.append(packet[1])
        y.append(packet[2])
        z.append(packet[3])
    if not len(x) % 100:
        print "%f%%" % (100.0*(len(x)/float(num_samples)))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
