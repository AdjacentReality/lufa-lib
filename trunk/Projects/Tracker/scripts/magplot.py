import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from serialtest import *

port = '/dev/ttyACM0'
out_file = 'samples.txt'
if (len(sys.argv) > 1):
    port = sys.argv[1]
if (len(sys.argv) > 2):
    out_file = sys.argv[2]
tracker = Tracker(port)
x = []
y = []
z = []
num_samples = 5000
while len(x) < num_samples:
    packet = tracker.read_packet()
    if packet[0] == PACKET_MAG:
        x.append(packet[1])
        y.append(packet[2])
        z.append(packet[3])
    if not len(x) % 100:
        print "%f%%" % (100.0*(float(len(x))/float(num_samples)))

f = open(out_file, 'w')
f.write("x = ")
f.write(str(x))
f.write(";\ny = ")
f.write(str(y))
f.write(";\nz = ")
f.write(str(z))
f.write(";\n")
f.close()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', aspect='equal')
ax.scatter(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
