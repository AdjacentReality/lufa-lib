import tracker
import sys

port = '/dev/ttyACM0'
if (len(sys.argv) > 1):
    port = sys.argv[1]

tracker = tracker.Tracker(port)
tracker.bootload()
