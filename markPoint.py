import sys
import os
from RadarDrivers_reconstruct.Radar import Radar


path = os.path.dirname(os.path.realpath(sys.argv[0]))
logFile = os.path.join(path, "point_array.txt")
