import numpy as np
from matplotlib import colors
import matplotlib.pyplot as plt
import os

filepath = os.path.split(os.path.realpath(__file__))[0]
map_file = os.path.join(filepath, "map.txt")
path_file = os.path.join(filepath, "path.txt")

map_reachable = list()
map_unknown = list()

with open(map_file, 'r') as f:
    for line in f.readlines():
        line = line.split(",")
        p = (int(line[0][1:]), int(line[1]))
        if(line[2] == 'REACHABLE)\n'):
            map_reachable.append(p)
        else:
            map_unknown.append(p)

path_data = list()
with open(path_file, 'r') as f:
    for line in f.readlines():
        line = line.split(",")
        line = (int(line[0][1:]), int(line[1][:-2]))
        path_data.append(line)

plt.scatter(*zip(*map_reachable))
if len(map_unknown) > 0:
    plt.scatter(*zip(*map_unknown))
plt.scatter(*zip(*path_data))

se = [path_data[0], path_data[-1]]
plt.scatter(*zip(*se))
plt.show()
