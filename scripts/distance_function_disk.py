import sys
import numpy as np
from cspace_visualizer import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

font_size = 45
showSamples = True

robot_name = "disk"
folder_name = "../data/samples/"

fname = folder_name+"cspace_robot_"+robot_name+".samples"


Q = getPoints(fname)
P1 = []
P2 = []
P3 = []
dmax_index = -1
dmax = 0
ctr = 0
for q in Q:
  feasible = q[0]
  sufficient = q[1]
  ball_radius = q[2]
  number_of_states = q[3]
  x = q[4]
  y = q[5]
  P1 = np.append(P1,x)
  P2 = np.append(P2,y)
  P3 = np.append(P3,ball_radius)
  if ball_radius > dmax:
    dmax = ball_radius
    dmax_index = ctr
  ctr = ctr+1

P1 = np.array(P1,dtype='float')
P2 = np.array(P2,dtype='float')
P3 = np.array(P3,dtype='float')

P1 = np.delete(P1, dmax_index, axis=0)
P2 = np.delete(P2, dmax_index, axis=0)
P3 = np.delete(P3, dmax_index, axis=0)

print P1.shape
print P2.shape
print P3.shape

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(P1,P2,P3,"r")

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
