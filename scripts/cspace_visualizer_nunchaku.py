import sys
import numpy as np
from cspace_visualizer import *

fname = 'vertices_nunchaku.txt'
q = readVerticesFromFile(fname)

N = q.shape[0]
PX = np.empty((N))
PT = np.empty((N))
for i in range(0,N):
  PX[i] = q[i,1]
  PT[i] = q[i,8]

fname = 'vertices_nunchaku_path.txt'
qpath = readVerticesFromFile(fname)

N = qpath.shape[0]
pathX = np.empty((N))
pathT = np.empty((N))
for i in range(0,N):
  pathX[i] = qpath[i,1]
  pathT[i] = qpath[i,8]

plotCSpaceDelaunay(PX,PT, maximumEdgeLength=0.25, continuous=False)
plotCSpacePath(pathX,pathT)
plt.savefig("cspace.png", bbox_inches='tight')

plt.show()

