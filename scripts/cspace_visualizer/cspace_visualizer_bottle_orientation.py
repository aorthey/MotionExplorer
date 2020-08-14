import sys
import numpy as np
from cspace_visualizer import *

fname = 'vertices_bottle.txt'
q = readVerticesFromFile(fname)

N = q.shape[0]
PX = np.empty((N))
PT = np.empty((N))
for i in range(0,N):
  PX[i] = q[i,1]
  PT[i] = q[i,3]

fname = 'vertices_bottle_path.txt'
qpath = readVerticesFromFile(fname)

N = qpath.shape[0]
pathX = np.empty((N))
pathT = np.empty((N))
for i in range(0,N):
  pathX[i] = qpath[i,1]
  pathT[i] = qpath[i,3]

plotCSpaceDelaunay(PX,PT, maximumEdgeLength=0.25)
plotCSpacePath(pathX,pathT)
plt.savefig("cspace_bottle.png", bbox_inches='tight')

plotCSpaceCylindricalProjection(PX,PT)
plotCSpacePathCylindricalProjection(pathX,pathT)
plt.savefig("cspace3d.png", bbox_inches='tight')

plt.show()

