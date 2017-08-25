import pickle as pk

import sys
import numpy as np
import matplotlib.cm as cm
from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)
rc('font', family='serif', size=30)

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.tri as mtri
from scipy.spatial import Delaunay

fname = 'vertices.txt'


fh = open(fname, "r")

file_string = fh.read()
file_list = file_string.split('\n')

q = []

for line in file_list:
  config = line.split('\t')
  if(len(config)>1):
    qq = config[1].split(' ')
    q.append(qq)
q = np.array(q)

N = q.shape[0]

PX = np.empty((N))
PT = np.empty((N))

for i in range(0,N):
  PX[i] = q[i,1]
  PT[i] = q[i,3]



X = np.cos(PT)
Y = np.sin(PT)
Z = PX
###############

points2D=np.vstack([PX,PT]).T
tri = Delaunay(points2D)
print tri.simplices.shape, '\n', tri.simplices[0]

triangles = np.array((tri.simplices[0]))
for i in range(0, tri.simplices.shape[0]):
  simplex = tri.simplices[i]
  x = tri.points[simplex[0]]
  y = tri.points[simplex[1]]
  z = tri.points[simplex[2]]
  d0 = np.sqrt(np.dot(x-y,x-y))
  d1 = np.sqrt(np.dot(x-z,x-z))
  d2 = np.sqrt(np.dot(z-y,z-y))
  max_edge = max([d0, d1, d2])
  if max_edge <= 0.25:
    triangles = np.vstack((triangles, simplex))

print tri.simplices

#plt.triplot(PX,PT, triangles, edgecolor='red')#tri.simplices.copy())

#centers = np.sum(pts[triangles], axis=1, dtype='int')/3.0

cx = np.sum(PX[triangles],axis=1)/3.0
ct = np.sum(PT[triangles],axis=1)/3.0


colors = np.array([ (x-1)**2 for x,y in np.vstack((cx,ct)).T])

fig = plt.figure(0)
ax = fig.add_subplot(111)
plt.tripcolor(PX,PT,triangles, cmap=plt.cm.Spectral_r, facecolors=colors, edgecolors='none')
plt.xlabel('X')
fig.autofmt_xdate()
plt.ylabel(r'\theta',rotation=0)
dxax=0.45
dyax=0.05
fig.patch.set_facecolor('white')
plt.text(dxax, 0-dyax,r'$\gg$',transform=ax.transAxes, fontsize=50)
plt.text(dxax, 1-dyax,r'$\gg$',transform=ax.transAxes, fontsize=50)
ax.tick_params(axis='both', which='major', pad=15)
plt.savefig("cspace.png", bbox_inches='tight')


def long_edges(x, y, triangles, radio=22):
  out = []
  for points in triangles:
    #print points
    a,b,c = points
    d0 = np.sqrt( (x[a] - x[b]) **2 + (y[a] - y[b])**2 )
    d1 = np.sqrt( (x[b] - x[c]) **2 + (y[b] - y[c])**2 )
    d2 = np.sqrt( (x[c] - x[a]) **2 + (y[c] - y[a])**2 )
    max_edge = max([d0, d1, d2])
    #print points, max_edge
    if max_edge > 0.25:
      out.append(True)
    else:
      out.append(False)
      #if max_edge < 0.1:
      #  out.append(True)
      #else:
      #  out.append(False)
  return out


rad = np.linalg.norm(X)
zen = np.arccos(Z)
azi = np.arctan2(Y,X)

tris = mtri.Triangulation(zen, azi)

mask = long_edges(zen,azi, tris.triangles, 0.2)
tris.set_mask(mask)

#print len(tris.triangles)
#print len(tris.triangles)
#print len(tris.edges)
#plt.triplot(tris, 'bo-', lw=1)
#plt.show()

fig = plt.figure(1)
fig.patch.set_facecolor('white')
ax  = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(X,Y,Z, \
    triangles=tris.get_masked_triangles(),cmap=plt.cm.Spectral, \
     linewidth=0, antialiased=False)
ax.set_xlabel(r'r')
ax.set_ylabel(r'\phi')
ax.set_zlabel(r'z')
ax.tick_params(axis='both', which='major', pad=15)
plt.savefig("cspace3d.png", bbox_inches='tight')
plt.show()
