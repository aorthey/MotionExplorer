import pickle as pk

import sys
import numpy as np
import matplotlib.cm as cm
import matplotlib.colors as colors
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

pointsize = 20

def readVerticesFromFile(fname):
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
  return q

def plotCSpacePath(pathX,pathT):
  plt.plot(pathX,pathT,'-k',linewidth=5)
  plt.plot(pathX[0],pathT[0],'ok',markersize=pointsize)
  plt.plot(pathX[-1],pathT[-1],'ok',markersize=pointsize)

def plotCSpaceDelaunay(PX,PT, maximumEdgeLength = 0.25, continuous=True):
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
    if max_edge <= maximumEdgeLength:
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
  plt.xlabel('x')
  fig.autofmt_xdate()
  #plt.ylabel('q',rotation=0)
  plt.ylabel(r'\theta',rotation=0)
  dxax=0.45
  dyax=0.05
  fig.patch.set_facecolor('white')
  if continuous:
    plt.text(dxax, 0-dyax,r'$\gg$',transform=ax.transAxes, fontsize=50)
    plt.text(dxax, 1-dyax,r'$\gg$',transform=ax.transAxes, fontsize=50)

  ax.tick_params(axis='both', which='major', pad=15)

def plotCSpaceDelaunayGrey(P1,P2,maximumEdgeLength=0.25):
  points2D=np.vstack([P1,P2]).T
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
    if max_edge <= maximumEdgeLength:
      triangles = np.vstack((triangles, simplex))

  print triangles.shape
  #cx = np.sum(P1[triangles],axis=1)/3.0
  #ct = np.sum(P2[triangles],axis=1)/3.0

  zFaces = np.ones(triangles.shape[0])
  cmap = colors.LinearSegmentedColormap.from_list("", [(0.8,0.8,0.8),"grey","grey"])
  plt.tripcolor(P1, P2, triangles, cmap=cmap, facecolors=zFaces,edgecolors='none')

def plotCSpaceDelaunay3D(P1,P2,maximumEdgeLength=0.25):
  points2D=np.vstack([P1,P2]).T
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
    if max_edge <= maximumEdgeLength:
      triangles = np.vstack((triangles, simplex))

  zFaces = np.ones(triangles.shape[0])
  cmap = colors.LinearSegmentedColormap.from_list("", [(0.8,0.8,0.8),"grey","grey"])
  plt.tripcolor(P1, P2, triangles, cmap=cmap, facecolors=zFaces,edgecolors='none')


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

def plotCSpacePathCylindricalProjection(pathX,pathT):
  X = np.cos(pathT)
  Y = np.sin(pathT)
  Z = pathX
  plt.plot(X,Y,Z,'-k',linewidth=5)
  #plt.scatter(X[0],Y[0],Z[0],c='k',s=pointsize)
  plt.plot([X[0]], [Y[0]], [Z[0]], markerfacecolor='k', markeredgecolor='k', marker='o', markersize=pointsize)
  plt.plot([X[-1]], [Y[-1]], [Z[-1]], markerfacecolor='k', markeredgecolor='k', marker='o', markersize=pointsize)
  #plt.scatter(X[-1],Y[-1],Z[-1],s=20,c='k')

def plotCSpaceCylindricalProjection(PX,PT):
  X = np.cos(PT)
  Y = np.sin(PT)
  Z = PX

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

def getPoints(fname, maxElements = float('inf')):
  ### output: [feasible {True,False}, sufficient {True,False}, open_ball_radius
  ### {real}, number_of_states {int}, states {vector of real}]

  import xml.etree.ElementTree
  root = xml.etree.ElementTree.parse(fname).getroot()
  Q = list()
  ctr = 0
  for child in root.findall('state'):
    if ctr > maxElements:
      return Q
    sufficient = child.get('sufficient')
    feasible = child.get('feasible')
    open_ball_radius = child.get('open_ball_radius')
    state = child.text
    state = state.split(" ")

    if feasible=='yes':
      feasible = True
    else:
      feasible = False
    if sufficient=='yes':
      sufficient = True
    else:
      sufficient = False
    q = list()
    q.append(feasible)
    q.append(sufficient)
    q.append(open_ball_radius)

    for s in state:
      if s != '':
        q.append(s)

    Q.append(q)
    ctr += 1
  return Q

def generateInfeasibleSamplesOneDim(fname, dim1=0, maxElements = float('inf')):
  Q = getPoints(fname)
  theta = np.linspace(-np.pi,np.pi,100)
  P1 = []
  P2 = []
  ctr = 0
  for q in Q:
    feasible = q[0]
    #sufficient = q[1]
    #ball_radius = q[2]
    #num_states = q[3]
    x = q[4+dim1]
    if not feasible:
      ctr += 1
      for j in range(theta.shape[0]):
        P1 = np.append(P1,x)
        P2 = np.append(P2,theta[j])
    if ctr > maxElements:
      break
  # np.save('tmp_QS_dense_1',P1)
  # np.save('tmp_QS_dense_2',P2)
  return [P1,P2]

def generateInfeasibleSamplesTwoDim(fname, dim1=0, dim2=1, maxElements = float('inf')):
  Q = getPoints(fname)
  P1 = []
  P2 = []
  ctr = 0
  for q in Q:
    feasible = q[0]
    #sufficient = q[1]
    #ball_radius = q[2]
    #num_states = q[3]
    x = q[4+dim1]
    y = q[4+dim2]
    if not feasible:
      ctr += 1
      P1 = np.append(P1,x)
      P2 = np.append(P2,y)
    if ctr > maxElements:
      break
  # np.save('tmp_QS_dense_1',P1)
  # np.save('tmp_QS_dense_2',P2)
  return [P1,P2]

def plotSamplesOneDim(fname, dim1=0, maxElements = float('inf')):
  Q = getPoints(fname, maxElements)
  for q in Q:
    feasible = q[0]
    sufficient = q[1]
    d = float(q[2])
    x = float(q[4+dim1])
    if not feasible:
      plt.axvline(x, color='k', linewidth=1)
    elif sufficient:
      plt.axvspan(x-d, x+d, alpha=0.5, hatch="/")
    else:
      delta=0.05
      plt.fill([x-d, x+d, x+d, x-d], [-delta,-delta,+delta,+delta], fill=False, hatch='\\')

def plotSamplesTwoDim(fname, dim1=0, dim2=1, maxElements=float('inf')):
  Q = getPoints(fname, maxElements)
  for q in Q:
    t1 = float(q[4+dim1])
    t2 = float(q[4+dim2])
    feasible = q[0]
    if feasible:
      plt.plot(t1,t2,'o',color='0.6', linewidth=1)
    else:
      plt.plot(t1,t2,'x',color='k', linewidth=1)

