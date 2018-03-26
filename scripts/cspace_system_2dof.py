import sys
import numpy as np
import matplotlib.pyplot as plt
from distance_point_line import DistPointLine

GRAY = (0.8,0.8,0.8)
R1=0.05
link_width=10
L1 = 0.5
L2 = 0.5

def GetObstacleSpheres():
  R1 = 0.25
  R2 = 0.15
  R3 = 0.2
  x = np.zeros((3,3)) #x,y,radius
  x[0,0]=-0.4
  x[0,1]=0.4
  x[0,2]=R1

  # x[1,0]=0.4
  # x[1,1]=0.7
  # x[1,2]=R2
  
  x[2,0]=0.4
  x[2,1]=-0.5
  x[2,2]=R3

  x[1,0]=-0.8
  x[1,1]=-0.5
  x[1,2]=R2
  return x


def GetWorldPositions(q):
  x = np.zeros((3,2))
  x[0,0]=0
  x[0,1]=0
  x[1,0]=L1*np.cos(q[0])
  x[1,1]=L1*np.sin(q[0])
  x[2,0]=L1*np.cos(q[0])+L2*np.cos(q[0]+q[1])
  x[2,1]=L1*np.sin(q[0])+L2*np.sin(q[0]+q[1])
  return x

def DistanceToSphere(q, x_sphere, y_sphere, r_sphere):
  x = GetWorldPositions(q)
  d1 = DistPointLine( x[0,0],x[0,1],x[1,0],x[1,1], x_sphere, y_sphere)
  d2 = DistPointLine( x[1,0],x[1,1],x[2,0],x[2,1], x_sphere, y_sphere)
  d = min(d1,d2)
  return d - r_sphere

def IsFeasible(q):
  X = GetObstacleSpheres()
  for k in range(X.shape[0]):
    d = DistanceToSphere(q, X[k,0],X[k,1],X[k,2])
    if d<=0:
      return False
  return True


def plotJoint(ax,x,y,R):
  c=plt.Circle((x,y),R+R/3.0,color='black',zorder=2)
  ax.add_artist(c)
  c=plt.Circle((x,y),R,color=GRAY,zorder=3)
  ax.add_artist(c)
  c=plt.Circle((x,y),R-R/2.0,color='black',zorder=4)
  ax.add_artist(c)

def plot2DOFAtConfig(ax,q):

  x = GetWorldPositions(q)
  plotJoint(ax,x[0,0],x[0,1],3.0/2.0*R1)
  plotJoint(ax,x[1,0],x[1,1],R1)
  plotJoint(ax,x[2,0],x[2,1],R1)
  plt.plot(x[:,0],x[:,1],color='black',linewidth=link_width+link_width/2.0,zorder=1)
  plt.plot(x[:,0],x[:,1],color=GRAY,linewidth=link_width,zorder=1)

def plot1DOFAtConfig(ax,q):

  x = GetWorldPositions(q)
  plotJoint(ax,x[0,0],x[0,1],3.0/2.0*R1)
  plotJoint(ax,x[1,0],x[1,1],R1)
  plt.plot(x[0:2,0],x[0:2,1],color='black',linewidth=link_width+link_width/2.0,zorder=1)
  plt.plot(x[0:2,0],x[0:2,1],color=GRAY,linewidth=link_width,zorder=1)

def plotObstacles(ax):

  X = GetObstacleSpheres()

  def plotSphere(ax,x,y,R):
    c=plt.Circle((x,y),R,color='black',zorder=-1)
    ax.add_artist(c)
    c=plt.Circle((x,y),R-0.02,color=GRAY,zorder=0)
    ax.add_artist(c)

  for k in range(X.shape[0]):
    plotSphere(ax, X[k,0],X[k,1],X[k,2])

