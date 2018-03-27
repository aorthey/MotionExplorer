import sys
import numpy as np
from cspace_visualizer import *
from cspace_system_2dof import *
import cspace_system_2dof as dof

c1 = (0.9,0.9,0.9)
c2 = (0.7,0.7,0.7)
c3 = (0.5,0.5,0.5)

N = 100
q1 = np.linspace(-np.pi,np.pi,N)
q2 = np.linspace(-np.pi,np.pi,N)
P1 = []
P2 = []
M = np.zeros((q1.shape[0], q2.shape[0]))
for i in range(q1.shape[0]):
  for j in range(q2.shape[0]):
    q = np.array((q1[i],q2[j]))
    if not IsFeasible(q):
      M[i,j] = 1
      P1 = np.append(P1,q1[i])
      P2 = np.append(P2,q2[j])

infeasibleColumns = np.sum(M,axis=1)>=N
Q1 = []
Q2 = []
for i in range(q1.shape[0]):
  for j in range(q2.shape[0]):
    q = np.array((q1[i],q2[j]))
    if infeasibleColumns[i]:
      Q1 = np.append(Q1,q1[i])
      Q2 = np.append(Q2,q2[j])

font_size = 45
offset = 0.08
p1 = np.array([0.5,2.57])
p2 = np.array([-1.57,-0.9])
p3 = np.array([2.5,-1.2])

###########################################################
fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'x',fontsize=font_size)
ax.set_ylabel(r'y',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=15)
lim=1.1
plt.axis([-lim,lim,-lim,lim])

dof.GRAY = c1
dof.plot2DOFAtConfig(ax,p1)
dof.GRAY = c2
dof.plot2DOFAtConfig(ax,p2)
dof.GRAY = c3
dof.plot2DOFAtConfig(ax,p3)

ax.annotate(r'q_1', (GetWorldPositions(p1)[2,0],GetWorldPositions(p1)[2,1]+offset))
ax.annotate(r'q_2', (GetWorldPositions(p2)[2,0]-offset,GetWorldPositions(p2)[2,1]+offset))
ax.annotate(r'q_3', (GetWorldPositions(p3)[2,0],GetWorldPositions(p3)[2,1]+offset))
plotObstacles(ax)
plt.savefig("2dof_workspace_M1.png", bbox_inches='tight')

###########################################################
fig = plt.figure(1)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'\theta_1',fontsize=font_size)
ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=15)

lim=3.14
plt.axis([-lim,lim,-lim,lim])
ax.annotate(r'q_1', (p1[0]+offset,p1[1]))
ax.annotate(r'q_2', (p2[0]+offset,p2[1]))
ax.annotate(r'q_3', (p3[0]+offset,p3[1]))
plt.plot(p1[0],p1[1],'o',color='black',markersize=10)
plt.plot(p2[0],p2[1],'o',color='black',markersize=10)
plt.plot(p3[0],p3[1],'o',color='black',markersize=10)

plotCSpaceDelaunayGrey(P1,P2,0.15)
plt.savefig("2dof_cspace_M1.png", bbox_inches='tight')

############################################################
#fig = plt.figure(2)
#fig.patch.set_facecolor('white')
#ax = fig.gca()
#ax.set_xlabel(r'x',fontsize=font_size)
#ax.set_ylabel(r'y',rotation=1.57,fontsize=font_size)
#ax.tick_params(axis='both', which='major', pad=15)
#lim=1.1
#plt.axis([-lim,lim,-lim,lim])
#dof.GRAY = c1
#dof.plot1DOFAtConfig(ax,p1)
#dof.GRAY = c2
#dof.plot1DOFAtConfig(ax,p2)
#dof.GRAY = c3
#dof.plot1DOFAtConfig(ax,p3)
#ax.annotate(r'q_1', (GetWorldPositions(p1)[1,0],GetWorldPositions(p1)[1,1]+offset))
#ax.annotate(r'q_2', (GetWorldPositions(p2)[1,0]-2*offset,GetWorldPositions(p2)[1,1]-3*offset))
#ax.annotate(r'q_3', (GetWorldPositions(p3)[1,0],GetWorldPositions(p3)[1,1]+offset))
#plotObstacles(ax)
#plt.savefig("2dof_workspace_M0.png", bbox_inches='tight')

###########################################################
#fig = plt.figure(3)
#fig.patch.set_facecolor('white')
#ax = fig.gca()
#ax.set_xlabel(r'\theta_1',fontsize=font_size)
#ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
#ax.tick_params(axis='both', which='major', pad=15)
#lim=3.14
#plt.axis([-lim,lim,-lim,lim])
#ax.annotate(r'q_1', (p1[0]+offset,p1[1]))
#ax.annotate(r'q_2', (p2[0]+offset,p2[1]))
#ax.annotate(r'q_3', (p3[0]-offset,p3[1]+2*offset))
#plt.plot(p1[0],p1[1],'o',color='black',markersize=10)
#plt.plot(p2[0],p2[1],'o',color='black',markersize=10)
#plt.plot(p3[0],p3[1],'o',color='black',markersize=10)
#
#plotCSpaceDelaunayGrey(Q1,Q2)
#plt.savefig("2dof_cspace_M0.png", bbox_inches='tight')
############################################################
plt.show()
