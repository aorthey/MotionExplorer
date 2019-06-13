import sys
import numpy as np
from cspace_visualizer import *
from cspace_system_2dof import *
import cspace_system_2dof as dof
from matplotlib.ticker import MaxNLocator


c1 = (0.9,0.9,0.9)
c2 = (0.7,0.7,0.7)
c3 = (0.5,0.5,0.5)

N = 500
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

font_size = 25
offset = 0.5
p1 = np.array([0.5,2.57])
p2 = np.array([-1.57,-0.9])
p3 = np.array([2.5,-1.2])
symbol='x'
greyshade = 0.75

x1loc= (p1[0]-offset,p1[1]-offset)
x2loc= (p3[0]-offset,p3[1]-offset)
y1loc= (p1[0]-offset,-offset)
y2loc= (p3[0]-offset,-offset)
fiber1loc= (p1[0]-1.6,1.5)
fiber2loc= (p3[0]-1.6,1.5)
# fiber1loc= (p1[0]-0.6,-1.5)
# fiber2loc= (p3[0]-0.6,-1.5)
############################################################
fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'\theta_1',fontsize=font_size)
ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=10, labelsize=0.8*font_size)

lim=3.14
plt.axis([-lim,lim,-lim,lim])

ax.annotate(r'x_1', x1loc)
ax.annotate(r'x_2', x2loc)
plt.plot(p1[0],p1[1],'o',color='black',markersize=10)
plt.plot(p3[0],p3[1],'o',color='black',markersize=10)

ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
plotCSpaceDelaunayGrey(P1,P2,shade=greyshade)
plt.savefig("2dof_cspace_1.png", bbox_inches='tight')

############################################################

fig = plt.figure(1)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'\theta_1',fontsize=font_size)
ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=10, labelsize=0.8*font_size)

lim=3.14
plt.axis([-lim,lim,-lim,lim])

ax.annotate(r'y_1', y1loc)
ax.annotate(r'y_2', y2loc)
plt.plot(p1[0],0,'o',color='black',markersize=10)
plt.plot(p3[0],0,'o',color='black',markersize=10)

ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
plt.axvline(p1[0], color='k', linestyle='dashed', linewidth=1)
plt.axvline(p3[0], color='k', linestyle='dashed', linewidth=1)
plt.axhline(0, color='k', linewidth=1)

plotCSpaceDelaunayGrey(Q1,Q2,shade=greyshade)
plt.savefig("2dof_cspace_2.png", bbox_inches='tight')
#############################################################
fig = plt.figure(2)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'\theta_1',fontsize=font_size)
ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=10, labelsize=0.8*font_size)
lim=3.14
plt.axis([-lim,lim,-lim,lim])

# ax.annotate(r'y_1', y1loc)
# ax.annotate(r'y_2', y2loc)
plt.plot(p1[0],0,'o',color='black',markersize=10)
plt.plot(p3[0],0,'o',color='black',markersize=10)

# ax.annotate(r'x_1', x1loc)
# ax.annotate(r'x_2', x2loc)
plt.plot(p1[0],p1[1],'o',color='black',markersize=10)
plt.plot(p3[0],p3[1],'o',color='black',markersize=10)

ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.yaxis.set_major_locator(MaxNLocator(integer=True))

f1 = ax.annotate(r'\pi^{-1}(y_1)', fiber1loc, rotation=1.57)
f2 = ax.annotate(r'\pi^{-1}(y_2)', fiber2loc, rotation=1.57)
# f1.set_rotation(90)
# f2.set_rotation(90)
plt.axvline(p1[0], color='k', linestyle='solid', linewidth=2)
plt.axvline(p3[0], color='k', linestyle='solid', linewidth=2)
plt.axhline(0, color='k', linewidth=1)

plotCSpaceDelaunayGrey(P1,P2,shade=0.88)
plotCSpaceDelaunayGrey(Q1,Q2,shade=greyshade)
plt.savefig("2dof_cspace_3.png", bbox_inches='tight')
#############################################################
plt.show()
