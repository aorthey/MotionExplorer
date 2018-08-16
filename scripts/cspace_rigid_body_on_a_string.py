import sys
import numpy as np
from cspace_visualizer import *
from cspace_system_2dof import *
import cspace_system_2dof as dof

def getPoints(fname):
  import xml.etree.ElementTree
  root = xml.etree.ElementTree.parse(fname).getroot()
  Q = list()
  for child in root.findall('state'):
    sufficient = child.get('sufficient')
    feasible = child.get('feasible')
    state = child.text
    state = state.split(" ")
    x = state[2]
    y = state[3]
    theta = state[4]
    if feasible=='yes':
      feasible = True
    else:
      feasible = False
    if sufficient=='yes':
      sufficient = True
    else:
      sufficient = False
    q = list()
    q.append(state[2])
    q.append(state[3])
    q.append(feasible)
    q.append(sufficient)
    Q.append(q)
  return Q

def plotPoints(fname):
    
  Q = getPoints(fname)
  for q in Q:
    x = float(q[0])
    y = float(q[1])
    feasible = q[2]
    sufficient = q[3]
    if not feasible:
      plt.axvline(x, color='k', linewidth=1)
    elif sufficient:
      plt.axvline(x, color='0.8', linewidth=1)
    else:
      delta=0.05
      plt.plot([x,x],[-delta,+delta],'-',color='0.8', linewidth=1)

def plotQuotientSpaceBackground(fname):
  Q = getPoints(fname)
  theta = np.linspace(-np.pi,np.pi,100)
  P1 = []
  P2 = []
  for q in Q:
    x = q[0]
    feasible = q[2]
    if not feasible:
      for j in range(theta.shape[0]):
        P1 = np.append(P1,x)
        P2 = np.append(P2,theta[j])
  plotCSpaceDelaunayGrey(P1,P2,0.15)

font_size = 45
fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'x',fontsize=font_size)
ax.set_ylabel(r'\theta',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=15)
lim=5.0
plt.axis([-lim,lim,-3.14,3.14])
fname = "../build/qs_samples.roadmap"
plotPoints(fname)
qfname = "../build/qs_dense.roadmap"
plotQuotientSpaceBackground(qfname)
plt.show()

# ###########################################################
# fig = plt.figure(0)
# fig.patch.set_facecolor('white')
# ax = fig.gca()
# ax.set_xlabel(r'x',fontsize=font_size)
# ax.set_ylabel(r'y',rotation=1.57,fontsize=font_size)
# ax.tick_params(axis='both', which='major', pad=15)
# lim=1.1
# plt.axis([-lim,lim,-lim,lim])

# dof.GRAY = c1
# dof.plot2DOFAtConfig(ax,p1)
# dof.GRAY = c2
# dof.plot2DOFAtConfig(ax,p2)
# dof.GRAY = c3
# dof.plot2DOFAtConfig(ax,p3)

# ax.annotate(r'q_1', (GetWorldPositions(p1)[2,0],GetWorldPositions(p1)[2,1]+offset))
# ax.annotate(r'q_2', (GetWorldPositions(p2)[2,0]-offset,GetWorldPositions(p2)[2,1]+offset))
# ax.annotate(r'q_3', (GetWorldPositions(p3)[2,0],GetWorldPositions(p3)[2,1]+offset))
# plotObstacles(ax)
# plt.savefig("2dof_workspace_M1.png", bbox_inches='tight')

# ###########################################################
# fig = plt.figure(1)
# fig.patch.set_facecolor('white')
# ax = fig.gca()
# ax.set_xlabel(r'\theta_1',fontsize=font_size)
# ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
# ax.tick_params(axis='both', which='major', pad=15)

# lim=3.14
# plt.axis([-lim,lim,-lim,lim])
# ax.annotate(r'q_1', (p1[0]+offset,p1[1]))
# ax.annotate(r'q_2', (p2[0]+offset,p2[1]))
# ax.annotate(r'q_3', (p3[0]+offset,p3[1]))
# plt.plot(p1[0],p1[1],'o',color='black',markersize=10)
# plt.plot(p2[0],p2[1],'o',color='black',markersize=10)
# plt.plot(p3[0],p3[1],'o',color='black',markersize=10)

# plotCSpaceDelaunayGrey(P1,P2,0.15)
# plt.savefig("2dof_cspace_M1.png", bbox_inches='tight')

#############################################################
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
#
############################################################
#fig = plt.figure(3)
#fig.patch.set_facecolor('white')
#ax = fig.gca()
#ax.set_xlabel(r'\theta_1',fontsize=font_size)
#ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
#ax.tick_params(axis='both', which='major', pad=15)
#lim=3.14
#plt.axis([-lim,lim,-lim,lim])
#ax.annotate(r'q_1', (p1[0]+offset,offset))
#ax.annotate(r'q_2', (p2[0]+offset,offset))
#ax.annotate(r'q_3', (p3[0]-5*offset,2*offset))
#plt.plot(p1[0],0,'o',color='black',markersize=10)
#plt.plot(p2[0],0,'o',color='black',markersize=10)
#plt.plot(p3[0],0,'o',color='black',markersize=10)
#
#plt.axvline(p1[0], color='k', linestyle='dashed', linewidth=1)
#plt.axvline(p2[0], color='k', linestyle='dashed', linewidth=1)
#plt.axvline(p3[0], color='k', linestyle='dashed', linewidth=1)
#plt.axhline(0, color='k', linewidth=1)
#
#plotCSpaceDelaunayGrey(Q1,Q2)
#plt.savefig("2dof_cspace_M0.png", bbox_inches='tight')
#############################################################
#plt.show()
