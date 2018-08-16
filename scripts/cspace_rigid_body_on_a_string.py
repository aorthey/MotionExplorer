import sys
import numpy as np
from cspace_visualizer import *
from cspace_system_2dof import *
import cspace_system_2dof as dof

grey = '0.6'
font_size = 45
lim=5.0
showSamples = False

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
    q.append(state[4])
    q.append(feasible)
    q.append(sufficient)
    Q.append(q)
  return Q

def plotQSPoints(fname):
    
  Q = getPoints(fname)
  for q in Q:
    x = float(q[0])
    y = float(q[1])
    feasible = q[3]
    sufficient = q[4]
    if not feasible:
      plt.axvline(x, color='k', linewidth=1)
    elif sufficient:
      plt.axvline(x, color=grey, linewidth=1)
    else:
      delta=0.05
      plt.plot([x,x],[-delta,+delta],'-',color=grey, linewidth=1)

def plotCSPoints(fname):
    
  Q = getPoints(fname)
  for q in Q:
    x = float(q[0])
    t = float(q[2])
    feasible = q[3]
    if feasible:
      plt.plot(x,t,'o',color=grey, linewidth=1)
    else:
      plt.plot(x,t,'x',color='k', linewidth=1)

def plotQuotientSpaceBackground():
  P1 = np.load(open(r'tmp_rigid_body_on_a_string_QS_dense_1.npy', 'rb'))
  P2 = np.load(open(r'tmp_rigid_body_on_a_string_QS_dense_2.npy', 'rb'))
  plotCSpaceDelaunayGrey(P1,P2,0.15)

def plotCSpaceBackground():
  P1 = np.load(open(r'tmp_rigid_body_on_a_string_CS_dense_1.npy', 'rb'))
  P2 = np.load(open(r'tmp_rigid_body_on_a_string_CS_dense_2.npy', 'rb'))
  plotCSpaceDelaunayGrey(P1,P2,0.15)


fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'x',fontsize=font_size)
ax.set_ylabel(r'\theta',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=15)
plt.axis([-lim,lim,-3.14,3.14])
if showSamples:
  fname = "../build/qs_samples.roadmap"
  plotQSPoints(fname)
plotQuotientSpaceBackground()

fig = plt.figure(1)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'x',fontsize=font_size)
ax.set_ylabel(r'\theta',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=15)
plt.axis([-lim,lim,-3.14,3.14])
if showSamples:
  fname = "../build/cs_samples.roadmap"
  plotCSPoints(fname)
plotCSpaceBackground()
plt.show()
