import sys
import numpy as np
import math
from cspace_visualizer import *
from cspace_system_2dof import *
import cspace_system_2dof as dof

grey = '0.6'
font_size = 45
lim=5.0

def getPoints(fname, maxElements = float('inf')):
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


    #### change here depending on which values you get
    x = state[2]
    if len(state)>5:
      y = state[3]
      theta = state[4]
    else:
      y = 0
      theta = 0
    if feasible=='yes':
      feasible = True
    else:
      feasible = False
    if sufficient=='yes':
      sufficient = True
    else:
      sufficient = False
    q = list()
    q.append(x)
    q.append(y)
    q.append(theta)
    q.append(feasible)
    q.append(sufficient)
    q.append(open_ball_radius)
    Q.append(q)
    ctr += 1
  return Q

def generateCSpaceDense(fname, maxElements = float('inf')):
  Q = getPoints(fname)
  P1 = []
  P2 = []
  ctr = 0
  for q in Q:
    x = q[0]
    t = q[2]
    feasible = q[3]
    if not feasible:
      ctr +=1
      P1 = np.append(P1,x)
      P2 = np.append(P2,t)
    if ctr > maxElements:
      break
  np.save('tmp_CS_dense_1',P1)
  np.save('tmp_CS_dense_2',P2)

def generateQuotientSpaceDense(fname, maxElements = float('inf')):
  Q = getPoints(fname)
  theta = np.linspace(-np.pi,np.pi,100)
  P1 = []
  P2 = []
  ctr = 0
  for q in Q:
    x = q[0]
    feasible = q[3]
    if not feasible:
      ctr += 1
      for j in range(theta.shape[0]):
        P1 = np.append(P1,x)
        P2 = np.append(P2,theta[j])
    if ctr > maxElements:
      break
  np.save('tmp_QS_dense_1',P1)
  np.save('tmp_QS_dense_2',P2)

# fname = "../build/qs_samples.roadmap"
# plotQSPoints(fname)

qfname = "../build/qs_dense.roadmap"
generateQuotientSpaceDense(qfname)
cfname = "../build/cs_dense.roadmap"
generateCSpaceDense(cfname)
