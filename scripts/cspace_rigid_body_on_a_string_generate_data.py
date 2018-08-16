import sys
import numpy as np
from cspace_visualizer import *
from cspace_system_2dof import *
import cspace_system_2dof as dof

grey = '0.6'
font_size = 45
lim=5.0

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

def generateCSpaceDense(fname):
  Q = getPoints(fname)
  P1 = []
  P2 = []
  for q in Q:
    x = q[0]
    t = q[2]
    feasible = q[3]
    if not feasible:
      P1 = np.append(P1,x)
      P2 = np.append(P2,t)
  np.save('tmp_rigid_body_on_a_string_CS_dense_1',P1)
  np.save('tmp_rigid_body_on_a_string_CS_dense_2',P2)

def generateQuotientSpaceDense(fname):
  Q = getPoints(fname)
  theta = np.linspace(-np.pi,np.pi,100)
  P1 = []
  P2 = []
  for q in Q:
    x = q[0]
    feasible = q[3]
    if not feasible:
      for j in range(theta.shape[0]):
        P1 = np.append(P1,x)
        P2 = np.append(P2,theta[j])
  np.save('tmp_rigid_body_on_a_string_QS_dense_1',P1)
  np.save('tmp_rigid_body_on_a_string_QS_dense_2',P2)

# fname = "../build/qs_samples.roadmap"
# plotQSPoints(fname)

qfname = "../build/qs_dense.roadmap"
generateQuotientSpaceDense(qfname)
cfname = "../build/cs_dense.roadmap"
generateCSpaceDense(cfname)
