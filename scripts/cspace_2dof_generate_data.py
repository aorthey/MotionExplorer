import sys
import numpy as np
from cspace_visualizer import *
from cspace_system_2dof import *

N = 100

q1 = np.linspace(-np.pi,np.pi,N)
q2 = np.linspace(-np.pi,np.pi,N)

P1 = []
P2 = []
for i in range(q1.shape[0]):
  for j in range(q2.shape[0]):
    q = np.array((q1[i],q2[j]))
    if not IsFeasible(q):
      P1 = np.append(P1,q1[i])
      P2 = np.append(P2,q2[j])

np.save('2dof_vertices_100.npy', (P1,P2))
