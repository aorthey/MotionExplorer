import matplotlib.pyplot as plt
import numpy as np
import sys
from force_field import ForceField
from force_field_estimator import ForceFieldEstimator
from basisfunctions import *
import cvxpy as cvx


################################################################################
#### generate samples Fsamples = (x,y,Fx,Fy)
################################################################################
FF = ForceField()
FF.Draw()

##simulate random walk
s = np.array([0.5,0.5])
S = []
F = []
for k in range(0,5):
  s = np.random.normal(s,0.05)
  sF = FF.GetForceAt(s[0],s[1])
  plt.quiver(s[0],s[1],sF[0],sF[1])
  S.append(s)
  F.append(sF)

S = np.array(S)
F = np.array(F)

# print F,S
# plt.plot(S[:,0],S[:,1],'-or')
# plt.show()

X_samples = S.transpose()
F_samples = F.transpose()

################################################################################
################################################################################
Phi = GetBasisFunctionsAtSamples(X_samples)
N_dim = Phi.shape[0]
K_basis_functions = Phi.shape[1]
M_samples = Phi.shape[2]

##sum basis functions over each sample
#Phi = np.sum(Phi,axis=1) 

W = cvx.Variable((K_basis_functions, N_dim))

v = np.array((0.1,0.3,1))

C=1e-3
print F_samples
obj = cvx.Minimize(cvx.norm(F_samples - (Phi.T*W)))
constr = [W>=0, np.ones(N).T*W == 1]
prob = cvx.Problem(obj, constr)
prob.solve()

W = W.value
#FFE = ForceFieldEstimator(X_samples, F_samples)
#FFE.Draw()

# Phi = GetBasisFunctionsAtSamples(Fsamples)
# N = Phi.shape[0]
# W = cvx.Variable(N)
# C=1e-3
# obj = cvx.Minimize(cvx.norm(Phi.T*W))
# constr = [W>=0, np.ones(N).T*W == 1]
# prob = cvx.Problem(obj, constr)
# prob.solve()

# print W.value

