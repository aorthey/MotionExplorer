import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.mplot3d import Axes3D
import os, os.path
from pdf2image import convert_from_path


from cvxpy import *
from math import pi
import cvxpy as cvx
import sys
import numpy as np

def Fpoly(x,K): 
  return np.array(map(lambda e: map(lambda x: pow(x,e), x),np.arange(0,K)))

N_dimension = 3
M_samples = 30
K_basis_functions = 10
s = np.array([0.5,0.5,0.5])

################################################################################
##simulate random walk
################################################################################
S = []
Y_force = []

def GetRadialForceAt(X, x_center, radius):
  F= map(lambda k: np.sign(X[:,k]-x_center)*1.0/radius*pow((X[:,k]-x_center),2),np.arange(0,X.shape[1]))
  F = np.array(F)
  if F.shape[0]<=1:
    F = F[0,:]
  else:
    F = F.T
  return F

def GetUniformForceAt(X, x_direction):
  F = map(lambda k: x_direction, np.arange(0,X.shape[1]))
  F = np.array(F)
  if F.shape[0]<=1:
    F = F[0,:]
  else:
    F = F.T
  return F

def GetForceAt(X):
  x_center = np.array([0.3,0.2,0.2])
  radius = 10
  F = GetRadialForceAt(X, x_center, radius)
  x_direction = 0.1*np.array([0.07,-0.02,0.03])
  F += GetUniformForceAt(X, x_direction)

  return F

for k in range(0,M_samples):
  s = s.reshape(N_dimension,-1)
  sF = GetForceAt(s)
  S.append(s[:,0])
  Y_force.append(sF)
  s = np.random.normal(s,0.05)

X_random_walk = np.array(S).T
Y_random_walk = np.array(Y_force).T

################################################################################
##### FUNC SPACE CONSTRAINTS
################################################################################
Phi = Fpoly(X_random_walk,K_basis_functions)
W = cvx.Variable((K_basis_functions,N_dimension))

epsilon=np.full((N_dimension),1e-1)
constraints = []
cost = norm(W,1)
for k in range(0,M_samples):
  cost += norm(Y_random_walk[:,k] - cvx.diag(W.T*Phi[:,:,k]))

obj = Minimize(cost)
#norm(Y[:,k] - cvx.diag(W.T*F[:,:,k])) + norm(W,1))
prob = Problem(obj, constraints)
prob.solve(solver=SCS, use_indirect=True, eps=1e-2, verbose=True)
#prob.solve(solver=SCS, eps=1e-3, verbose=True)
W = W.value

################################################################################
###EVALUATE
################################################################################
dt = 0.1
x = y = np.arange(0,1,dt)
xx, yy, zz = np.meshgrid(x, x, x)
X_test = np.array((zip(xx.flatten(),yy.flatten(),zz.flatten()))).T
Phi_test = Fpoly(X_test,K_basis_functions)

Y_estimate = []
for k in range(0, Phi_test.shape[2]):
  Fk = np.diag(np.dot(W.T,Phi_test[:,:,k]))
  Y_estimate.append(Fk)

Y_estimate = np.array(Y_estimate).T
Y_truth = GetForceAt(X_test)

################################################################################
##### PLOT
################################################################################

DIR = 'images/'
N_files = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])

fname_pdf = DIR+"force_field_estimator_%d.pdf"%(N_files)
pp = PdfPages(fname_pdf)

fig = plt.figure(0)
ax = fig.gca(projection='3d')

plt.quiver(X_test[0,:],X_test[1,:],X_test[2,:],
    Y_estimate[0,:],Y_estimate[1,:], Y_estimate[2,:],
    color='black', label="Estimated")

plt.quiver(X_test[0,:],X_test[1,:],X_test[2,:],
    Y_truth[0,:],Y_truth[1,:], Y_truth[2,:],
    color='grey', label="Ground Truth")

plt.plot(X_random_walk[0,:],X_random_walk[1,:],X_random_walk[2,:],'-ko',ms=1)

plt.quiver(X_random_walk[0,:],X_random_walk[1,:],X_random_walk[2,:],
    Y_random_walk[0,:],Y_random_walk[1,:],Y_random_walk[2,:],
    color='red', label="Random Walk")

legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, +1.10),
          fancybox=True, shadow=True, ncol=5)

pp.savefig(plt.gcf())
pp.close()

pages = convert_from_path(fname_pdf)
for page in pages:
  page.save(fname_pdf+'.png', 'PNG')
plt.show()
plt.close()


################################################################################
########### SHOW distribution of weights on functional space
################################################################################
# B = []
# for i in range(0,K_basis_functions):
#   B.append(np.linalg.norm(W[i,:]))

# plt.figure(2)

# n, bins, patches = plt.hist(B, K_basis_functions, facecolor='green', alpha=0.75)
