import matplotlib.pyplot as plt
from cvxpy import *
from math import pi
import cvxpy as cvx
import sys
import numpy as np

def Fpoly(x,K): 
        return np.array(map(lambda e: map(lambda x: pow(x,e), x),np.arange(0,K)))
def dFpoly(x,K): 
        A= np.array(map(lambda e: map(lambda x: pow(x,e-1)/e, x),np.arange(1,K)))
        O = np.zeros((1,len(x)))
        return np.vstack((O,A))

K = 105
M = 100

N=3
X=cvx.Variable((3,M))
constraints = []

##### FUNC SPACE CONSTRAINTS
t = np.linspace(0,1,M)
F = Fpoly(t,K)
#dF = dFpoly(t,K)
W = cvx.Variable((K,3))

if M >= K:
  print "functional space not big enough",K,"<",M
  sys.exit(0)

for i in range(0,M):
  constraints.append(X[:,i] == W.T*F[:,i])

##### INIT/GOAL CONSTRAINTS
XI=np.array((-1,-1,0))
XG=np.array((1.5,1.5,pi))
constraints.append(X[:,0] == XI)
constraints.append(X[:,-1] == XG)

##### DIST CONSTRAINTS on consecutive WPs
epsilon=np.array((0.2,0.2,0.1))
for i in range(0,M-1):
  #constraints.append(norm(X[:,i] - X[:,i+1]) <= epsilon)
  constraints.append(norm(W.T*F[:,i] - W.T*F[:,i+1]) <= epsilon)

##### BOX CONSTRAINTS on X,Y,T
constraints.append(X[0,:] <= 2)
constraints.append(X[0,:] >= -2)
constraints.append(X[1,:] <= 2)
constraints.append(X[1,:] >= -2)
constraints.append(X[2,:] <= 2*pi)
constraints.append(X[2,:] >= 0)

objective = Minimize(norm(W,1))
prob = Problem(objective, constraints)

#prob.solve(solver=SCS, use_indirect=True, eps=1e-2, verbose=True)
prob.solve(solver=SCS, eps=1e-3, verbose=True)

plt.plot(XI[0],XI[1],'o',markersize=10,color='k')
plt.plot(XG[0],XG[1],'o',markersize=10,color='k')
#plt.plot(X.value[0],X.value[1],marker='o', linestyle='--', color='g',linewidth=2.0)

XF = X.value[0]
YF = X.value[1]

plt.plot(XF,YF, '-', color='g', markersize=10)
plt.plot(XF,YF, '-', marker='o',color='r', markersize=2)

#plt.plot(X.value[0],X.value[1], linestyle='-', color='r',linewidth=100)
#plt.plot(xf,yf,linestyle='-', color='r',linewidth=5.0)
plt.show()


########### SHOW distribution of weights on functional space
#B = []
#for i in range(0,M):
#        B.append(norm(W[i,:]).value)
#
#plt.figure(2)
#
#n, bins, patches = plt.hist(B, K, facecolor='green', alpha=0.75)
