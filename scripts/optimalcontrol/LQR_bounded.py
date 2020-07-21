from __future__ import division, print_function

import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import sys

def lqr(A,B,Q,R):
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)

    return K, X, eigVals

def plotState(x):
  plt.plot(x[0],x[1],'bo', markersize=3)

def plotLQR(A, B, Q, R, xinit, xdes):
  ## use pseudovelocities

  [K,X,eigVals] = lqr(A,B,Q,R)
  dt = 0.1
  t = 0
  T = 10
  x = xinit
  while t < T:
      xdiff  = x-xdes
      v = -np.array(np.dot(K,xdiff))[0]
      u = np.dot(M,v)
      print(u)
      print("Value:", np.dot(xdiff, np.dot(Q,xdiff)))
      dx = np.dot(A,x) + np.dot(B,u)
      x = dx*dt + x
      t = t+dt
      plotState(x)


M = np.eye(3)
M[0,0]=10

A = np.zeros([6,6])
A[:3,3:] = np.eye(3)
B = np.zeros([6,3])
B[3::,:] = np.dot(np.linalg.inv(M), np.eye(3))
Q = np.eye(6)
R = np.eye(3)

# dx/dt = A x + B u
# cost = integral x.T*Q*x + u.T*R*u

## control affine system

## dx = -G(x) + M^-1*B*u

fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax  = fig.gca()

plt.plot(0,0,'ko', markersize=5)

#Steer system from xI to xG
xI = np.array([-5, 0, 0, 3, 0, 0])
plotState(xI)

xG = np.array([0, -1, 0, 0, 0, 0])
plotState(xG)
plotLQR(A, B, Q, R, xI, xG)

xG = np.array([0, 1, 0, 0, 0, 0])
plotState(xG)
plotLQR(A, B, Q, R, xI, xG)
plt.show()

