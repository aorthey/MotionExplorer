from __future__ import division, print_function

import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import sys

def lqr(A,B,Q,R):
    """Solve the continuous time lqr controller.

    dx/dt = A x + B u

    cost = integral x.T*Q*x + u.T*R*u
    """
    #ref Bertsekas, p.151
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))

    eigVals, eigVecs = scipy.linalg.eig(A-B*K)

    return K, X, eigVals


#def dlqr(A,B,Q,R):
#    """Solve the discrete time lqr controller.

#    x[k+1] = A x[k] + B u[k]

#    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
#    """
#    #ref Bertsekas, p.151

#    #first, try to solve the ricatti equation
#    X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

#    #compute the LQR gain
#    K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))

#    eigVals, eigVecs = scipy.linalg.eig(A-B*K)

#    return K, X, eigVals


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
      v = -np.array(np.dot(K,x-xdes))[0]
      u = np.dot(M,v)
      print(u)
      dx = np.dot(A,x) + np.dot(B,u)
      x = dx*dt + x
      t = t+dt
      plotState(x)

#Steer system from xI to xG
xI = np.array([-2, -2, 0, 5, -3, 0])
xG = np.array([0, +1, 2, 0, 0, 0])

M = np.eye(3)

A = np.zeros([6,6])
A[:3,3:] = np.eye(3)
# B = np.zeros([6,3])
# B[3:,:] = np.eye(3)
# B[4,1] = 0
B = np.zeros([6,3])
B[3:,:] = np.eye(3)

Q = np.eye(6)
R = np.eye(3)

# dx/dt = A x + B u
# cost = integral x.T*Q*x + u.T*R*u

fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax  = fig.gca()

plt.plot(0,0,'ko', markersize=5)
plotState(xI)
plotState(xG)

plotLQR(A,B,Q,R,xI, xG)

plt.show()

