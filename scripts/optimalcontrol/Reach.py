import numpy as np
import scipy.linalg
from scipy.linalg import expm, logm
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import sys
from scipy.spatial.transform import Rotation

def splitConfig(x):
  q = x[:6]
  dq = x[6:]
  return [q,dq]

def mergeConfig(q, dq):
  x = np.zeros(len(q)+len(dq))
  x[:len(q)] = q
  x[len(q):len(q)+len(dq)] = dq
  return x

def se3Derivative(u):
    X1 = np.zeros([4,4])
    X2 = np.zeros([4,4])
    X3 = np.zeros([4,4])
    X4 = np.zeros([4,4])
    X5 = np.zeros([4,4])
    X6 = np.zeros([4,4])

    X1[0,3] = 1
    ##########################
    X2[1,3] = 1
    #########################
    X3[2,3] = 1
    #########################
    X4[0,1] = -1
    X4[1,0] = 1
    #########################
    X5[0,2] = 1
    X5[2,0] = -1
    #########################
    X6[1,2] = -1
    X6[2,1] = 1

    dx = X1*u[0] + X2*u[1] + X3*u[2] + X4*u[3] + X5*u[4] + X6*u[5]
    return dx

def SE3ToConfig(q_SE3):
  x = np.zeros(6)
  x[:3] = q_SE3[0:3,3]
  R = Rotation.from_dcm(q_SE3[:3,:3])
  x[3:] = np.array(R.as_euler('zyx'))
  return x


def configToSE3(q):
    X = np.zeros([4,4])
    X[0,3] = q[0]
    X[1,3] = q[1]
    X[2,3] = q[2]
    X[3,3] = 1

    R = Rotation.from_euler('zyx', [q[3], q[4], q[5]])
    X[:3,:3] = R.as_dcm() #as_matrix in 1.4.0
    return X

def M(q):
  return np.eye(6)

def G(q, dq):
  return np.dot(np.linalg.inv(M(q)), np.eye(6))

def H(q, dq):
  return np.zeros(6)

## H(x) + G(x)*u
def EquationOfMotion(q, dq, u):
    ddq = H(q,dq) + np.dot(G(q,dq), u)
    return ddq

def drawPath(x, color):
  plt.plot(x[:,0],x[:,1],x[:,2],'bo-', color=color, markersize=3)

def integratePath(x, u, T, color='b'):
  dt = 0.1
  t = 0

  xpath = []
  while t < T:
    [q0,dq0] = splitConfig(x)

    ddq0 = EquationOfMotion(q0, dq0, u)
    dq1 = dq0 + dt*ddq0

    q0_SE3 = configToSE3(q0)
    dq0_SE3 = se3Derivative(dq0)

    q1_SE3 = np.dot(q0_SE3, expm(dt*dq0_SE3))
    q1 = SE3ToConfig(q1_SE3)

    x = mergeConfig(q1, dq1)
    xpath.append(x)

    t = t+dt

  xpath = np.array(xpath)
  drawPath(xpath, color=color)

fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax = fig.gca(projection='3d')

#Steer system from xI to xG
xI = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0])
umin = np.array([0, 0, 0, -0.5, -0.0, -0.0])
umax = np.array([0, 0, 0, +0.5, +0.0, +0.0])
du = umax - umin
T = 3


xG = np.array([+3, +0.765, 0, 0.3, 0, 0, 1, 0, 0, 0, 0, 0])

plt.plot([xG[0]],[xG[1]],[xG[2]],'go', markersize=8)

[q0,dq0] = splitConfig(xI)
[q1,dq1] = splitConfig(xG)
q0_SE3 = configToSE3(q0)
q1_SE3 = configToSE3(q1)

dq0_des = logm(np.dot(np.linalg.pinv(q0_SE3), q1_SE3))

# q1_SE3 = np.dot(q0_SE3, expm(dt*dq0_SE3))
# q1 = SE3ToConfig(q1_SE3)


# ## (1) can we do the change in dq0 to dq1 (relative in Tq0X) (velocity change
# ## possible)


for k in range(0,len(umin)):
  ukmin = np.zeros(6)
  ukmin[k] = umin[k]
  ukmax = np.zeros(6)
  ukmax[k] = umax[k]
  integratePath(xI, ukmin, T, color='r')
  integratePath(xI, ukmax, T, color='r')



# # for k in range(0,5):
# #   u = (np.multiply(du,np.random.rand(6,1).T) + umin).flatten()
# #   integratePath(xI, u, T)

plt.show()

