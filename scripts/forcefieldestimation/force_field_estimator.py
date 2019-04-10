import matplotlib.pyplot as plt
import numpy as np
import cvxpy as cvx
from basisfunctions import *

class ForceFieldEstimator:

  def __init__(self, X_samples, F_samples):
    self.Phi = GetBasisFunctionsAtSamples(X_samples)

    N = self.Phi.shape[0]
    W = cvx.Variable(N)
    print self.Phi.shape
    C=1e-3
    obj = cvx.Minimize(cvx.norm(self.Phi.T*W))
    constr = [W>=0, np.ones(N).T*W == 1]
    prob = cvx.Problem(obj, constr)
    prob.solve()

    self.W = W.value

  def Draw(self):
    dt = 0.01
    x = y = np.arange(0,1,dt)
    self.xx, self.yy = np.meshgrid(x, y)
    self.Fx, self.Fy = self.GetForceAt(self.xx, self.yy)
    Nth = 1
    print self.xx
    print self.Fx
    Q = plt.quiver(self.xx[::Nth, ::Nth], self.yy[::Nth, ::Nth], self.Fx[::Nth, ::Nth], self.Fy[::Nth, ::Nth], pivot='mid', units='inches', color='grey')
    plt.scatter(self.xx[::Nth, ::Nth], self.yy[::Nth, ::Nth], color='r', s=5)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Force Field Estimator')

  def GetForceAt(self, xx, yy):
    ################################################################################
    xx = np.reshape(xx, xx.shape[0]*xx.shape[1])
    yy = np.reshape(yy, yy.shape[0]*yy.shape[1])
    X = np.array(list(zip(xx,yy))).transpose()

    Phi = GetBasisFunctionsAtSamples(X)

    return Phi.T*self.W


if __name__ == '__main__':
  print "NYI"

