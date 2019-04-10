import numpy as np
from itertools import product

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C

def f(x):
  return np.array([2*x[0],1])

X = np.array(([0.1,0.3],[0.4,0.5],[0.3,0.4],[0.7,0.34])).T
#X = np.array([param1, param2]).T
Y = f(X)[0]

x1 = np.linspace(0,1,10)
x2 = np.linspace(0,1,10)
x = (np.array([x1, x2])).T

kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2))
gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9)
gp.fit(X.T, Y)
y_pred, sigma = gp.predict(x, return_std=True)

# x1x2 = np.array(list(product(x1, x2)))
# y_pred, sigma = gp.predict(x1x2, return_std=True)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plt.plot(X[0,:], X[1,:], Y, 'or')
#plt.plot(X, y, 'r.', markersize=10, label=u'Observations')
plt.plot(x1, x2, y_pred, 'b-', label=u'Prediction')

# plt.fill(np.concatenate([x, x[::-1]]),
#          np.concatenate([y_pred - 1.9600 * sigma,
#                         (y_pred + 1.9600 * sigma)[::-1]]),
#          alpha=.5, fc='b', ec='None', label='95% confidence interval')
# plt.xlabel('$x$')
# plt.ylabel('$f(x)$')
# plt.ylim(-10, 20)
plt.show()
