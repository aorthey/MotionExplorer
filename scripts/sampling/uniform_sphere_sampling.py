import numpy as np
import matplotlib.pyplot as plt

#n: dimension
#M: number of samples
n = 2
M = 100
radius=1.45

p_metric = 2

X = np.zeros(n*M)
for j in range(0,M):
  Xk = np.random.uniform(-1,1,n)

  if p_metric == 1:
    d = np.sum(np.abs(Xk))
  else:
    d = np.sqrt(np.sum(Xk**2))

  X[j*n:(j+1)*n] = radius*Xk/d

X = np.reshape(X,(M,n))
plt.plot(X[:,0],X[:,1],'ok')

t = np.linspace(-np.pi,+np.pi,100)
plt.plot(radius*np.cos(t),radius*np.sin(t),'-k')
plt.show()
