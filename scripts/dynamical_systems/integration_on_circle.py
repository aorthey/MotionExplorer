import matplotlib.pyplot as plt
import numpy as np

#Point Robot on Circle

## Constant velocity integration
v = 1.3
dt_step = 0.01
M_steps = 100
x0 = np.complex(1,0)

X=[]
dt = dt_step
xC = x0
xE = x0
for k in range(0,M_steps):
  ## integration by conjugation or by exponentiation
  xC = np.exp(xC)*np.exp(np.complex(0,dt*v))*np.exp(-xC)
  xE = np.exp(np.complex(0,dt*v))*x0
  x = xC
  print "error:",np.abs(xE-xC)
  dt += dt_step
  X.append(x)

X = np.array(X)
T = np.linspace(0,2*np.pi,100)
plt.plot(np.cos(T), np.sin(T), '-b')
plt.plot(X.real, X.imag, 'ok')
plt.show()


