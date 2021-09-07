import numpy as np
import matplotlib.pyplot as plt
import sys

t0=+0
t1=+2.5

x0 = 1
x1 = 0

t = np.linspace(t0,t1,100)

if t0 > t1:
  print("Not allowed")
  sys.exit(0)

def function(t):
  x = (t-t0)/(t1-t0)

  if x < 0.0:
    return x0
  if x >= 1.0:
    return x1

  return x0 + (x1-x0)*(x*x*(3-2*x))

plt.axvline(t0, x0, x1, ls='--', c='k')
plt.axvline(t1, x0, x1, ls='--', c='k')
plt.plot(t,list(map(function, t)))
plt.show()
