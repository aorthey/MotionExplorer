import numpy as np
import matplotlib.pyplot as plt

print("Smooth valued functions on [0,1]")

#constraints: f(t == 0) = 1, f(t >= d) = 0 

d = 0.5
t = np.linspace(0,1,100)


X1 = np.exp(-(t**2)/2)

x = t/d
def function(t):
  #in [t0, t1]

  t0 = 0.0
  t1 = 0.5
  # x = (t-t0)/(t1-t0)
  x = (t-t0)/(t1-t0)

  if x >= 1.0:
    return 0.0
  return 1.0 + (-1.0)*(x*x*(3-2*x))

plt.plot(t,X1)
plt.plot(t,list(map(function, t)))
plt.show()
