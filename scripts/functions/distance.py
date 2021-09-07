import numpy as np
import matplotlib.pyplot as plt

print("Smooth valued function on [-infty, 0]")
t = np.linspace(0,5,100)

dcutoff = 1
damping = 10

def eval(x):
  if x > dcutoff:
    return 0
  else:
    return (-x*10 / (1 + abs(x*10)) + 1) - (-dcutoff*10 / (1 + abs(dcutoff*10)) + 1)

# X1 = (-t*10 / (1 + abs(t*10)) + 1)
X1 = np.array(list(map(eval, t)))



plt.plot(t,X1)
plt.show()
