import numpy as np
import matplotlib.pyplot as plt

print "Smooth valued functions on [0,1]"
t = np.linspace(0,1,100)


X1 = np.exp(-(t**2)/2)


plt.plot(t,X1)
plt.show()
