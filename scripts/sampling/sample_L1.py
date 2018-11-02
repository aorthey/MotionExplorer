import numpy as np
import matplotlib.pyplot as plt

radius = 1
K=200 #samples
M=200 #points to render boundary

## random samples in outer boundary
r1 = np.random.uniform(-radius,radius,K)
r2 = np.random.uniform(-radius,radius,K)

xr=np.linspace(-radius,radius,M)
yr=radius-abs(xr)

plt.plot(xr,yr,"-k")
plt.plot(xr,-yr,"-k")
plt.scatter(r1,r2)
plt.show()

