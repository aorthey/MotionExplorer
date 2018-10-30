### Ball_inscribed_L1norm.py
### find the largest ball around the origin which is lying inside an L1 norm in
### the form $\sum_i r_i \|x_i\| \leq d$. 

import numpy as np
import matplotlib.pyplot as plt

#r1/d * |x| + r2/d * |y| = 1
r1 = 1.3
r2 = 1.8
d = 1.8
x = np.linspace(-d/r1,d/r1,500)

## plot L1 norm ball of radius d
#|y| = (d - r1*|x|)/r2
y = (d - r1*abs(x))/r2
plt.plot(x,y,'-k')
y = -(d - r1*abs(x))/r2
plt.plot(x,y,'-k')

t = np.linspace(0,2*np.pi,100)

### outer radius sphere
plt.plot(d*np.cos(t),d*np.sin(t),'--k')
### smallest d/r_min sphere (upper bound for real sphere)
dprime = np.min((d/r1,d/r2))
plt.plot(dprime*np.cos(t),dprime*np.sin(t),'--b')
###  d*r_min/sqrt(2) (assuming that the diamond has equal sides) sphere (lower bound for real sphere)
dprime = np.min((d/r1,d/r2))/np.sqrt(2)
plt.plot(dprime*np.cos(t),dprime*np.sin(t),'--b')

#k1 = d/r1
#k2 = d/r2
k1 = 1.0/r1
k2 = 1.0/r2
ddprime = d*k2*np.cos(np.pi/2-np.arctan(k1/k2))
plt.plot(ddprime*np.cos(t),ddprime*np.sin(t),'-r')

plt.plot([0,d/r1],[0,0],'-k')
plt.plot([0,0],[0,d/r2],'-k')
plt.axis('equal')
plt.show()
