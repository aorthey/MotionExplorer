import numpy as np
import matplotlib.pyplot as plt


#r1/d * |x| + r2/d * |y| = 1
r1 = 1
r2 = 2.8
d = 1.8
x = np.linspace(-d/r1,d/r1,500)

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

k1 = d/r1
k2 = d/r2
ddprime = np.cos(np.pi/2-np.arctan(k1/k2))*k2
plt.plot(ddprime*np.cos(t),ddprime*np.sin(t),'-r')

plt.plot([0,d/r1],[0,0],'-k')
plt.plot([0,0],[0,d/r2],'-k')
plt.axis('equal')
plt.show()
