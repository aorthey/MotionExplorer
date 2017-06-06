import matplotlib.pyplot as plt
from sklearn.decomposition import FastICA, PCA
import sys
import numpy as np
from cvxpy import *

from forcefieldgenerator import *
from basisfunctions import *

L = 0
U = 1
padding = 0.1
dt = 0.01
Nth = 7

################################################################################
################################################################################
## uniform force
Fu = np.array([0.2,0.3])

## radial force
frs = np.array([0.5,0.5])
frp = -0.1

################################################################################
### generate ground truth: Fx,Fy are the true forces which we need to estimate
################################################################################
x = y = np.arange(L,U,dt)
xx, yy = np.meshgrid(x, y)

FUx,FUy = UniformForce(xx,yy,Fu)
FRx,FRy = RadialForce(xx,yy,frs,frp)

Fx = FUx + FRx
Fy = FUy + FRy

xr = np.reshape(xx,xx.shape[0]*xx.shape[1],[])
yr = np.reshape(yy,yy.shape[0]*yy.shape[1],[])
Fxr = np.reshape(Fx,Fx.shape[0]*Fx.shape[1],[])
Fyr = np.reshape(Fy,Fy.shape[0]*Fy.shape[1],[])

################################################################################
#### generate samples (x,y,Fx,Fy)
################################################################################
Mth = 10
Fsamples = np.dstack([xr[::Mth],yr[::Mth],Fxr[::Mth],Fyr[::Mth]])[0]

################################################################################
#### 
################################################################################

Phi = GetBasisFunctionsAtSamples( Fsamples )
print Phi

N = Phi.shape[0]
W = Variable(N)
C=1e-3
loss = Phi.T*W
constr = [W>0,np.ones(N).T*W == 1]
prob = Problem(Minimize(loss), constr)
prob.solve()

np.array(np.sort(W.value.T)[0])[0][-3:]

################################################################################
#### plot field
################################################################################


plt.figure()
plt.title("pivot='mid'; every third arrow; units='inches'")
Q = plt.quiver(xx[::Nth, ::Nth], yy[::Nth, ::Nth], Fx[::Nth, ::Nth], Fy[::Nth, ::Nth], pivot='mid', units='inches')
plt.scatter(xx[::Nth, ::Nth], yy[::Nth, ::Nth], color='r', s=5)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Force Field Estimation')

axes = plt.gca()
axes.set_xlim([L-padding, U+padding])
axes.set_ylim([L-padding, U+padding])


#plt.grid(True)
#plt.savefig("test.png")
plt.show()
