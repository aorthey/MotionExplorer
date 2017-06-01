import matplotlib.pyplot as plt
import numpy as np


L = 0
U = 1
padding = 0.1
dt = 0.01
Nth = 10

## uniform force
Fu = np.array([0,0])

## radial force
frs = np.array([0.3,0.5])
frp = -0.05

### generate ground truth: Fx,Fy are the true forces which we need to estimate
x = y = np.arange(L,U,dt)
xx, yy = np.meshgrid(x, y)
Fx= Fu[0]*np.ones(xx.shape)
Fy= Fu[1]*np.ones(xx.shape)
Fx = Fx + frp*(xx-frs[0])/((xx-frs[0])**2 + (yy-frs[1])**2)
Fy = Fy + frp*(yy-frs[1])/((xx-frs[0])**2 + (yy-frs[1])**2)
Fx[np.argwhere(np.isnan(Fx))] = 0
Fy[np.argwhere(np.isnan(Fy))] = 0

xr = np.reshape(xx,xx.shape[0]*xx.shape[1],[])
yr = np.reshape(yy,yy.shape[0]*yy.shape[1],[])
Fxr = np.reshape(Fx,Fx.shape[0]*Fx.shape[1],[])
Fyr = np.reshape(Fy,Fy.shape[0]*Fy.shape[1],[])

### generate samples (x,y,Fx,Fy)

Mth = 10
Fsamples = np.dstack([xr[::Mth],yr[::Mth],Fxr[::Mth],Fyr[::Mth]])[0]


#### radial basis functions
N = 5
ll=0
ul=1
sx = sy = np.linspace(ll, ul, N)
p = np.linspace(-1,1,5)

ssx,ssy,pp = np.meshgrid(sx,sy,p)

ssx= np.reshape(ssx,ssx.shape[0]*ssx.shape[1]*ssx.shape[2],[])
ssy= np.reshape(ssy,ssy.shape[0]*ssy.shape[1]*ssy.shape[2],[])
spp= np.reshape(pp,pp.shape[0]*pp.shape[1]*pp.shape[2],[])

Model = np.dstack([ssx,ssy,spp])[0]



K = Model.shape[0]
M = Fsamples.shape[0]

Theta = np.zeros([K,M,2])
for i in range(0,K):
  print i,"/",K
  for j in range(0,M):
    ix = Model[i,0]
    iy = Model[i,1]
    ip = Model[i,2]
    px = Fsamples[j,0]
    py = Fsamples[j,1]

    pp = np.array([px,py])
    ss = np.array([ix,iy])
  
    r = np.linalg.norm(pp-ss)

    if r > 0:
      Theta[i,j,:] = ip*(pp-ss)/r
    else:
      Theta[i,j,:] = 0

#### fit it
FF = Fsamples[:,2:4]

D = np.zeros([K])
W = np.zeros([K])

for i in range(0,K):
  D[i] = np.linalg.norm(FF - Theta[i,:,:])

idx = np.argmin(D)

print D
print Model[idx,:]


#
#
#
#
#plt.figure()
#plt.title("pivot='mid'; every third arrow; units='inches'")
#Q = plt.quiver(xx[::Nth, ::Nth], yy[::Nth, ::Nth], Fx[::Nth, ::Nth], Fy[::Nth, ::Nth], pivot='mid', units='width')
#plt.scatter(xx[::Nth, ::Nth], yy[::Nth, ::Nth], color='r', s=5)
#plt.xlabel('x')
#plt.ylabel('y')
#plt.title('Force Field Estimation')
#
#axes = plt.gca()
#axes.set_xlim([L-padding, U+padding])
#axes.set_ylim([L-padding, U+padding])
#
#
##plt.grid(True)
##plt.savefig("test.png")
#plt.show()
#
