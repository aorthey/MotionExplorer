import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import sys

def SampleShell(center, radius_inner, radius_outer):
  n = center.shape[0]
  x = np.random.uniform(-1,1,n) #random direction
  d = np.sqrt(np.sum(x**2)) #scale to unit direction

  ## get random radius
  u = np.random.uniform(0,1)
  # ro = radius_outer * radius_outer;
  # ri = radius_inner * radius_inner;

  r = u*pow(radius_outer, n) + (1-u)*pow(radius_inner, n)
  R = pow(r, (1/n))
  # r = np.sqrt(u*(ro-ri)+ri)
  return (R/d)*x

#n: dimension
#M: number of samples
n = 3
M = 10000
radius_outer =4.5
radius_inner =3.0

X = np.zeros(n*M)
for j in range(0,M):
  X[j*n:(j+1)*n] = SampleShell(np.zeros((n,1)), radius_inner, radius_outer)

X = np.reshape(X,(M,n))

if n==2:
  plt.plot(X[:,0],X[:,1],'ok')
  t = np.linspace(-np.pi,+np.pi,100)
  plt.plot(radius_inner*np.cos(t),radius_inner*np.sin(t),'-k')
  plt.plot(radius_outer*np.cos(t),radius_outer*np.sin(t),'-k')
elif n==3:
  ax = plt.axes(projection='3d')
  N = 16
  halfspace = (X[:,0] > 0)
  plt.plot(X[halfspace,0],X[halfspace,1],X[halfspace,2],'ok')
  theta = np.linspace(-0.5*np.pi,+0.5*np.pi,N)
  for k in range(0,N):
    phi = (k/N)*np.pi;
    plt.plot(radius_inner*np.cos(theta)*np.sin(phi),
        radius_inner*np.sin(theta)*np.sin(phi),radius_inner*np.cos(phi),'-k')
    plt.plot(radius_outer*np.cos(theta)*np.sin(phi),
        radius_outer*np.sin(theta)*np.sin(phi),radius_outer*np.cos(phi),'-k')

plt.show()
