import numpy as np
def UniformForce(xx, yy, Fu):
  Fx= Fu[0]*np.ones(xx.shape)
  Fy= Fu[1]*np.ones(xx.shape)
  return [Fx,Fy]

def RadialForce( xx, yy, center, power):
  r = ((xx-center[0])**2 + (yy-center[1])**2)# + (zz-center[2])**2)
  #(xx-frs[0])/((xx-center[0])**2 + (yy-center[1])**2)
  #(yy-frs[1])/((xx-center[0])**2 + (yy-center[1])**2)
  Fx = power*(xx-center[0])/r
  Fy = power*(yy-center[1])/r
  epsilon = 0.01
  Fx[r<epsilon] = 0
  Fy[r<epsilon] = 0
  #sys.exit(0)
  return [Fx,Fy]

