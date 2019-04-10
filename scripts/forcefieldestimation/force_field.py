import matplotlib.pyplot as plt
import numpy as np

class ForceField:

  def __init__(self):
    dt = 0.01
    x = y = np.arange(0,1,dt)
    self.xx, self.yy = np.meshgrid(x, y)
    self.Fx, self.Fy = self.GetForceAt(self.xx, self.yy)

  def Draw(self):
    Nth = 7
    Q = plt.quiver(self.xx[::Nth, ::Nth], self.yy[::Nth, ::Nth], self.Fx[::Nth, ::Nth], self.Fy[::Nth, ::Nth], pivot='mid', units='inches', color='grey')
    plt.scatter(self.xx[::Nth, ::Nth], self.yy[::Nth, ::Nth], color='r', s=5)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Force Field')

  def UniformForce(self, xx, yy, Fu):
    Fx= Fu[0]*np.ones(xx.shape)
    Fy= Fu[1]*np.ones(yy.shape)
    return [Fx,Fy]

  def RadialForce(self, xx, yy, center, power):
    r = ((xx-center[0])**2 + (yy-center[1])**2)# + (zz-center[2])**2)
    #(xx-frs[0])/((xx-center[0])**2 + (yy-center[1])**2)
    #(yy-frs[1])/((xx-center[0])**2 + (yy-center[1])**2)
    Fx = power*(xx-center[0])/r
    Fy = power*(yy-center[1])/r
    epsilon = 0.01
    if type(Fx) is np.ndarray:
      Fx[r<epsilon] = 0
      Fy[r<epsilon] = 0
    return [Fx,Fy]

  def GetForceAt(self, xx, yy):
    ################################################################################
    ## uniform force
    fu = np.array([0.2,0.3])
    ## radial force
    frs = np.array([0.5,0.5])
    frp = -0.1
    ################################################################################
    FUx,FUy = self.UniformForce(xx, yy, fu)
    FRx,FRy = self.RadialForce(xx, yy, frs,frp)

    Fx = FUx + FRx
    Fy = FUy + FRy
    return Fx, Fy

if __name__ == '__main__':
  F = ForceField()
  F.Draw()
  plt.show()

