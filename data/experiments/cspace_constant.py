import numpy as np

def SpecialEuclideanNeighborhood(rmax):
  r1 = 1
  r2 = rmax
  rmin = np.min((r1, r2))
  rmax = np.max((r1, r2))
  k1 = 1.0/rmin
  k2 = 1.0/rmax
  c= k2*np.cos(0.5*np.pi - np.arctan(k1/k2))
  return c

print "PlanarLshape:",SpecialEuclideanNeighborhood(1.58902485821)
print "Xshape:",SpecialEuclideanNeighborhood(1.70073513517)
