import numpy as np

# constant is computed by taking the largest sphere around the robot (rmax), and
# then computing how the inscribed ball changes

# The neighborhood as defined by Quinlan_1994 is actually an L1 neighborhood,
# i.e. it is diamond-shaped. We search here for the largest inscribed ball in
# this neighborhood.

def SpecialEuclideanNeighborhood(rmax):
  r1 = 1
  r2 = rmax
  rmin = np.min((r1, r2))
  rmax = np.max((r1, r2))
  k1 = 1.0/rmin
  k2 = 1.0/rmax
  c= k2*np.cos(0.5*np.pi - np.arctan(k1/k2))
  return c

print "03D_misleading (PlanarLshape)    :",SpecialEuclideanNeighborhood(1.58902485821)
print "03D_nonsimple (PlanarRectangle)  :",SpecialEuclideanNeighborhood(1.05948)
print "06D_misleading_Xshape            :",SpecialEuclideanNeighborhood(1.70073513517)
print "06D_doubleLshape                 :",SpecialEuclideanNeighborhood(1.677)
print "10D_snake (the SE(3) part)       :",SpecialEuclideanNeighborhood(0.169705627485)
