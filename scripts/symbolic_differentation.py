import sympy as sy
from sympy import cos,sin

u,v = sy.symbols('u v')
cu = sy.cos(u)
cv = sy.cos(v)
su = sy.sin(u)
sv = sy.sin(v)
cu2 = cu * cu
cu3 = cu * cu2
cu4 = cu * cu3
cu5 = cu * cu4
cu6 = cu * cu5
cu7 = cu * cu6

a = 3*cv - 30*su + 90*cu4*su-60*cu6*su+5*cu*cv*su
b = 3*cv - 3*cu2*cv - 48*cu4*cv + 48*cu6*cv - 60*su + 5*cu*cv*su - 5*cu3*cv*su - 80 * cu5*cv*su + 80*cu7*cv*su

x = -2.0/15.0*cu*a
y = -1.0/15.0*su*b
z = 2.0/15.0 * (3+5*cu*su) * sv

print("** dX/du:")

dxdu = sy.simplify(sy.diff(x, u))
dydu = sy.simplify(sy.diff(y, u))
dzdu = sy.simplify(sy.diff(z, u))

print(dxdu, dydu, dzdu)

print("** dX/dv:")

dxdv = sy.simplify(sy.diff(x, v))
dydv = sy.simplify(sy.diff(y, v))
dzdv = sy.simplify(sy.diff(z, v))

print(dxdv, dydv, dzdv)

dx = sy.simplify(dydu*dzdv - dydv*dzdu)
dy = sy.simplify(-(dxdu*dzdv - dzdu*dxdv))
dz = sy.simplify(dxdu*dydv - dydu*dxdv)

print("** dX/du ^ dX/dv:")
print(dx, dy, dz)

D = sy.sqrt(dx*dx + dy*dy+ dz*dz)
# D = sy.simplify(sy.sqrt(dx*dx + dy*dy+ dz*dz))
print(D)



# usign = 0
# if pi - u > 0:
#   usign = 1

# a1 = 6*cu2 - 2*sin(u)*(3 + cos(v) + 3 sin(u)) + 2 cos(v)*(sin(2*u) - sin(u))*usign
# a2 = 16*cos(u) - 2*(cos(2*u) - 2*cos(u))*cos(v)*usign
# a3 = 2*sin(u)*sin(v)

# b1 = 2*(cos(u) - 2)*sin(v)*((1 + cos(u))*usign - 1
# b2 = 2*(cos(u) - 2)*sin(u)*sin(v)*usign
# b3 = 2*(2 - cos(u))*cos(v)

# s1 = a2*b3 - a3*b2
# s2 = a3*b1 - a1*b3
# s3 = a1*b2 - a2*b1

#https://stackoverflow.com/questions/26300510/generating-random-points-on-a-surface-of-an-n-dimensional-torus
#https://math.stackexchange.com/questions/2842911/how-to-perform-wedge-product/2842962
