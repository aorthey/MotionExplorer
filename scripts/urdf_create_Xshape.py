import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.4
thicknessx = 0.3
thicknessy = L1
spacing = 0.001

robot_name = 'Xshape'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr  = createCuboid("link0", 0,                            0, 0,                         thicknessx, thicknessy, thicknessx)
hstr += createCuboid("link1", -L1/2-thicknessx/2-spacing,   0, 0,                         L1,         thicknessy, thicknessx)
hstr += createCuboid("link2", 0,                            0, L1/2+thicknessx/2+spacing, thicknessx, thicknessy, L1)
hstr += createCuboid("link3", 0,                            0, -L1/2-thicknessx/2-spacing,thicknessx, thicknessy, L1)
hstr += createCuboid("link4", L1/2+thicknessx/2+spacing,    0, 0,                         L1,         thicknessy, thicknessx)

hstr += createRigidJoint( "link0", "link1")
hstr += createRigidJoint( "link0", "link2")
hstr += createRigidJoint( "link0", "link3")
hstr += createRigidJoint( "link0", "link4")

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
t1 = min(thicknessx, thicknessy)
t2 = max(thicknessx, thicknessy)

CreateSphereRobot(folder, robot_name + "_sphere_inner", t1/2)

#D = np.sqrt( (0.5*p)**2 + p**2)
D = np.sqrt( (0.5*t2)**2 + (L1+0.5*t1)**2)

CreateSphereRobot(folder, robot_name + "_sphere_outer", D)

CreateCylinderRobot(folder, robot_name + "_capsule_inner", t1/2, L1)

CreateCylinderRobot(folder, robot_name + "_capsule_outer", L1+t1/2, L1)

