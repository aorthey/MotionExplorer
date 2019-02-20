import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

thicknessx = 0.2
thicknessy = 1.7
thicknessz = 3.0

robot_name = 'card/card'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr   = createCuboid("base", 0,                            0, 0,                         thicknessx, thicknessy, thicknessz)
hstr  += createCylinder("head"  ,0, 0, thicknessz/2, thicknessy/2, thicknessx)
hstr  += createRigidJoint("head", "base")

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
CreateSphereRobot(robot_name + "_sphere_inner", thicknessx/2)

p = max(max(thicknessz,thicknessx),thicknessy)
D = np.sqrt( (0.5*p)**2 + p**2)

CreateSphereRobot(robot_name + "_sphere_outer", D)

#CreateCylinderRobot(robot_name + "_capsule_inner", thicknessx/2, thicknessz)
#CreateCylinderRobot(robot_name + "_capsule_outer", L1+thicknessx/2, thicknessz)
