import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.7
L2 = 1.2
thicknessx = 0.2
thicknessy = 1.5

robot_name = 'Lshape/Lshape'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')
hstr  = createCuboid("link1",0,0,0,L1,thicknessy,thicknessx)
#hstr += createCuboid("link2",-L1/2,0,thickness/2+L2/2,thickness,thickness,L2)
hstr += createCuboid("link2",-L1/2+thicknessx/2,0,L2/2+thicknessx/2,thicknessx,thicknessy,L2)
hstr += createRigidJoint( "link1", "link2")
f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
CreateSphereRobot(robot_name + "_sphere_inner", thicknessx/2)
d = np.sqrt((L1/2)**2+L2**2)
CreateSphereRobot(robot_name + "_sphere_outer", d)

CreateCylinderRobot(robot_name + "_capsule_inner", thicknessx/2, L1)

CreateCylinderRobot(robot_name + "_capsule_outer", L2+thicknessx/2, L1)
