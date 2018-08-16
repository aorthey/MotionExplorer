import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.2
thickness = 0.7
thickness_Z = 0.1

robot_name = 'PlanarLshape'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr  = createCuboid("link1",0,0,0,thickness,thickness,thickness_Z)
hstr += createCuboid("link2",0,L1/2+thickness/2,0,thickness,L1-1e-10,thickness_Z)
hstr += createCuboid("link3",L1/2+thickness/2,0,0,L1,thickness,thickness_Z)

hstr += createRigidJoint("joint_"+"l1"+"_"+"l2", "link1", "link2")
hstr += createRigidJoint("joint_"+"l1"+"_"+"l3", "link1", "link3")

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
d = np.sqrt((L1+thickness/2.0)**2+(thickness/2.0)**2)

## thickness_Z change for visibility in simulator (does not change the collisions)
CreateCylinderRobot(folder, robot_name + "_cylinder_inner", thickness/2, 2.0*thickness_Z, 1.57)
CreateCylinderRobot(folder, robot_name + "_cylinder_outer", d, thickness_Z/2.0, 1.57)
# CreateSphereRobot(folder, robot_name + "_sphere_inner", thickness/2)
# CreateSphereRobot(folder, robot_name + "_sphere_outer", d)
