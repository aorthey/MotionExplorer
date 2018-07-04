import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.8
thickness = 0.7
thickness_Z = 0.1

robot_name = 'PlanarLshape'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr  = createCuboid("link1",0,0,0,L1,thickness,thickness_Z)
hstr += createCuboid("link2",-L1/2+thickness/2,L1/2+thickness/2,0,thickness,L1,thickness_Z)

hstr += createRigidJoint("joint_"+"l1"+"_"+"l2", "link1", "link2")

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
CreateSphereRobot(folder, robot_name + "_sphere_inner", thickness/2)
d = np.sqrt((L1/2)**2+L1**2)
CreateSphereRobot(folder, robot_name + "_sphere_outer", d)
