import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.5
thickness = 0.2

robot_name = 'DoubleLshape'
folder='Lshape/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr  = createCuboid("link1",0,0,0,L1,thickness,thickness)
hstr += createCuboid("link2",-L1/2+thickness/2,0,L1/2+thickness/2,thickness,thickness,L1)

hstr += createRigidJoint("joint_"+"l1"+"_"+"l2", "link1", "link2")
hstr += createCuboid("link3",L1/2-thickness/2,L1/2+thickness/2,0,thickness,L1,thickness)

hstr += createRigidJoint("joint_"+"l1"+"_"+"l3", "link1", "link3")

hstr += createCuboid("link4",L1/2-thickness/2,L1,-L1/2-thickness/2,thickness,thickness,L1)

hstr += createRigidJoint("joint_"+"l3"+"_"+"l4", "link3", "link4")

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

CreateCylinderRobot(folder, robot_name + "_capsule_inner", thickness/2, L1)

CreateCylinderRobot(folder, robot_name + "_capsule_outer", L1+thickness/2, L1)

