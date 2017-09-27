import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.0
L2 = 1.5
thickness1 = 0.4
thickness2 = 0.2

robot_name = 'Lshape'
folder='Lshape/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')
hstr  = createCuboid("link1",0,0,0,L1,thickness1,thickness1)
#hstr += createCuboid("link2",-L1/2,0,thickness/2+L2/2,thickness,thickness,L2)
hstr += createCuboid("link2",-L1/2+thickness1/2,0,L2/2+thickness1/2,thickness2,thickness2,L2)
hstr += createRigidJoint("joint_"+"l1"+"_"+"l2", "link1", "link2")
f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
CreateSphereRobot(folder, robot_name + "_sphere", thickness1/2)
CreateCylinderRobot(folder, robot_name + "_capsule", thickness1/2, L1)

