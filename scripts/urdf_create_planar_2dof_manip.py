import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.5
L2 = 1.5
thickness = 0.2
thickness_Z = 0.1

robot_name = '2dof_manip'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr  = createRotatedCylinder("jlink1",0,0,0,0,0,1.57,thickness,thickness_Z)
hstr += createCuboid("link1",0,L1/2+thickness,0,  2*thickness,L1,thickness_Z)
hstr += createRotatedCylinder("jlink2",0,L1+2*thickness,0,0,0,1.57,thickness,thickness_Z)
hstr += createCuboid("link2",0,L2/2+thickness,0,2*thickness,L2-1e-10,thickness_Z)

hstr += createRevoluteJoint("joint_"+"jlink1"+"_"+"link1", "jlink1", "link1",0,0,0, lowerLimit = -3.14, upperLimit = 3.14)
hstr += createRigidJoint("joint_"+"link1"+"_"+"jlink2", "link1", "jlink2")
hstr += createRevoluteJoint("joint_"+"jlink2"+"_"+"link2", "jlink2", "link2",0,L1+2*thickness,0, lowerLimit = -3.14, upperLimit = 3.14)

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname
###############################################################################
## Create inner 1dof robot
###############################################################################
inner_robot_name = robot_name + "_1dof_inner"
fname = getPathname(folder, inner_robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+inner_robot_name+'">\n')

hstr  = createRotatedCylinder("jlink1",0,0,0,0,0,1.57,thickness,thickness_Z*2)
hstr += createCuboid("link1",0,L1/2+thickness,0,  2*thickness,L1,thickness_Z*2)
hstr += createRotatedCylinder("jlink2",0,L1+2*thickness,0,0,0,1.57,thickness,thickness_Z*2)

hstr += createRevoluteJoint("joint_"+"jlink1"+"_"+"link1", "jlink1", "link1",0,0,0, lowerLimit = -3.14, upperLimit = 3.14)
hstr += createRigidJoint("joint_"+"link1"+"_"+"jlink2", "link1", "jlink2")

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

###############################################################################
## Create outer 1dof robot
###############################################################################
outer_robot_name = robot_name + "_1dof_outer"
fname = getPathname(folder, outer_robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+inner_robot_name+'">\n')

hstr  = createRotatedCylinder("link1",0,0,0,0,0,1.57,thickness,thickness_Z/2)
hstr += createRotatedCylinder("link2",0,L1+2*thickness,0,0,0,1.57,L2+thickness,thickness_Z/2)

hstr += createRevoluteJoint("joint_"+"link1"+"_"+"link2", "link1", "link2", lowerLimit = -3.14, upperLimit = 3.14)

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname
