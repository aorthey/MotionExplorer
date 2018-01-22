import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

l_body =3
r_body = 0.5
l_connector = 1.5
r_connector = 0.05
l_wing  = 4
w_wing  = 0.01

robot_name = 'hubble'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr   = createCylinder("body1" , 0, 0, 0, r_body, l_body)
hstr  += createCylinder("body2", -l_body/3.0, 0, 0, 1.3*r_body, l_body/3.0)

hstr  += createRotatedCylinder("connector1", 0, -r_body-l_connector/2.0, 0,
    pi/2.0,
    0, 0, r_connector, l_connector)
hstr  += createRotatedCylinder("connector2", 0, +r_body+l_connector/2.0, 0,
    pi/2.0,
    0, 0, r_connector, l_connector)

hstr  += createCuboid("wing1", 0, -r_body-2.0*l_connector/3.0, 0,
    w_wing, 2.0*l_connector/3.0, l_wing)
hstr  += createCuboid("wing2", 0, +r_body+2.0*l_connector/3.0, 0,
    w_wing, 2.0*l_connector/3.0, l_wing)

hstr += createRigidJoint("joint_"+"b1"+"_"+"b2", "body1", "body2")
hstr += createRigidJoint("joint_"+"b1"+"_"+"connector1", "body1", "connector1")
hstr += createRigidJoint("joint_"+"b1"+"_"+"connector2", "body1", "connector2")
hstr += createRevoluteJointY("joint_"+"connector1"+"_"+"wing1", "connector1", "wing1")
hstr += createRevoluteJointY("joint_"+"connector2"+"_"+"wing2", "connector2", "wing2")

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
CreateSphereRobot(folder, robot_name + "_sphere_inner", r_body)
CreateSphereRobot(folder, robot_name + "_sphere_outer", l_body)
CreateCylinderRobot(folder, robot_name + "_capsule_inner", r_body, l_body)
CreateCylinderRobot(folder, robot_name + "_capsule_outer", l_body, l_body)

