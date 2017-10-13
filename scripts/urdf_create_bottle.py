import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

body_radius= 0.12
body_length = 0.5
neck_radius = 0.5*body_radius
neck_length = 0.6*body_length
cap_radius = 0.8*body_radius
cap_height = 0.05

robot_name = 'bottle'
folder='bottle/'

fname = robot_name+'.urdf'
pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
pathname = os.path.abspath(pathname)+'/'

fname = pathname + folder + fname

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')
hstr  = createCylinder("body", 0, 0, 0, body_radius, body_length)
hstr += createSphere("scarf",body_length/2,0,0,body_radius)
hstr += createCylinder("neck", body_length/2+neck_length/2, 0, 0, neck_radius, neck_length)
hstr += createCylinder("cap", body_length/2+neck_length+cap_height/2, 0, 0, cap_radius, cap_height)
hstr += createRigidJoint("joint_"+"body"+"_"+"scarf", "body", "scarf")
hstr += createRigidJoint("joint_"+"body"+"_"+"neck", "body", "neck")
hstr += createRigidJoint("joint_"+"neck"+"_"+"cap", "neck", "cap")
f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('    <noselfcollision pairs="scarf neck"/>\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
CreateSphereRobot(folder, robot_name + "_sphere_inner", body_radius)

CreateSphereRobot(folder, robot_name + "_sphere_outer", body_length+neck_length+cap_height)

CreateCylinderRobot(folder, robot_name + "_capsule_inner", body_radius, body_length)

CreateCylinderRobot(folder, robot_name + "_capsule_outer", body_radius, body_length+neck_length+cap_height)

