import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

foot_radius= 0.1
foot_length = 0.1

leg1_length = 0.4
leg2_length = 0.5
leg3_length = 0.4

leg_radius = 0.04

joint_radius = 2*leg_radius
joint_length = 2*leg_radius

robot_name = 'magneto'
folder='magneto/'

fname = robot_name+'.urdf'
pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
pathname = os.path.abspath(pathname)+'/'

fname = pathname + folder + fname

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr  = createCylinder("foot", 0, 0, 0, foot_radius, foot_length)
hstr += createCylinder("leg1", foot_length/2+leg1_length/2, 0, 0, leg_radius, leg1_length)
hstr += createRigidJoint("joint_"+"foot"+"_"+"leg1", "foot", "leg1")

hstr += createRotatedCylinder("joint1", foot_length/2+leg1_length+joint_radius, 0, 0, 1.57, 0, 0, joint_radius, joint_length)
hstr += createRigidJoint("joint_"+"joint1"+"_"+"leg1", "leg1", "joint1")

hstr += createCylinder("leg2", leg2_length/2+joint_radius, 0, 0, leg_radius, leg2_length)
hstr += createRevoluteJointY("joint_"+"leg2"+"_"+"joint1","joint1", "leg2",foot_length/2+leg1_length+joint_radius,0,0)

hstr += createRotatedCylinder("joint2", leg2_length+2*joint_radius, 0, 0, 1.57, 0, 0, joint_radius, joint_length)
hstr += createRigidJoint("joint_"+"joint2"+"_"+"leg2", "leg2", "joint2")

hstr += createCylinder("leg3", leg3_length/2+joint_radius, 0, 0, leg_radius, leg3_length)
hstr += createRevoluteJointY("joint_"+"leg3"+"_"+"joint2","joint2", "leg3",leg2_length+2*joint_radius,0,0)

hstr += createCylinder("foot2", leg3_length+joint_radius+foot_length/2, 0, 0, foot_radius, foot_length)
hstr += createRigidJoint("joint_"+"foot2"+"_"+"leg3", "leg3", "foot2")

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname
