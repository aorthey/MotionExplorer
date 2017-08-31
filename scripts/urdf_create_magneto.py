import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

def createSymmetricMagneto(robot_name, foot_radius, leg_length, leg_radius):
  foot_length = 2*leg_radius

  folder='magneto/'

  fname = robot_name+'.urdf'
  pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
  pathname = os.path.abspath(pathname)+'/'

  fname = pathname + folder + fname

  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')

  hstr  = createRotatedCylinder("foot", 0, 0, 0, 1.57, 0, 0, foot_radius, foot_length)
  hstr += createCylinder("leg", foot_radius+leg_length/2, 0, 0, leg_radius, leg_length)
  hstr += createRotatedCylinder("foot2", 0, 0, 0, 1.57, 0, 0, foot_radius, foot_length)
  hstr += createRevoluteJointY("joint_"+"foot"+"_"+"leg","foot", "leg",0,0,0)
  hstr += createRevoluteJointY("joint_"+"leg"+"_"+"foot2","leg", "foot2",2*foot_radius+leg_length,0,0)

  f.write(hstr)
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()

  print "\nCreated new file >>",fname
def createMagneto(robot_name, foot_radius, leg1_length, leg2_length, leg3_length, leg_radius):
  joint_radius = leg_radius
  joint_length = 2*leg_radius
  foot_length = 2*leg_radius

  folder='magneto/'

  fname = robot_name+'.urdf'
  pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
  pathname = os.path.abspath(pathname)+'/'

  fname = pathname + folder + fname

  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')

  hstr  = createRotatedCylinder("foot", 0, 0, 0, 1.57, 0, 0, foot_radius, foot_length)
  hstr += createCylinder("leg1", foot_radius+leg1_length/2, 0, 0, leg_radius, leg1_length)
  hstr += createRigidJoint("joint_"+"foot"+"_"+"leg1", "foot", "leg1")

  hstr += createRotatedCylinder("joint1", foot_radius+leg1_length+joint_radius, 0, 0, 1.57, 0, 0, joint_radius, joint_length)
  hstr += createRigidJoint("joint_"+"joint1"+"_"+"leg1", "leg1", "joint1")

  hstr += createCylinder("leg2", leg2_length/2+joint_radius, 0, 0, leg_radius, leg2_length)
  hstr += createRevoluteJointY("joint_"+"leg2"+"_"+"joint1","joint1", "leg2",foot_radius+leg1_length+joint_radius,0,0)

  hstr += createRotatedCylinder("joint2", leg2_length+2*joint_radius, 0, 0, 1.57, 0, 0, joint_radius, joint_length)
  hstr += createRigidJoint("joint_"+"joint2"+"_"+"leg2", "leg2", "joint2")

  hstr += createCylinder("leg3", leg3_length/2+joint_radius, 0, 0, leg_radius, leg3_length)
  hstr += createRevoluteJointY("joint_"+"leg3"+"_"+"joint2","joint2", "leg3",leg2_length+2*joint_radius,0,0)

  hstr += createRotatedCylinder("foot2", leg3_length+joint_radius+foot_radius, 0, 0, 1.57, 0, 0, foot_radius, foot_length)
  hstr += createRigidJoint("joint_"+"foot2"+"_"+"leg3", "leg3", "foot2")

  f.write(hstr)
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()

  print "\nCreated new file >>",fname

foot_radius= 0.05

leg1_length = 0.4
leg2_length = 0.5
leg3_length = 0.4

leg_radius = 0.04

#robot_name = 'magneto'
#createMagneto(robot_name, foot_radius, leg1_length, leg2_length, leg3_length, leg_radius)

leg_length=0.01
robot_name = 'magneto0'
createSymmetricMagneto(robot_name, foot_radius, leg_length, leg_radius)

leg_length=0.5
robot_name = 'magneto1'
createSymmetricMagneto(robot_name, foot_radius, leg_length, leg_radius)

robot_name = 'magneto2'
leg1_length = 0.2
leg2_length = 0.4
leg3_length = 0.2
createMagneto(robot_name, foot_radius, leg1_length, leg2_length, leg3_length, leg_radius)
