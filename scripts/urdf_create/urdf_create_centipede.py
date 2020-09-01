import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *


link_radius = 0.04
joint_radius = link_radius
def createSingleLeg(connectorLink, suffix, x, y, yaw=0):
  link1_length = 0.3
  link2_length = 0.5
  link3_length = 0.3
  joint_length = 2*link_radius

  hstr  = createRotatedCylinder(suffix+"central_link", 0, 0, 0, 0, 0, 1.57, 0.8*joint_radius, joint_length)
  hstr += createRigidJoint( connectorLink, suffix+"central_link", x, y, 0, 0, 0,
      yaw)

  hstr += createCylinder(suffix+"link1", joint_radius+link1_length/2, 0, 0, link_radius, link1_length)
  hstr += createRevoluteJointZ(suffix+"central_link", suffix+"link1",0,0,0)

  hstr += createRotatedCylinder(suffix+"joint1",
      joint_radius+link1_length+joint_radius, 0, 0, 1.57, 0, 0, 0.8*joint_radius, joint_length)
  hstr += createRigidJoint( suffix+"link1", suffix+"joint1")

  hstr += createCylinder(suffix+"link2", link2_length/2+joint_radius, 0, 0, link_radius, link2_length)
  hstr += createRevoluteJointY(suffix+"joint1", suffix+"link2",joint_radius+link1_length+joint_radius,0,0)

  hstr += createRotatedCylinder(suffix+"joint2", link2_length+2*joint_radius, 0,
      0, 1.57, 0, 0, 0.8*joint_radius, joint_length)
  hstr += createRigidJoint( suffix+"link2", suffix+"joint2")

  hstr += createCylinder(suffix+"link3", link3_length/2+joint_radius, 0, 0, link_radius, link3_length)
  hstr += createRevoluteJointY(suffix+"joint2", suffix+"link3",link2_length+2*joint_radius,0,0)

  d = link3_length+joint_radius+joint_radius
  hstr += createFoot(suffix, suffix+"link3", d)
  # hstr += createSphere(suffix+"foot", d, 0, 0, 0.8*joint_radius)
  # hstr += createRigidJoint(suffix+"link3", suffix+"foot")
  return hstr

def createFoot(suffix, lastLink, d):
  hstr  = createEmptyLink(suffix+"foot", 0, 0, 0)
  hstr += createRigidJoint(lastLink, suffix+"foot", d)
  return hstr

def createDoubleLeg(connectorLink, suffix, x, y):
  length_middle_axis = 0.3
  hstr = createCylinder(suffix+"middle_axis", 0, 0, 0, 0.9*link_radius, length_middle_axis)
  hstr += createRigidJoint( connectorLink, suffix+"middle_axis", x, y, 0)
  hstr += createSingleLeg(suffix+"middle_axis", suffix+"leg1",
      0.5*length_middle_axis+joint_radius, 0)
  hstr += createSingleLeg(suffix+"middle_axis", suffix+"leg2",
      -0.5*length_middle_axis-joint_radius, 0,
      3.14)
  return hstr

def createLegPairs(N, distanceBetweenPairs):
    
  y = 0
  hstr=""
  for n in range(0,N):
    cname = "main"+str(n)
    suffix = "pair"+str(n)
    sname = cname + "_connector"
    lengthCylinder = 0.5*distanceBetweenPairs - joint_radius
    hstr += createRotatedCylinder(cname+"_A", 0, 0, 0, 1.57, 0, 0, link_radius,
        lengthCylinder)
    if n > 0:
      hstr += createRigidJoint("main"+str(n-1), cname+"_A", 0,
          -lengthCylinder-link_radius-joint_radius, 0)

    hstr += createSphere(sname, 0, 0, 0, 0.8*joint_radius)
    hstr += createRevoluteJointX(cname+"_A", sname, 0,
        -0.5*lengthCylinder - joint_radius, 0)

    hstr += createRotatedCylinder(cname, 0, 0, 0, 1.57, 0, 0, link_radius,
        lengthCylinder)
    hstr += createRigidJoint(sname, cname, 0, -0.5*lengthCylinder-joint_radius, 0)

    hstr += createDoubleLeg(cname, suffix, 0,
        -0.5*lengthCylinder-joint_radius)
  return hstr

def createCentipede(robot_name):
  fname = getPathname(robot_name)

  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')

  hstr = createLegPairs(3, 0.5)

  f.write(hstr)
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()
  print("\nCreated new file >>",fname)


robot_name = 'contact/centipede'
createCentipede(robot_name)
