import os 
import numpy as np
import sys
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

#number of segments of snake INCLUDING head segment
Nsegments = 10

head_radius = 0.1
segment_length = 3*head_radius
segment_width = head_radius
segment_joint_radius = 0.7*head_radius
thickness_Z = 0.1

limit = 3*pi/8
lowerLimit=-limit
upperLimit=limit
epsilon_gap = 1e-5

def createHead(branchname):
  linkname1 = "eye"
  hstr = createRotatedCylinder(linkname1, 0, 0, 0, 0, 0, 1.57, head_radius, thickness_Z)
  hstr+= createCuboid(branchname, 0, 0, 0, 
      l=head_radius,
      w=2*head_radius,
      h=thickness_Z)
  hstr+= createRigidJoint(linkname1, branchname, -0.5*head_radius, 0, 0)
  return hstr

def attachBranchSegment(parentlinkname, linkname, xoffset):
  jlinkname = linkname+'_joint_link'
  #hstr = createSphere(jlinkname,0,0,0,segment_joint_radius)
  hstr = createRotatedCylinder(jlinkname,0,0,0,0,0,1.57,radius=segment_joint_radius,length=thickness_Z)
  hstr+= createRigidJoint(parentlinkname, jlinkname, xoffset, 0, 0)
  hstr+= createRevoluteJoint(jlinkname,linkname, 0, 0, 0, lowerLimit=lowerLimit,upperLimit=upperLimit) 
  segment_offset = -0.5*segment_length - segment_joint_radius - epsilon_gap
  hstr+= createCuboid(linkname,segment_offset,0,0,
      l=segment_length,
      w=2*segment_width,
      h=thickness_Z) 
  return hstr

def GetNsegmentString(Nsegments):
  branchname = "body"
  s = createHead(branchname+str(0))
  xoffset = -(0.5*head_radius + segment_joint_radius + epsilon_gap)
  for i in range(0,Nsegments-1):
    s+= attachBranchSegment(branchname+str(i),branchname+str(i+1), xoffset)
    xoffset = -(segment_length + 2*segment_joint_radius + epsilon_gap)
  return s

def CreateRobotNsegments(robot_name, Nsegments):
  fname = getPathname(robot_name)
  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')
  f.write(GetNsegmentString(Nsegments))
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()
  print "\nCreated new file >>",fname

robot_name = 'snake/snake_planar'
for i in range(0,Nsegments):
  CreateRobotNsegments(robot_name+"_"+str(i)+"_segments", i+1)

CreateCylinderRobot(robot_name + "_disk_inner", head_radius, thickness_Z, 1.57)
# CreateSphereRobot(robot_name + "_0_segments_sphere_inner", headradius)
# CreateSphereRobot(robot_name + "_0_segments_sphere_outer", np.sqrt(2)*headradius)
# CreateSphereRobot(robot_name + "_"+str(Nsegments-1)+"_segments_sphere_outer", (Nsegments-1)*length+(Nsegments-1)*2*sRadius+headradius)
