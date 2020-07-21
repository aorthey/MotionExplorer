import os 
import numpy as np
import sys
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

#number of segments of snake INCLUDING head segment
Nsegments = 10

headradius = 0.1
segment_length = 3*headradius
limit = 3*pi/8

lowerLimit=-limit
upperLimit=limit
radius_cylinder = headradius
sRadius = 0.7*headradius

epsilon_gap = 1e-5

def createHead(branchname):
  linkname1 = "eye"
  hstr = createSphere(linkname1, 0, 0, 0, headradius)
  hstr+= createCylinder(branchname, 0, 0, 0, radius=headradius,length=headradius)
  hstr+= createRigidJoint(linkname1, branchname, -0.5*headradius, 0, 0)
  return hstr

def attachBranchSegment(parentlinkname, linkname, xoffset):
  jlinkname = linkname+'_joint_link'
  hstr = createSphere(jlinkname,0,0,0,sRadius)
  hstr+= createRigidJoint(parentlinkname, jlinkname, xoffset, 0, 0)
  hstr+= createSphericalJoint(jlinkname,linkname, 0, 0, 0, lowerLimit=lowerLimit,upperLimit=upperLimit) 
  hstr+= createCylinder(linkname,-0.5*segment_length-sRadius-epsilon_gap,0,0,radius = radius_cylinder,length=segment_length) 
  return hstr

def GetNsegmentString(Nsegments):
  branchname = "body"
  s = createHead(branchname+str(0))
  xoffset = -(0.5*headradius+sRadius + epsilon_gap)
  for i in range(0,Nsegments-1):
    s+= attachBranchSegment(branchname+str(i),branchname+str(i+1), xoffset)
    xoffset = -(segment_length + 2*sRadius + epsilon_gap)
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
  print("\nCreated new file >>",fname)

robot_name = 'snake/snake'
for i in range(0,Nsegments):
  CreateRobotNsegments(robot_name+"_"+str(i)+"_segments", i+1)
  if i>0:
    CreateSphereRobot(robot_name + "_"+str(i)+"_segments_sphere_outer", i*segment_length+i*2*sRadius+headradius)

CreateSphereRobot(robot_name + "_0_segments_sphere_inner", headradius)
CreateSphereRobot(robot_name + "_0_segments_sphere_outer", np.sqrt(2)*headradius)
