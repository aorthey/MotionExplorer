import os 
import numpy as np
import sys
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

Nsegments = 60

thickness = 0.03
length_segment = 0.1
radius_cylinder = thickness
radius_connector = thickness

limit = pi/2

lowerLimit=-limit
upperLimit=limit

epsilon_gap = 1e-5


def attachTail(parentlinkname, linkname, xoffset):
  hstr = createSphere(linkname,0,0,0,radius_connector)
  hstr+= createRigidJoint(parentlinkname, linkname, xoffset, 0, 0)

  toffset= -(radius_connector + thickness)
  tailname = linkname+"_tail"
  hstr+= createSphere(tailname,0,0,0,thickness)
  hstr+= createRigidJoint(linkname, tailname, toffset, 0, 0)
  return hstr

def createHead(linkname):
  # hstr = createSphere(linkname, 0, 0, 0, thickness)
  hstr= createCylinder(linkname,0,0,0,radius =
      radius_cylinder,length=length_segment) 
  return hstr

def attachBranchSegment(parentlinkname, linkname, xoffset):
  jlinkname = linkname+'_joint_link'
  hstr = createSphere(jlinkname,0,0,0,radius_connector)
  hstr+= createRigidJoint(parentlinkname, jlinkname, xoffset, 0, 0)
  hstr+= createSphericalJoint(jlinkname,linkname, 0, 0, 0, lowerLimit=lowerLimit,upperLimit=upperLimit) 
  hstr+= createCylinder(linkname,-0.5*length_segment-radius_connector-epsilon_gap,0,0,radius =
      radius_cylinder,length=length_segment) 
  return hstr

def GetNsegmentString(Nsegments):
  branchname = "body"
  s = createHead(branchname+str(0))
  xoffset = -(thickness + epsilon_gap + 0.5*length_segment)
  for i in range(0,Nsegments-1):
    s += attachBranchSegment(branchname+str(i),branchname+str(i+1), xoffset)
    xoffset = -(length_segment + 2*radius_connector + epsilon_gap)
  # s+= attachTail(branchname+str(Nsegments-1), branchname+str(Nsegments), xoffset)
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

robot_name = 'cable/cable'
for i in range(0,Nsegments):
  CreateRobotNsegments(robot_name+"_"+str(i)+"_segments", i+1)

CreateRobotNsegments(robot_name, Nsegments)

## create klampt twisted configuration
radiusTwist = 0.5
Nconfigs = 6 + 3*(Nsegments-1)
# print('<qinit config=\"'+str(Nconfigs)+' '+'0 '*Nconfigs+'\"/>')

s = '<qinit config=\"'+str(Nconfigs)+' '+'0 '*6
twist=3.1415
for k in range(0,Nsegments):
  if k%7 == 0:
    s+= "0.0 "+str(twist/3)+" 0.0 "
    s+= "0.0 "+str(twist/3)+" 0.0 "
    s+= "0.0 "+str(twist/3)+" 0.0 "
    twist = -twist
    k+=2
  else:
    s+= "0.0 0.0 0.0 "

s+= '\"/>'
print(s)

s = '<qgoal config=\"'+str(Nconfigs)+' '+'0 '*6
for k in range(0,Nsegments):
  s+= "0.0 0.2 0.5 "
s+= '\"/>'


print(s)
