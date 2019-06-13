import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.5
L2 = 1.5
thickness = 0.2
thickness_Z = 0.1

def GetString1DOFLinkage():
  hstr  = createRotatedCylinder("jlink1",0,0,0,0,0,1.57,thickness,thickness_Z)
  hstr += createCuboid("link1",0,L1/2+thickness,0,  2*thickness,L1,thickness_Z)
  hstr += createRotatedCylinder("jlink2",0,0,0,0,0,1.57,thickness,thickness_Z)

  hstr += createRevoluteJoint("jlink1", "link1",0,0,0, lowerLimit = -3.14, upperLimit = 3.14)
  hstr += createRigidJoint( "link1", "jlink2", 0, L1+2*thickness, 0)
  return hstr

def GetString2DOFLinkage():
  hstr = GetString1DOFLinkage()
  hstr += createCuboid("link2",0,L2/2+thickness,0,2*thickness,L2-1e-10,thickness_Z)
  hstr += createRevoluteJoint("jlink2", "link2",0,0,0, lowerLimit = -3.14, upperLimit = 3.14)
  hstr += createCuboid("link3", 0, 0, 0, L2/2, 0.5*thickness, thickness_Z)

  endeff_width = 0.5*thickness
  endeff_length = 2*thickness
  hstr += createCuboid("endeffectorL",-thickness-0.5*endeff_width,0.5*endeff_length, 0,
      endeff_width, endeff_length, thickness_Z)
  hstr += createCuboid("endeffectorR",0.5*endeff_width, 0.5*endeff_length, 0,
      endeff_width, endeff_length, thickness_Z)

  hstr += createRigidJoint( "link2", "link3", 0, L2 + thickness, 0)
  #hstr += createRigidJoint( "link3", "endeffectorL", 0, L2+thickness, 0)
  hstr += createRigidJoint( "link3", "endeffectorL", 0, 0.5*0.5*thickness, 0)

  hstr += createPrismaticJoint( "link3", "endeffectorR", thickness, 0.5*0.5*thickness, 0,
      lowerLimit=-0.3, upperLimit=0.1)
  return hstr

robot_name = '2dof_manipulator/2dof_manip'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr = GetString2DOFLinkage()

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)
