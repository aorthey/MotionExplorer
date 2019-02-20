import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

length = 0.8
width  = 0.5
height = 0.2

def GetXMLString(robot_name):
  hstr   = '<?xml version="1.0"?>\n'
  hstr  += '<robot name="'+robot_name+'">\n'

    #      y
    #       |
    # width |
    #       |---------
    #          length     x

  hstr  += createCuboid("chassi" , 0, 0, 0, length, width, height)
  ll = 0.4*length
  lw = 0.2*width
  px = 0.4*length
  py = 0.7*width

  hstr  += createCuboid("wheel1" , -px, +py, 0, ll, lw, height)
  hstr  += createCuboid("wheel2" , -px, -py, 0, ll, lw, height)
  hstr  += createCuboid("wheel3" , +px, +py, 0, ll, lw, height)
  hstr  += createCuboid("wheel4" , +px, -py, 0, ll, lw, height)
  hstr  += createRotatedCylinder("frontspoiler" , 0.5*length, 0, 0, 0, 0, 90, 0.5*width, height)

  aw = py - 0.5*width - 0.5*lw
  ap = py - 0.5*lw - 0.6*aw
  hstr  += createCuboid("axis1", +px, +ap, 0, 0.05*length, aw, 0.3*height)
  hstr  += createCuboid("axis2" , -px, +ap, 0, 0.05*length, aw, 0.3*height)
  hstr  += createCuboid("axis3" , +px, -ap, 0, 0.05*length, aw, 0.3*height)
  hstr  += createCuboid("axis4" , -px, -ap, 0, 0.05*length, aw, 0.3*height)
  #hstr = createRotatedCylinder("body",0,0,0,0,0,yaw,radius,length)
  hstr  += createRigidJoint( "chassi", "wheel1")
  hstr  += createRigidJoint( "chassi", "wheel2")
  hstr  += createRigidJoint( "chassi", "wheel3")
  hstr  += createRigidJoint( "chassi", "wheel4")
  hstr  += createRigidJoint( "chassi", "frontspoiler")
  hstr  += createRigidJoint( "chassi", "axis1")
  hstr  += createRigidJoint( "chassi", "axis2")
  hstr  += createRigidJoint( "chassi", "axis3")
  hstr  += createRigidJoint( "chassi", "axis4")

  hstr += '  <klampt package_root="../../.." default_acc_max="4" >\n'
  hstr += '  </klampt>\n'
  hstr += '</robot>'
  return hstr

robot_name = 'dubinscar/dubinscar'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write(GetXMLString(robot_name))
f.close()
print "\nCreated new file >>",fname

CreateCylinderRobot(robot_name + "_capsule_inner", 0.5*width, height)
CreateCylinderRobot(robot_name + "_capsule_outer", width, height)
