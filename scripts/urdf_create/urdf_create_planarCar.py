import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

length = 0.4
width  = 0.2
height = 0.1

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
  py = 0.65*width

  hstr  += createRotatedCylinder("wheel1" , -px, +py, 0, 
      r=0, p=1.57, yaw=1.57, radius=0.5*ll, length=lw)
  hstr  += createRotatedCylinder("wheel2" , -px, -py, 0, 
      r=0, p=1.57, yaw=1.57, radius=0.5*ll, length=lw)
  hstr  += createRotatedCylinder("wheel3" , +px, +py, 0, 
      r=0, p=1.57, yaw=1.57, radius=0.5*ll, length=lw)
  hstr  += createRotatedCylinder("wheel4" , +px, -py, 0, 
      r=0, p=1.57, yaw=1.57, radius=0.5*ll, length=lw)

  hstr  += createRotatedCuboid("frontspoiler" , 0.5*length, 0, 0, 
      l=  np.sqrt(2)*0.5*width,
      w = np.sqrt(2)*0.5*width,
      h = height,
      r=0, p=0,
      yaw=0.5*1.57)

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

robot_name = 'planar/car'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write(GetXMLString(robot_name))
f.close()
print("\nCreated new file >>",fname)

CreateCylinderRobot(robot_name + "_capsule_inner", 0.5*width, height)
