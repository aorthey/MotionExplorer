import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

l_mid = 1.7
l_side = 1.5
r_mid = 0.15
r_side = 0.1

def GetXMLString(robot_name):
  hstr   = '<?xml version="1.0"?>\n'
  hstr  += '<robot name="'+robot_name+'">\n'
  d = (l_mid-l_side)/2
  hstr  += createCylinder("mid"  , 0, 0, 0, r_mid, l_mid)
  hstr  += createCylinder("left" , -d, -r_mid-r_side, 0, r_side, l_side)
  hstr  += createCylinder("right" , -d, +r_mid+r_side, 0, r_side, l_side)

  hstr  += createSphere("s_mid"  , l_mid/2, 0, 0, r_mid)
  hstr  += createSphere("s_left" , l_side/2-d, -r_mid-r_side, 0, r_side)
  hstr  += createSphere("s_right" , l_side/2-d, +r_mid+r_side, 0, r_side)

  hstr  += createRigidJoint("joint_"+"mid"+"_"+"s_mid", "mid", "s_mid")
  hstr  += createRigidJoint("joint_"+"mid"+"_"+"left", "mid", "left")
  hstr  += createRigidJoint("joint_"+"mid"+"_"+"right", "mid", "right")
  hstr  += createRigidJoint("joint_"+"left"+"_"+"s_left", "left", "s_left")
  hstr  += createRigidJoint("joint_"+"right"+"_"+"s_right", "right", "s_right")
  hstr += '  <klampt package_root="../../.." default_acc_max="4" >\n'
  hstr += '  </klampt>\n'
  hstr += '</robot>'
  return hstr

robot_name = 'rocket'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write(GetXMLString(robot_name))
f.close()
print "\nCreated new file >>",fname
