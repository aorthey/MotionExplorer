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
  hstr  += createMesh("meshes/shuttle.stl")
  hstr += '  <klampt package_root="../../.." default_acc_max="4" >\n'
  hstr += '  </klampt>\n'
  hstr += '</robot>'
  return hstr

robot_name = 'spaceshuttle'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write(GetXMLString(robot_name))
f.close()
print "\nCreated new file >>",fname
