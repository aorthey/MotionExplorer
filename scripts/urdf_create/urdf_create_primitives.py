import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

def CreateSphereRobot(robot_name, radius):
  fname = getPathname(robot_name)

  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')
  hstr = createSphere("body",0,0,0,radius,PHYSICAL=True)
  f.write(hstr)
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()
  print "\nCreated new file >>",fname,"[sphere robot radius=",radius,"]"

def CreateCylinderRobot(robot_name, radius, length, yaw=0):
  fname = getPathname(robot_name)

  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')
  #hstr = createCylinder("body", 0, 0, 0, radius, length)
  hstr = createRotatedCylinder("body",0,0,0,0,0,yaw,radius,length)
  f.write(hstr)
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()
  print "\nCreated new file >>",fname,"[cylinder robot radius=",radius," length=",length,"]"

