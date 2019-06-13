import os 
import numpy as np
import sys
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

link1 = "body"
link2 = "apex"

length = 0.6
width = 0.2
thickness_Z = 0.1

# hstr = createRotatedCylinder(link1, 0, 0, 0, 0, 0, 1.57, 0.5*width, thickness_Z)
hstr= createCuboid(link1, 0, 0, 0, 
    l=length,
    w=width,
    h=thickness_Z)
hstr+= createRotatedCuboid(link2, 0, 0, 0, 
    l=width,
    w=width,
    h=thickness_Z, r=0, p=0, yaw=0.5*1.57)

hstr+= createRigidJoint(link1, link2, 0.5*length, 0, 0)

robot_name = 'planar/rod'
fname = getPathname(robot_name)
f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')
f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()
print("\nCreated new file >>",fname)

