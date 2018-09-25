import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

length = 2.0
width = 0.7

height = 0.1

robot_name = 'PlanarRectangle'
folder='robots/'
fname = getPathname(folder, robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr  = createCuboid("link",0,0,0,length,width,height)

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",fname

### create nested robots
d_inner = np.minimum(length, width)/2.0
d_outer = np.sqrt( (length/2.0)**2 + (width/2.0)**2)

## thickness_Z change for visibility in simulator (does not change the collisions)
CreateCylinderRobot(folder, robot_name + "_cylinder_inner", d_inner, 2.0*height, 1.57)
CreateCylinderRobot(folder, robot_name + "_cylinder_outer", d_outer, height/2.0, 1.57)
