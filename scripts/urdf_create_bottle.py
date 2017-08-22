import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

body_radius= 0.12
body_length = 0.5
neck_radius = 0.5*body_radius
neck_length = 0.6*body_length
cap_radius = 0.8*body_radius
cap_height = 0.05

name = 'bottle'
folder='bottle/'

fname = folder+name+'.urdf'
xmlname = folder+name+'.xml'
pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
pathname = os.path.abspath(pathname)+'/'
fname = pathname + fname
xmlname = pathname + xmlname
env_name = 'bottle/twohole.tri'

f = open(fname,'w')

f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+name+'">\n')

hstrs = createCylinder("body", 0, 0, 0, body_radius, body_length)
f.write(hstrs)

hstrs = createSphere("scarf",body_length/2,0,0,body_radius)
f.write(hstrs)

hstrs = createCylinder("neck", body_length/2+neck_length/2, 0, 0, neck_radius, neck_length)
f.write(hstrs)

hstrs = createCylinder("cap", body_length/2+neck_length+cap_height/2, 0, 0, cap_radius, cap_height)
f.write(hstrs)

hstr = createRigidJoint("joint_"+"body"+"_"+"scarf", "body", "scarf")
f.write(hstr)
hstr = createRigidJoint("joint_"+"body"+"_"+"neck", "body", "neck")
f.write(hstr)
hstr = createRigidJoint("joint_"+"neck"+"_"+"cap", "neck", "cap")
f.write(hstr)

f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('    <noselfcollision pairs="scarf neck"/>\n')

f.write('  </klampt>\n')
f.write('</robot>')
f.close()
print "\nCreated new file >>",
print fname

#f = open(xmlname,'w')
#f.write('<?xml version="1.0"?>\n\n')
#f.write('<world>\n')
#
#robotstr  = '  <robot name=\"'+name+'\"'
#robotstr += ' file="'+str(fname)+'"'
#robotstr += ' translation="0 0 0"'
#robotstr += ' rotateRPY="0 0 0"'
#robotstr += ' />\n\n'
#f.write(robotstr)
#
#terrainstr  = '  <rigidObject '
#terrainstr += ' name=\"'+str(env_name)+'\"'
#terrainstr += ' file=\"/home/aorthey/git/orthoklampt/data/'+str(env_name)+'\"'
#terrainstr += ' translation="0 2 -1"/>\n\n'
#f.write(terrainstr)
#
##terrainstr  = '  <rigidObject '
##terrainstr += ' name=\"'+str(env_name)+'\"'
##terrainstr += ' file=\"/home/aorthey/git/orthoklampt/data/terrains/'+str(env_name)+'\"'
##terrainstr += ' translation="0 0 0"/>\n\n'
##f.write(terrainstr)
#
#f.write('</world>')
#f.close()
#
#print "\nCreated new file >>",
#print xmlname
