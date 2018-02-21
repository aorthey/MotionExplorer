import os 
import numpy as np
import sys
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

robot_name = 'robots/snake'
Nsegments = 3

headradius = 0.12
length = 0.3

limit = pi/4
stublength = length/8
radius = headradius/4

lowerLimit=-limit
upperLimit=limit
radius_cylinder = headradius

sRadius = 2*radius
config = ''
folder=''
fname = folder+robot_name+'.urdf'
pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
pathname = os.path.abspath(pathname)+'/'
fname = pathname + fname

def createHead(headname):
  hstrs = createSphere("eye",0,0,0,headradius)
  hstrc = createCylinder(headname,-headradius/2,0,0,headradius,headradius)
  hstrj = createRigidJoint("joint_eye_"+headname,"eye",headname)
  return hstrs+hstrc+hstrj

def attachBranchSegment(parentlinkname, linkname, x, y, z, lastSegment=False):
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname
  sbl1 = createCylinder(linkname1,-length/2-sRadius,0,0,radius_cylinder,length) 
  sbj1 = createSphericalJoint(parentlinkname+'_'+linkname+'_joint_revolute', parentlinkname,linkname1, x, y, z, lowerLimit=lowerLimit,upperLimit=upperLimit) 
  if lastSegment:
    return sbl1+sbj1
  sbl2 = createSphere(linkname2,-length-2*sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint(linkname+'_fixed', linkname1, linkname2,0,0,0)
  return sbl1+sbj1+sbl2+sbj2

def createBranchSegment(parentlinkname, linkname, x, y, z):
  sbc = commentNewBranch(linkname) 
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname
  sbl1 = createCylinder(linkname1,-stublength/2,0,0,radius_cylinder,stublength) 
  sbj1 = createRigidJoint(parentlinkname+'_'+linkname+'_joint', parentlinkname,linkname1, x, y, z) 
  sbl2 = createSphere(linkname2,-stublength-sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint(linkname+'_fixed', linkname1, linkname2, 0, 0, 0)
  return sbc+sbl1+sbj1+sbl2+sbj2

def createBody(headname, segments):
  s=''
  branchname = "body"
  s += createBranchSegment(headname,branchname+str(0),-headradius-0.01,0,0)
  for i in range(1,segments-1):
    if i==1:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-stublength-sRadius,0,0)
    else:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-length-2*sRadius,0,0)

  if segments>2:
    s+= attachBranchSegment(branchname+str(segments-2),branchname+str(segments-1),-length-2*sRadius,0,0,True)
  else:
    s+= attachBranchSegment(branchname+str(segments-2),branchname+str(segments-1),-stublength-sRadius,0,0,True)

  Njoints = 6 + 1 + 2*(segments-1) + (1+segments)

  print "[default position config]"
  global config
  config = "config=\""+str(Njoints)+" "+" 0"*Njoints+"\""
  print config


  return s


f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')
f.write(createHead("head"))
f.write(createBody("head",Nsegments))
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()
print "\nCreated new file >>",fname


robot_name_head = robot_name + "_head_inner"
fname = pathname + folder+robot_name_head+'.urdf'
f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name_head+'">\n')
f.write(createHead("head"))
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()
print "\nCreated new file >>",fname

robot_name_onelink = robot_name + "_onelink_inner"
fname = pathname + folder+robot_name_onelink+'.urdf'
f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name_onelink+'">\n')
f.write(createHead("head"))
f.write(createBody("head",Nsegments-1))
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()
print "\nCreated new file >>",fname
### create nested robots
CreateSphereRobot(folder, robot_name + "_sphere_inner", headradius)
CreateSphereRobot(folder, robot_name + "_sphere_outer", Nsegments*length)
