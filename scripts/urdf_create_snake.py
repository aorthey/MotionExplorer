import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

length = 0.1
stublength = length/8
radius = 0.01
headradius = 0.1
radius_cylinder = headradius-headradius/2
Nsegments = 9
limit = pi/4
lowerLimit=-limit
upperLimit=limit

name = 'snake'
env_name = 'pipes/pipedreamin.tri'


sRadius = 2*radius
config = ''
folder=''
fname = folder+name+'.urdf'
xmlname = folder+name+'.xml'
pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
pathname = os.path.abspath(pathname)+'/'
fname = pathname + fname
xmlname = pathname + xmlname

f = open(fname,'w')

f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+name+'">\n')

def createHead(headname):
  hstrs = createSphere("eye",0,0,0,headradius)
  hstrc = createCylinder(headname,-headradius/2,0,0,headradius,headradius)
  hstrj = createRigidJoint("joint_eye_"+headname,"eye",headname)
  return hstrs+hstrc+hstrj

def attachBranchSegment(parentlinkname, linkname, x, y, z):
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname
  sbl1 = createCylinder(linkname1,-length/2-sRadius,0,0,radius_cylinder,length) 
  sbj1 = createSphericalJoint(parentlinkname+'_'+linkname+'_joint_revolute', parentlinkname,linkname1, x, y, z, lowerLimit=lowerLimit,upperLimit=upperLimit) 
  sbl2 = createSphere(linkname2,-length-2*sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint(linkname+'_fixed', linkname1, linkname2,0,0,0)
  return sbl1+sbj1+sbl2+sbj2

def createBranchSegment(parentlinkname, linkname, x, y, z):
  sbc = commentNewBranch(linkname) 
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname
  # sbl1 = createCylinder(linkname1,-length/2,0,0,radius_cylinder,length) 
  # sbj1 = createRigidJoint(parentlinkname+'_'+linkname+'_joint', parentlinkname,linkname1, x, y, z) 
  # sbl2 = createSphere(linkname2,-length-sRadius,0,0,0.95*sRadius)
  # sbj2 = createRigidJoint(linkname+'_fixed', linkname1, linkname2, 0, 0, 0)
  sbl1 = createCylinder(linkname1,-stublength/2,0,0,radius_cylinder,stublength) 
  sbj1 = createRigidJoint(parentlinkname+'_'+linkname+'_joint', parentlinkname,linkname1, x, y, z) 
  sbl2 = createSphere(linkname2,-stublength-sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint(linkname+'_fixed', linkname1, linkname2, 0, 0, 0)
  return sbc+sbl1+sbj1+sbl2+sbj2

def createBody(headname):
  s=''
  branchname = "body"
  s += createBranchSegment(headname,branchname+str(0),-headradius,0,0)
  for i in range(1,Nsegments):
    if i==1:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-stublength-sRadius,0,0)
    else:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-length-2*sRadius,0,0)
    #global length
    #length = length/2
    global radius_cylinder
    radius_cylinder = radius_cylinder-(radius_cylinder/6)

  ## CSpace structure:
  ## SE(3) ~ R^6 (local chart) 
  ## + Nsegments 

  Njoints = 6 + 1 + 2*(Nsegments-1) + (1+Nsegments)

  print "[default position config]"
  global config
  config = "config=\""+str(Njoints)+" "+" 0"*Njoints+"\""
  print config


  return s

headname = "head"
f.write(createHead(headname))
f.write(createBody(headname))
#f.write('  <klampt package_root="../.." flip_yz="1" use_vis_geom="1">\n')
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
#f.write('    <noselfcollision pairs="head eye"/>\n')
#branch_00_spherical_joint_link
#f.write('    <link name="branch_00_spherical_joint_link" physical="0" />\n')
#f.write('    <noselfcollision pairs="eye branch_00"/>\n')
#f.write('    <noselfcollision pairs="eye branch_00_cylinder"/>\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print "\nCreated new file >>",
print fname

f = open(xmlname,'w')
f.write('<?xml version="1.0"?>\n\n')
f.write('<world>\n')

robotstr  = '  <robot name=\"'+name+'\"'
robotstr += ' file="'+str(fname)+'"'
robotstr += ' translation="0 0 0"'
robotstr += ' rotateRPY="0 0 0"'
robotstr += ' '+config+'/>\n\n'
f.write(robotstr)

terrainstr  = '  <rigidObject '
terrainstr += ' name=\"'+str(env_name)+'\"'
terrainstr += ' file=\"/home/aorthey/git/orthoklampt/data/terrains/'+str(env_name)+'\"'
terrainstr += ' translation="0 0 0"/>\n\n'
f.write(terrainstr)

ctrlstr  = '  <simulation>\n'
ctrlstr += '    <globals maxContacts="20" />\n'
ctrlstr += '    <robot index="0">\n'
ctrlstr += '      <controller type="PolynomialPathController" />\n'
ctrlstr += '    </robot>\n'
ctrlstr += '  </simulation>\n\n'
f.write(ctrlstr)
f.write('</world>')
f.close()

print "\nCreated new file >>",
print xmlname
