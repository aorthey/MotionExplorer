import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

length = 0.15
radius = 0.01
sphere_scale = 2
headradius = 0.1
Nsegments = 5
Nbranches = 4
aperture = 0.4 ## aperture of bouquet of branches
name = 'sentinel_complete'

sRadius = sphere_scale*radius
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
  sbl1 = createCylinder(linkname1,-length/2-sRadius,0,0,radius,length) 
  sbj1 = createSphericalJoint(parentlinkname+'_'+linkname+'_joint_revolute', parentlinkname,linkname1, x, y, z) 
  sbl2 = createSphere(linkname2,-length-2*sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint(linkname+'_fixed', linkname1, linkname2,0,0,0)
  return sbl1+sbj1+sbl2+sbj2

def createBranchSegment(parentlinkname, linkname, x, y, z):
  sbc = commentNewBranch(linkname) 
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname
  sbl1 = createCylinder(linkname1,-length/2,0,0,radius,length) 
  sbj1 = createRigidJoint(parentlinkname+'_'+linkname+'_joint', parentlinkname,linkname1, x, y, z) 
  sbl2 = createSphere(linkname2,-length-sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint(linkname+'_fixed', linkname1, linkname2, 0, 0, 0)
  return sbc+sbl1+sbj1+sbl2+sbj2

def createBranch(headname, branchname,x,y,z):
  s = createBranchSegment(headname,branchname+str(0),x,y,z)
  for i in range(1,Nsegments):
    if i==1:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-length-sRadius,0,0)
    else:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-length-2*sRadius,0,0)
  return s

def createBranchBundle(headname):
  s=''
  tt = headradius / max(radius, sphere_scale*radius)
  if tt < 2.0:
    print "[warning] head radius too small, branches too big"

  scale = 0.5

  for i in range(0,Nbranches):
    y = scale*headradius*cos(i*2*pi/Nbranches)
    z = scale*headradius*sin(i*2*pi/Nbranches)
    s+=createBranch("head","branch_"+str(i),-headradius,y,z)

  ## CSpace structure:
  ## SE(3) ~ R^6 (local chart) 
  ## + Nbranches*(Nsegments-1) spherical joints /w ## limits ~ (R^2)
  ## + Nbranches*(1+Nsegments) rigid fixed links

  Njoints = 6 + 1 + Nbranches * 2*(Nsegments-1) + Nbranches*(1+Nsegments)
  print "[default position config]"
  print "config=\""+str(Njoints)+" "+" 0"*Njoints+"\""
  print "[arms open config]"

  global config
  config = str("config=\"")

  x=0
  y=0
  z=1
  config += str(Njoints) + " "
  config += str(x) + " " + str(y) + " " + str(z)
  config += str(" 0"*3) ## SE(3)
  config += str(" 0"*1) ## head-eye

  ## create a nice bouquet of branches
  for i in range(0,Nbranches):
    config += str(" 0"*2) ## first segment is fixed
    y = cos(i*2*pi/Nbranches)
    z = sin(i*2*pi/Nbranches)
    thetaY = atan2(abs(y),abs(z))
    thetaZ = pi/2-atan2(abs(y),abs(z))
    if y>0:
      thetaY *= -1
    if z<0:
      thetaZ *= -1

    for j in range(1,Nsegments):
      config += " "+str(aperture*thetaZ)+" "+str(aperture*thetaY)
      config+= " 0"

    #config+= " 0"*Nsegments ## shoulder of branch
  config+= "\""
  print config
  return s

headname = "head"
f.write(createHead(headname))
f.write(createBranchBundle(headname))
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
terrainstr = '  <terrain file=\"/home/aorthey/git/Klampt/data/terrains/plane.tri\"'
terrainstr += '  translation=\"0 0 0\"/>\n\n'
f.write(terrainstr)

robotstr  = '  <robot name=\"'+name+'\"'
robotstr += ' file="'+str(fname)+'"'
robotstr += ' translation="0 0 0"'
robotstr += ' rotateRPY="0 0 3.14"'
robotstr += ' '+config+'/>\n\n'
f.write(robotstr)

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
