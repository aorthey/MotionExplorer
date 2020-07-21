import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

length = 0.15
stublength = length/8
radius = 0.01
radius_cylinder = 0.02
sphere_scale = 2
headradius = 0.1
# Nsegments = 6
# Nbranches = 8
aperture = 0.4 ## aperture of bouquet of branches
limit = pi/2

sRadius = sphere_scale*radius
config = ''

def createHead(headname):
  hstrs = createSphere("eye",0,0,0,headradius)
  hstrc = createCylinder(headname,-headradius/2,0,0,headradius,headradius)
  hstrj = createRigidJoint("eye",headname)
  return hstrs+hstrc+hstrj

def attachBranchSegment(parentlinkname, linkname, x, y, z):
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname
  sbl1 = createCylinder(linkname1,-length/2-sRadius,0,0,radius_cylinder,length) 
  sbj1 = createSphericalJoint(parentlinkname,linkname1, x, y, z, -limit, limit) 
  sbl2 = createSphere(linkname2,-length-2*sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint(linkname1, linkname2, 0,0,0)
  return sbl1+sbj1+sbl2+sbj2

def createBranchSegment(parentlinkname, linkname, x, y, z):
  sbc = commentNewBranch(linkname) 
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname
  # sbl1 = createCylinder(linkname1,-length/2,0,0,radius_cylinder,length) 
  # sbj1 = createRigidJoint( y, z) 
  # sbl2 = createSphere(linkname2,-length-sRadius,0,0,0.95*sRadius)
  # sbj2 = createRigidJoint( 0, 0)
  sbl1 = createCylinder(linkname1,-stublength/2,0,0,radius_cylinder,stublength) 
  sbj1 = createRigidJoint( parentlinkname, linkname1, x, y, z) 
  sbl2 = createSphere(linkname2,-stublength-sRadius,0,0,0.95*sRadius)
  sbj2 = createRigidJoint( linkname1, linkname2, 0, 0, 0)
  return sbc+sbl1+sbj1+sbl2+sbj2

def createBranch(headname, branchname,x,y,z, Nsegments):
  s = createBranchSegment(headname,branchname+str(0),x,y,z)
  for i in range(1,Nsegments):
    if i==1:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-stublength-sRadius,0,0)
    else:
      s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-length-2*sRadius,0,0)
  return s

def createBranchBundle(headname, Nsegments, Nbranches):
  s=''
  tt = headradius / max(radius, sphere_scale*radius)
  if tt < 2.0:
    print("[warning] head radius too small, branches too big")

  scale = 0.7

  for i in range(0,Nbranches):
    y = scale*headradius*cos(i*2*pi/Nbranches)
    z = scale*headradius*sin(i*2*pi/Nbranches)
    s+=createBranch("head","branch_"+str(i),-headradius,y,z, Nsegments)

  ## CSpace structure:
  ## SE(3) ~ R^6 (local chart) 
  ## + 1 rigid fixed joint eye-head
  ## + Nbranches*(Nsegments-1) spherical joints /w ## limits ~ (R^2)
  ## + Nbranches*(1+Nsegments) rigid fixed links

  Njoints = 6 + 1 + Nbranches * 2*(Nsegments-1) + Nbranches*(1+Nsegments)
  print( "[default position config]")
  print( "config=\""+str(Njoints)+" "+" 0"*Njoints+"\"")
  print( "[arms open config]")
  global config
  config = str("config=\"")

  x=0
  y=0
  z=0
  config += str(Njoints) + " "
  config += str(x) + " " + str(y) + " " + str(z)
  config += str(" 0"*3) ## SE(3)
  if Nsegments>0:
    config += str(" 0"*1) ## head-eye

    ## create a nice bouquet of branches
    for i in range(0,Nbranches):
      config += str(" 0"*2) ## first segment is fixed
      y = cos(i*2*pi/Nbranches)
      z = sin(i*2*pi/Nbranches)

      thetaZ = atan2(abs(y),abs(z))
      thetaY = pi/2-atan2(abs(y),abs(z))

      if y>0:
        thetaZ *= -1
      if z<0:
        thetaY *= -1

      for j in range(1,Nsegments):
        config += " "+str(aperture*thetaZ)+" "+str(aperture*thetaY)
        config+= " 0"

      #config+= " 0"*Nsegments ## shoulder of branch
  config+= "\""
  print( config)
  return s
def CreateSentinelRobot( robot_name, Nsegments, Nbranches):
  fname = getPathname(robot_name)
  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')
  headname = "head"
  f.write(createHead(headname))
  if Nsegments > 0:
    f.write(createBranchBundle(headname, Nsegments, Nbranches))
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()

  print("\nCreated new file >>",fname)


robot_name = 'sentinel'
CreateSentinelRobot(robot_name, Nsegments=6, Nbranches=8)
robot_name = 'sentinel_head'
CreateSentinelRobot(robot_name, Nsegments=0, Nbranches=0)

