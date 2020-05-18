import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *

length = 0.15
radius = 0.01
radius_cylinder = 0.02
radius_cylinder_stub = 0.02

sphere_scale = 2
headradius = 0.1
headlength = 0.2
stublength = length/2
# Nsegments = 6
# Nbranches = 8
aperture = 0.4 ## aperture of bouquet of branches
limit = pi/2

sRadiusStub = sphere_scale*radius
sRadius = sphere_scale*radius
config = ''

def createHead(headname):
  hstrs = createSphere("eye", 0, 0, 0, headradius)
  hstrc = createCylinder(headname, -headlength/2, 0, 0, headradius, headlength)
  hstrj = createRigidJoint("eye",headname)
  return hstrs+hstrc+hstrj

def createBranch(headname, branchname, x, y, z, Nsegments):
  s = createBranchSegment(headname, branchname+str(0),x,y,z)
  for i in range(1,Nsegments):
      s+= attachBranchSegment(branchname+str(i-1), branchname+str(i),
          0, y, z)
  return s

def createBranchSegment(parentlinkname, linkname, x, y, z):
  s = comment(linkname) 
  linkname1 = linkname+'_cylinder_stub'
  linkname2 = linkname+'_sphere_stub'
  linkname3 = linkname+'_cylinder'
  linkname4 = linkname

  s+= createCylinder(linkname1, -stublength/2, 0, 0, radius_cylinder_stub, stublength) 
  s+= createRigidJoint( parentlinkname, linkname1, x, y, z) 
  s+= createSphere(linkname2,-stublength-sRadiusStub, 0, 0, 0.95*sRadiusStub)
  s+= createRigidJoint( linkname1, linkname2, 0, 0, 0)

  n = np.sqrt(y*y+z*z)
  d = (length/2+sRadiusStub)/n
  theta = np.arctan2(y,z)

  s+= createRotatedCylinder(linkname3,
      -stublength-radius_cylinder_stub,d*y, d*z, 0, theta, 1.57, radius_cylinder_stub, length) 
  s+= createRigidJoint( linkname2, linkname3, 0, 0, 0)

  d = (length+2*sRadiusStub)/n
  # s+= createSphere(linkname4, -stublength-radius_cylinder, d*y, d*z, 0.95*sRadius)
  s+= createSphere(linkname4, 0,0,0, 0.95*sRadiusStub)
  s+= createRigidJoint( linkname3, linkname4, -stublength-radius_cylinder_stub, d*y,
      d*z)
  return s

def attachBranchSegment(parentlinkname, linkname, x, y, z):
  linkname1 = linkname+'_cylinder'
  linkname2 = linkname

  s = comment(linkname) 
  s+= createCylinder(linkname1, -length/2-sRadius, 0, 0, radius_cylinder, length) 
  # s+= createRevoluteJointZ(parentlinkname, linkname1, 0, 0, 0, lowerLimit=-limit, upperLimit=limit) 

  n = np.sqrt(y*y+z*z)
  s+= createRevoluteJointXYZ(parentlinkname, linkname1, 0, z/n, -y/n, 0, 0, 0, -limit, limit) 
  s+= createSphere(linkname2, 0, 0, 0, 0.95*sRadius)
  s+= createRigidJoint(linkname1, linkname2, -length-2*sRadius, 0, 0)

  return s


def createBranchBundle(headname, Nsegments, NsegmentsLast, Nbranches, NbranchesMax):
  s=''
  tt = headradius / max(radius, sphere_scale*radius)
  if tt < 2.0:
    print("[warning] head radius too small, branches too big")


  d = 0.7*headradius
  x = -headlength
  for i in range(0,Nbranches):
    y = d*cos(i*2*pi/NbranchesMax)
    z = d*sin(i*2*pi/NbranchesMax)
    if i < Nbranches-1:
      s+=createBranch("head","branch_"+str(i), x, y, z, Nsegments)
    else:
      s+=createBranch("head","branch_"+str(i), x, y, z, NsegmentsLast)

  ## CSpace structure:
  ## SE(3) ~ R^6 (local chart) 
  ## + 1 rigid fixed joint (head)
  ## + 4 rigid fixed joint *Nbranches (branch)
  ## + Nbranches*(Nsegments-1) spherical joints /w ## limits ~ (R^2)
  ## + Nbranches*(1+Nsegments) rigid fixed links

  Njoints = 6 + 1 + 4*Nbranches + 2*(Nsegments-1)*Nbranches
  print( "[default position config]")
  print( "config=\""+str(Njoints)+" "+" 0"*Njoints+"\"")
  print( "[arms open config]")
  global config
  config = str("config=\"")

  config += str(Njoints) + " "
  config += str(" 0 0 -2 0 1.57 0") ## SE(3)
  if Nsegments>0:
    config += str(" 0"*1) ## head-eye

    ## create a nice bouquet of branches
    for i in range(0,Nbranches):
      config += str(" 0"*4)

      for j in range(1,Nsegments):
        if j < 0.5*Nsegments:
          config+= " 0"
        else:
          config+= " 0.15"
        config+= " 0"

  config+= "\""
  print( config)
  return s

def CreateOctopus( robot_name, Nsegments, NsegmentsLast, Nbranches, NbranchesMax):
  fname = getPathname(robot_name)
  f = open(fname,'w')
  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')
  headname = "head"
  f.write(createHead(headname))
  if Nsegments > 0:
    f.write(createBranchBundle(headname, Nsegments, NsegmentsLast, Nbranches, NbranchesMax))
  f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
  f.write('  </klampt>\n')
  f.write('</robot>')
  f.close()

  print("\nCreated new file >>",fname)

Nbranches = 8


for i in range(0, Nbranches+1):
    robot_name = 'sentinel/octopus_'+str(i)+'_arms_3_segments'
    CreateOctopus(robot_name, Nsegments=7, NsegmentsLast=4, Nbranches=i,
        NbranchesMax=Nbranches)
    robot_name = 'sentinel/octopus_'+str(i)+'_arms'
    CreateOctopus(robot_name, Nsegments=7, NsegmentsLast=7, Nbranches=i,
        NbranchesMax=Nbranches)

