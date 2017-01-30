import os 
import numpy as np
from math import cos,sin,pi
from urdf_create import *

length = 0.15
radius = 0.02
sphere_scale = 1.1
headradius = 0.1
Nsegments = 5
Nbranches = 7

fname = 'sentinel2.urdf'
pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
pathname = os.path.abspath(pathname)+'/'
fname = pathname + fname
print "\nCreated new file >>",
print fname

f = open(fname,'w')

f.write('<?xml version="1.0"?>')
f.write('<robot name="sentinel">')

def createHead(headname):
  hstrs = createSphere("eye",0,0,0,headradius)
  hstrc = createCylinder(headname,-headradius/2,0,0,headradius,headradius)
  hstrj = createRigidJoint("joint_eye_"+headname,"eye",headname)
  return hstrs+hstrc+hstrj

def attachBranchSegment(parentlinkname, linkname, x, y, z):
  sbl1 = createCylinder(linkname,x+length/2,y,z,radius,length) 
  sbj1 = createRevoluteJoint(parentlinkname+'_'+linkname+'_joint_revolute', parentlinkname,linkname, x, y, z) 
  sbl2 = createSphere(linkname+'_sphere',0,0,0,sphere_scale*radius) 
  sbj2 = createRigidJoint(linkname+'_fixed', linkname, linkname+'_sphere',x,y,z)
  return sbl1+sbj1+sbl2+sbj2

def createBranchSegment(parentlinkname, linkname, x, y, z):
  sbc = commentNewBranch(linkname) 
  sbl1 = createCylinder(linkname,-length/2,0,0,radius,length) 
  sbj1 = createRigidJoint(parentlinkname+'_'+linkname+'_joint', parentlinkname,linkname, x, y, z) 
  sbl2 = createSphere(linkname+'_sphere',-length,0,0,sphere_scale*radius) 
  sbj2 = createRigidJoint(linkname+'_fixed', linkname, linkname+'_sphere',0,0,0)
  return sbc+sbl1+sbj1+sbl2+sbj2

def createBranch(headname, branchname,x,y,z):
  s = createBranchSegment(headname,branchname+str(0),x,y,z)
  for i in range(1,Nsegments):
    s+= attachBranchSegment(branchname+str(i-1),branchname+str(i),-length,0,0)
  return s

def createBranchBundle(headname):
  s=''
  scale = 0.8
  for i in range(0,Nbranches):
    y = scale*headradius*cos(i*2*pi/Nbranches)
    z = scale*headradius*sin(i*2*pi/Nbranches)
    s+=createBranch("head","branch_"+str(i),-headradius,y,z)
  return s

headname = "head"
f.write(createHead(headname))
f.write(createBranchBundle(headname))

f.write('  <klampt package_root="../.." use_vis_geom="1" />')
f.write('</robot>')
f.close()
