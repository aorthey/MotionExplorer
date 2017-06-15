import os 
import numpy as np
import sys
from math import cos,sin,pi,atan2
from urdf_create import *


robot_names = ['snake','snake_irreducible']
env_names = ['twister/twister.tri','underwater/underwater.tri']
Nsegments_vec = [1,1]

length = 0.1
limit = pi/4
stublength = length/4
radius = 0.05
headradius = 0.1
kappa = (2*sin(limit))/(length*Nsegments_vec[0])

for k in range(0,len(robot_names)):

  robot_name = robot_names[k]

  lowerLimit=-limit
  upperLimit=limit
  radius_cylinder = headradius-headradius/4

  Nsegments = Nsegments_vec[k]
  print robot_name,Nsegments_vec[k]
  #sys.exit(0)

  sRadius = 2*radius
  config = ''
  folder=''
  fname = folder+robot_name+'.urdf'
  pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
  pathname = os.path.abspath(pathname)+'/'
  fname = pathname + fname

  f = open(fname,'w')

  f.write('<?xml version="1.0"?>\n')
  f.write('<robot name="'+robot_name+'">\n')

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
    s += createBranchSegment(headname,branchname+str(0),-headradius-0.01,0,0)
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

  print "\nCreated new robot >>",
  print fname

  for env_name in env_names:

    base=os.path.basename(env_name)
    env_singleword = os.path.splitext(base)[0]
    xmlname = folder+robot_name+'_'+env_singleword+'.xml'

    xmlname = pathname + xmlname

    f = open(xmlname,'w')
    f.write('<?xml version="1.0"?>\n\n')
    f.write('<world>\n')

    robotstr  = '  <robot name=\"'+robot_name+'\"'
    robotstr += ' file="'+str(fname)+'"'
    robotstr += ' translation="0 0 0"'
    robotstr += ' rotateRPY="0 0 0"'
    robotstr += ' '+config+'/>\n\n'
    f.write(robotstr)

    cmmntstr = '  <!-- Irreducible Curvature kappa ='+str(kappa)+' -->\n\n'
    f.write(cmmntstr)

    terrainstr  = '  <rigidObject '
    terrainstr += ' name=\"'+str(env_name)+'\"'
    terrainstr += ' file=\"/home/aorthey/git/orthoklampt/data/terrains/'+str(env_name)+'\"'
    terrainstr += ' translation="0 0 0"/>\n\n'
    f.write(terrainstr)

    Njoints = 6 + 1 + 2*(Nsegments-1) + (1+Nsegments)
    plannersettingsstr  = '  <plannersettings>\n\n'
    plannersettingsstr += '    <qinit config="'+str(Njoints)+'  -3.3 0 0'+(Njoints-3)*" 0"+'"/>\n'
    plannersettingsstr += '    <qgoal config="'+str(Njoints)+'  5.2 0 0'+(Njoints-3)*" 0"+'"/>\n'
    plannersettingsstr += '    <se3min config="6  -6 -6 -1 -3.141592 -1.57 -3.14"/>\n'
    plannersettingsstr += '    <se3max config="6  6 6 16 3.141592 1.57 3.14"/>\n\n'
    plannersettingsstr += '  </plannersettings>\n\n'
    f.write(plannersettingsstr)

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

print "Curvature: ",kappa
