import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

L1 = 1.5
L2 = 1.5
thickness = 0.2
thickness_Z = 0.1

class LinkageCreator(object):

  def Add1DOFLinkage(self, current_link, build_endeffector = False):
    linkname = "link"+str(current_link)
    jointname1 = "jlink"+str(current_link)
    jointname2 = "jlink"+str(current_link+1)
    hstr = ""
    if current_link == 0:
      hstr += createRotatedCylinder(jointname1, 0,0,0,0,0,1.57,thickness,thickness_Z)

    hstr += createCuboid(linkname,0,L1/2+thickness,0,  2*thickness,L1,thickness_Z)
    hstr += createRevoluteJoint(jointname1, linkname, 0,0,0, lowerLimit = -3.14, upperLimit = 3.14)

    if build_endeffector:
      endeff_width = 0.5*thickness
      endeff_length = 2*thickness
      hstr += createCuboid("endeffectorL",-thickness-0.5*endeff_width,0.25*endeff_length,0,
          endeff_width,endeff_length,thickness_Z)
      hstr += createCuboid("endeffectorR",+thickness+0.5*endeff_width,0.25*endeff_length,0,
          endeff_width,endeff_length,thickness_Z)
      hstr += createRigidJoint(linkname, "endeffectorL", 0, L2+thickness, 0)
      hstr += createRigidJoint(linkname, "endeffectorR", 0, L2+thickness, 0)
    else:
      hstr += createRotatedCylinder(jointname2, 0,0,0,0,0,1.57,thickness,thickness_Z)
      hstr += createRigidJoint(linkname, jointname2, 0, L1+2*thickness, 0)
    return hstr

  def GetNdofLinkage(self, N, build_endeffector=False):
    current_link = 0
    hstr = ""
    if build_endeffector:
      N = N-1

    for i in range(0,N):
      hstr += self.Add1DOFLinkage(current_link)
      current_link = current_link + 1

    if build_endeffector:
      hstr += self.Add1DOFLinkage(current_link, build_endeffector)
    return hstr

  def CreateNdofLinkage(self, robot_name, links, build_endeffector=False):
    fname = getPathname(robot_name)

    f = open(fname,'w')
    f.write('<?xml version="1.0"?>\n')
    f.write('<robot name="'+robot_name+'">\n')

    hstr = self.GetNdofLinkage(links, build_endeffector)

    f.write(hstr)
    f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
    f.write('  </klampt>\n')
    f.write('</robot>')
    f.close()

    print "\nCreated new file >>",fname


linkage_creator = LinkageCreator()
robot_name = 'planar_manipulator/2dof_manip'
linkage_creator.CreateNdofLinkage(robot_name, 2, build_endeffector=True)
robot_name = 'planar_manipulator/2dof_manip_1dof_inner'
linkage_creator.CreateNdofLinkage(robot_name, 1)

N = {3,4,5,6,7,8,9,10,11,12,13,14}
for n in N:
  robot_name = 'planar_manipulator/'+str(n)+'dof_manip'
  linkage_creator.CreateNdofLinkage(robot_name, n)

robot_name = 'planar_manipulator/'+str(15)+'dof_manip'
linkage_creator.CreateNdofLinkage(robot_name, 15, build_endeffector=True)
