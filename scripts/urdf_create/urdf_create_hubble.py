import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

l_body =3
r_body = 0.5
l_connector = 1.5
r_connector = 0.05
l_wing  = 4
w_wing  = 0.01
w_lid = 0.05
r_hinge = r_body/10.0
l_hinge = r_body/2.0


def GetXMLString(robot_name, with_lid=True):
  hstr   = '<?xml version="1.0"?>\n'
  hstr  += '<robot name="'+robot_name+'">\n'
  hstr  += createCylinder("body1" , 0, 0, 0, r_body, l_body)
  hstr  += createCylinder("body2", -l_body/3.0, 0, 0, 1.3*r_body, l_body/3.0)

  hstr  += createRotatedCylinder("connector1", 0, -r_body-l_connector/2.0, 0,
      pi/2.0,
      0, 0, r_connector, l_connector)
  hstr  += createRotatedCylinder("connector2", 0, +r_body+l_connector/2.0, 0,
      pi/2.0,
      0, 0, r_connector, l_connector)

  hstr  += createCuboid("wing1", 0, -r_body-2.0*l_connector/3.0, 0,
      w_wing, 2.0*l_connector/3.0, l_wing)
  hstr  += createCuboid("wing2", 0, +r_body+2.0*l_connector/3.0, 0,
      w_wing, 2.0*l_connector/3.0, l_wing)

  if with_lid:
    hstr  += createRotatedCylinder("lid_hinge1", l_body/2.0, 0, r_body,
        pi/2.0,
        0, 
        0,
        r_hinge, l_hinge)
    hstr  += createRotatedCylinder("lid1", w_lid/2.0+0.01, 0, -r_body,
        0,
        pi/2.0,
        0, 
        r_body, w_lid)

  hstr += createRigidJoint( "body1", "body2")
  hstr += createRigidJoint( "body1", "connector1")
  hstr += createRigidJoint( "body1", "connector2")
  hstr += createRevoluteJointY("joint_"+"connector1"+"_"+"wing1", "connector1", "wing1")
  hstr += createRevoluteJointY("joint_"+"connector2"+"_"+"wing2", "connector2", "wing2")
  if with_lid:
    hstr += createRigidJoint( "body1", "lid_hinge1")
    hstr += createRevoluteJointY("joint_"+"lid_hinge1"+"_"+"lid1", "lid_hinge1",
    "lid1", x=l_body/2.0, z=r_body, lowerLimit=-2.0, upperLimit=0)
  hstr += '  <klampt package_root="../../.." default_acc_max="4" >\n'
  hstr += '  </klampt>\n'
  hstr += '</robot>'
  return hstr

robot_name = 'hubble/hubble'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write(GetXMLString(robot_name))
f.close()
print "\nCreated new file >>",fname

### create nested robots
CreateSphereRobot(robot_name + "_sphere_inner", r_body)
CreateSphereRobot(robot_name + "_sphere_outer", l_body)
CreateCylinderRobot(robot_name + "_capsule_inner", r_body, l_body)
CreateCylinderRobot(robot_name + "_capsule_outer", l_body, l_body)

robot_name = robot_name+'_without_lid'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write(GetXMLString(robot_name,with_lid=False))
f.close()
print "\nCreated new file >>",fname

