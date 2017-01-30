import numpy as np
from math import cos,sin,pi

def createCylinder(lname,x,y,z,radius,length):
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="1"/>\n'
  s+= '    <inertia ixx="0.005" ixy="0.001" ixz="0" iyy="0.006" iyz="0" izz="0.007"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  s+= '    <origin rpy="0 1.54 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </visual>\n'
  s+= '  <collision>\n'
  s+= '    <origin rpy="0 1.54 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </collision>\n'
  s+= ' </link>\n\n'
  return s

def createSphere(lname,x,y,z,radius):
  s=''
  s+='<link name="'+lname+'">\n'
  s+='  <inertial>\n'
  s+='  <mass value="1"/>\n'
  s+='  <inertia ixx="0.005" ixy="0.001" ixz="0" iyy="0.006" iyz="0" izz="0.007"/>\n'
  s+='  </inertial>\n'
  s+='  <collision>\n'
  s+='    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='    <geometry>\n'
  s+='      <sphere radius="'+str(radius)+'"/>\n'
  s+='    </geometry>\n'
  s+='  </collision>\n'
  s+='  <visual>\n'
  s+='    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='    <geometry>\n'
  s+='      <sphere radius="'+str(radius)+'"/>\n'
  s+='    </geometry>\n'
  s+='  </visual>\n'
  s+='</link>\n\n'
  return s

def createRigidJoint(jname, parentname, childname, x=0, y=0, z=0):
  s= ''
  s+='<joint name="'+jname+'" type="fixed">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='</joint>\n\n'
  return s

def commentNewBranch(bname):
  s= '<!--'+'#'*76+'\n'
  s+=bname+'\n'
  s+= "#"*78
  s+='-->\n'
  return s
def createRevoluteJoint(jname, parentname, childname, x=0, y=0, z=0):
  s= ''
  s+='<joint name="'+jname+'" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <axis xyz="0 1 0"/>\n'
  s+='  <dynamics damping="2" friction="0"/>\n'
  s+='  <limit lower="-1.57" upper="1.57" effort="10" velocity="1000"/>\n'
  s+='</joint>\n\n'
  return s
