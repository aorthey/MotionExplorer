import numpy as np
from math import cos,sin,pi

mass = 0.05
damping = 0.5
effort = 0.01
def createCylinder(lname,x,y,z,radius,length,COLLISION_ENABLED=True):
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="'+str(mass)+'"/>\n'
  s+= '    <inertia ixx="0.005" ixy="0.001" ixz="0" iyy="0.006" iyz="0" izz="0.007"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  s+= '    <origin rpy="0 1.54 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </visual>\n'
  if COLLISION_ENABLED:
    s+= '  <collision>\n'
    s+= '    <origin rpy="0 1.54 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
    s+= '    <geometry>\n'
    s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
    s+= '    </geometry>\n'
    s+= '  </collision>\n'
  s+= ' </link>\n\n'
  return s

def createSphere(lname,x,y,z,radius,PHYSICAL=True):
  s=''
  if PHYSICAL:
    s+='<link name="'+lname+'">\n'
  else:
    s+='<link name="'+lname+'" physical="0">\n'
  s+='  <inertial>\n'
  s+='  <mass value="'+str(mass)+'"/>\n'
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
  s+='  <dynamics damping="'+str(damping)+'" friction="0"/>\n'
  s+='  <limit lower="-1.57" upper="1.57" effort="'+str(effort)+'" velocity="1000"/>\n'
  s+='</joint>\n\n'
  return s

def createSphericalJoint(jname, parentname, childname, x=0, y=0, z=0):
  tmpname = parentname+"_spherical_joint_link"
  s= ''
  s+='<joint name="'+jname+'_Y" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+tmpname+'"/>\n'
  s+='  <axis xyz="0 1 0"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="0"/>\n'
  s+='  <limit lower="-1.57" upper="1.57" effort="'+str(effort)+'" velocity="1000"/>\n'
  s+='</joint>\n\n'
  s+= createSphere(tmpname,0,0,0,0.01,PHYSICAL=False)
  s+='<joint name="'+jname+'_X" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="0 0 0"/>\n'
  s+='  <parent link="'+tmpname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <axis xyz="0 0 1"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="0"/>\n'
  s+='  <limit lower="-1.57" upper="1.57" effort="'+str(effort)+'" velocity="1000"/>\n'
  s+='</joint>\n\n'
  return s
