import numpy as np
from math import cos,sin,pi

mass = 1
damping = 0.1
effort = 0.01
velocity=100


def createCylinder(lname,x,y,z,radius,length,COLLISION_ENABLED=True):
  Ixx = Iyy = (1.0/12.0)*mass*length*length + (1.0/4.0)*mass*radius*radius
  Izz = (1.0/4.0)*mass*radius*radius
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="'+str(mass)+'"/>\n'
  s+= '    <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  s+= '    <origin rpy="0 1.54 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </visual>\n'
  if COLLISION_ENABLED:
    s+= '  <collision>\n'
    s+= '    <origin rpy="0 1.57 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
    s+= '    <geometry>\n'
    s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
    s+= '    </geometry>\n'
    s+= '  </collision>\n'
    s+= ' </link>\n\n'
  return s

def createSphere(lname,x,y,z,radius,PHYSICAL=True):
  Ixx = Iyy = Izz = (2.0/5.0)*mass*radius*radius
  s=''
  if PHYSICAL:
    s+='<link name="'+lname+'">\n'
  else:
    s+='<link name="'+lname+'" physical="0">\n'
  s+='  <inertial>\n'
  s+='   <mass value="'+str(mass)+'"/>\n'
  s+='   <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
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

def createRevoluteJoint(jname, parentname, childname, x=0, y=0, z=0, lowerLimit=-1.57, upperLimit=1.57):
  s= ''
  s+='<joint name="'+jname+'_Z" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <axis xyz="0 0 1"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="0"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  return s

def createSphericalJoint(jname, parentname, childname, x=0, y=0, z=0, lowerLimit=-1.57,upperLimit=1.57):
  tmpname = parentname+"_spherical_joint_link"
  s= ''
  s+='<joint name="'+jname+'_Z" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+tmpname+'"/>\n'
  s+='  <axis xyz="0 0 1"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="0"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  s+= createSphere(tmpname,0,0,0,0.001,PHYSICAL=False)
  s+='<joint name="'+jname+'_Y" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="0 0 0"/>\n'
  s+='  <parent link="'+tmpname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <axis xyz="0 1 0"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="0"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  return s
