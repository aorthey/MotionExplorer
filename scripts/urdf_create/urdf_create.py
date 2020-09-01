import numpy as np
import os 
from math import cos,sin,pi

mass = 1
damping = 0.01
friction = 0.5
effort = 0.01
velocity=100

def createMesh(lname,x=0,y=0,z=0,COLLISION_ENABLED=True):
  mass = 1
  Ixx = mass
  Iyy = mass
  Izz = mass
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="'+str(mass)+'"/>\n'
  s+= '    <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  s+= '    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <mesh filename="'+lname+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </visual>\n'
  if COLLISION_ENABLED:
    s+= '  <collision>\n'
    s+= '    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
    s+= '    <geometry>\n'
    s+= '      <mesh filename="'+lname+'"/>\n'
    s+= '    </geometry>\n'
    s+= '  </collision>\n'
    s+= ' </link>\n\n'
  return s

def createCuboid(lname,x,y,z,l,w,h,COLLISION_ENABLED=True):
  Ixx = mass*(1.0/12.0)*(h*h + l*l)
  Iyy = mass*(1.0/12.0)*(w*w + l*l)
  Izz = mass*(1.0/12.0)*(w*w + h*h)
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="'+str(mass)+'"/>\n'
  s+= '    <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  s+= '    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <box size="'+str(l)+' '+str(w)+' '+str(h)+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </visual>\n'
  if COLLISION_ENABLED:
    s+= '  <collision>\n'
    s+= '    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
    s+= '    <geometry>\n'
    s+= '      <box size="'+str(l)+' '+str(w)+' '+str(h)+'"/>\n'
    s+= '    </geometry>\n'
    s+= '  </collision>\n'
    s+= ' </link>\n\n'
  return s

def createRotatedCuboid(lname,x,y,z,l,w,h,r,p,yaw,COLLISION_ENABLED=True):
  Ixx = mass*(1.0/12.0)*(h*h + l*l)
  Iyy = mass*(1.0/12.0)*(w*w + l*l)
  Izz = mass*(1.0/12.0)*(w*w + h*h)
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="'+str(mass)+'"/>\n'
  s+= '    <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  # s+= '    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <origin rpy="'+str(r)+' '+str(p)+' '+str(yaw)+'" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <box size="'+str(l)+' '+str(w)+' '+str(h)+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </visual>\n'
  if COLLISION_ENABLED:
    s+= '  <collision>\n'
    # s+= '    <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
    s+= '    <origin rpy="'+str(r)+' '+str(p)+' '+str(yaw)+'" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
    s+= '    <geometry>\n'
    s+= '      <box size="'+str(l)+' '+str(w)+' '+str(h)+'"/>\n'
    s+= '    </geometry>\n'
    s+= '  </collision>\n'
    s+= ' </link>\n\n'
  return s

def createCylinder(lname,x,y,z,radius,length,COLLISION_ENABLED=True):
  Ixx = Iyy = (1.0/12.0)*mass*length*length + (1.0/4.0)*mass*radius*radius
  Izz = (1.0/4.0)*mass*radius*radius
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="'+str(mass)+'"/>\n'
  s+= '    <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  s+= '    <origin rpy="0 1.57 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
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
def createRotatedCylinder(lname,x,y,z,r,p,yaw,radius,length,COLLISION_ENABLED=True):
  Ixx = Iyy = (1.0/12.0)*mass*length*length + (1.0/4.0)*mass*radius*radius
  Izz = (1.0/4.0)*mass*radius*radius
  s= ' <link name="'+lname+'">\n'
  s+= '  <inertial>\n'
  s+= '    <mass value="'+str(mass)+'"/>\n'
  s+= '    <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
  s+= '  </inertial>\n'
  s+= '  <visual>\n'
  s+= '    <origin rpy="'+str(r)+' '+str(p)+' '+str(yaw)+'" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+= '    <geometry>\n'
  s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
  s+= '    </geometry>\n'
  s+= '  </visual>\n'
  if COLLISION_ENABLED:
    s+= '  <collision>\n'
    s+= '    <origin rpy="'+str(r)+' '+str(p)+' '+str(yaw)+'" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
    s+= '    <geometry>\n'
    s+= '      <cylinder length="'+str(length)+'" radius="'+str(radius)+'"/>\n'
    s+= '    </geometry>\n'
    s+= '  </collision>\n'
    s+= ' </link>\n\n'
  return s

def createEmptyLink(lname,x,y,z):
  Ixx = Iyy = Izz = (2.0/5.0)*mass
  s=''
  s+='<link name="'+lname+'" physical="0">\n'
  s+='  <inertial>\n'
  s+='   <mass value="'+str(mass)+'"/>\n'
  s+='   <inertia ixx="'+str(Ixx)+'" ixy="0" ixz="0" iyy="'+str(Iyy)+'" iyz="0" izz="'+str(Izz)+'"/>\n'
  s+='  </inertial>\n'
  s+='</link>\n\n'
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

def createRigidJoint(parentname, childname, x=0, y=0, z=0, r=0, p=0, yaw=0, prefix=""):
  jname = prefix+"joint_fixed_"+parentname+"_"+childname
  s= ''
  s+='<joint name="'+jname+'" type="fixed">\n'
  s+='  <origin rpy="'+str(r)+' '+str(p)+' '+str(yaw)+'" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='</joint>\n\n'
  return s

def createPrismaticJoint(parentname, childname, x=0, y=0, z=0, lowerLimit=0, upperLimit=1):
  jname = "joint_prismatic_"+parentname+"_"+childname
  s= ''
  s+='<joint name="'+jname+'" type="prismatic">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  return s

def comment(text):
  s= '<!--'+'#'*76+'\n'
  s+=text+'\n'
  s+= "#"*78
  s+='-->\n'
  return s
def commentNewBranch(bname):
  s= '<!--'+'#'*76+'\n'
  s+=bname+'\n'
  s+= "#"*78
  s+='-->\n'
  return s

def createRevoluteJoint(parentname, childname, x=0, y=0, z=0, lowerLimit=-1.57, upperLimit=1.57):
  return createRevoluteJointXYZ(parentname, childname, 0, 0, 1, x, y, z, lowerLimit, upperLimit)

def createRevoluteJointZ(parentname, childname, x=0, y=0, z=0, lowerLimit=-1.57, upperLimit=1.57):
  return createRevoluteJointXYZ(parentname, childname, 0, 0, 1, x, y, z, lowerLimit, upperLimit)
def createRevoluteJointY(parentname, childname, x=0, y=0, z=0, lowerLimit=-1.57, upperLimit=1.57):
  return createRevoluteJointXYZ(parentname, childname, 0, 1, 0, x, y, z, lowerLimit, upperLimit)
def createRevoluteJointX(parentname, childname, x=0, y=0, z=0, lowerLimit=-1.57, upperLimit=1.57):
  return createRevoluteJointXYZ(parentname, childname, 1, 0, 0, x, y, z, lowerLimit, upperLimit)

def createRevoluteJointRPY_XYZ(parentname, childname, r, p, yaw, ex, ey, ez, x=0, y=0, z=0, lowerLimit=-1.57, upperLimit=1.57):
  jname = "joint_revolute_"+parentname+"_"+childname
  s= ''
  s+='<joint name="'+jname+'_Z" type="revolute">\n'
  s+='  <origin rpy="'+str(r)+' '+str(p)+' '+str(yaw)+'" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <axis xyz="'+str(ex)+' '+str(ey)+' '+str(ez)+'"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="'+str(friction)+'"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  return s
def createRevoluteJointXYZ(parentname, childname, ex, ey, ez, x=0, y=0, z=0, lowerLimit=-1.57, upperLimit=1.57):
  jname = "joint_revolute_"+parentname+"_"+childname
  s= ''
  s+='<joint name="'+jname+'_Z" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <axis xyz="'+str(ex)+' '+str(ey)+' '+str(ez)+'"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="'+str(friction)+'"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  return s

def createSphericalJoint(parentname, childname, x=0, y=0, z=0, lowerLimit=-1.57,upperLimit=1.57):
  jname = "joint_spherical_"+parentname+"_"+childname
  tmpname = parentname+"_spherical_joint_link"
  s= ''
  s+='<joint name="'+jname+'_Z" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="'+str(x)+' '+str(y)+' '+str(z)+'"/>\n'
  s+='  <parent link="'+parentname+'"/>\n'
  s+='  <child link="'+tmpname+'"/>\n'
  s+='  <axis xyz="0 0 1"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="'+str(friction)+'"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  s+= createSphere(tmpname,0,0,0,0.001,PHYSICAL=False)
  s+='<joint name="'+jname+'_Y" type="revolute">\n'
  s+='  <origin rpy="0 0 0" xyz="0 0 0"/>\n'
  s+='  <parent link="'+tmpname+'"/>\n'
  s+='  <child link="'+childname+'"/>\n'
  s+='  <axis xyz="0 1 0"/>\n'
  s+='  <dynamics damping="'+str(damping)+'" friction="'+str(friction)+'"/>\n'
  s+='  <limit lower="'+str(lowerLimit)+'" upper="'+str(upperLimit)+'" effort="'+str(effort)+'" velocity="'+str(velocity)+'"/>\n'
  s+='</joint>\n\n'
  return s

def getPathname(robot_name, folder_name=None):
  fname = robot_name+'.urdf'
  pathname = os.path.dirname(os.path.realpath(__file__))+'/../../data/robots'
  if folder_name is None:
    pathname = os.path.abspath(pathname)+'/'
  else:
    pathname = os.path.abspath(pathname)+'/'+folder_name+'/'
  fname = pathname + robot_name + '.urdf'
  if not os.path.exists(os.path.dirname(fname)):
    os.makedirs(os.path.dirname(fname))
  return fname
