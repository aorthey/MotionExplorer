import numpy as np
import os
import sys

def sample_spherical(npoints, ndim=3):
  vec = np.random.randn(ndim, npoints)
  vec /= np.linalg.norm(vec, axis=0)
  return vec

def CreateSphereXML(name, start, goal, objs):
  f = open(xml_name, 'w')

  f.write("<?xml version=\"1.0\"?>\n")
  f.write("\n")
  f.write("<world>\n")
  f.write("\n")

  f.write("<robot name=\"point\" file=\"../../../../robots/point_nonphysical.urdf\"/>\n")
  f.write("\n")

  for obj in objs:
    f.write("<rigidObject name=\"cylinder_difference\" \
        file=\"../../../../terrains/primitives/sphere.tri\"  \
      translation=\"{} {} {}\"/>".format(obj.pos[0],obj.pos[1],obj.pos[2]))
    f.write("\n")

  f.write("\n")
  f.write("<plannerinput>\n")
  f.write("\n")

  s = start.pos
  g = goal.pos
  f.write("  <qinit  config=\"6  {} {} {} 0 0 0\"/>\n".format(s[0],s[1],s[2]))
  f.write("  <qgoal  config=\"6  {} {} {} 0 0 0 \"/>\n".format(g[0],g[1],g[2]))
  f.write("  <se3min config=\"6  -3 -3 -3 -3.141593 -1.57 -3.141593\"/>\n")
  f.write("  <se3max config=\"6  +3 +3 +3 +3.141593 +1.57 +3.141593\"/>\n")

  f.write("  <epsilongoalregion>0.01</epsilongoalregion>\n")
  f.write("  <freeFloating>1</freeFloating>\n")

  f.write("  <algorithm name=\"benchmark:review2021_manifolds\"/>\n")
  # f.write("  <algorithm name=\"ompl:rrtconnect\"/>\n")
  f.write("  <multilevel>\n")
  f.write("    <level robot_index=\"0\" type=\"SPHERE\"/>\n")
  f.write("  </multilevel>\n")

  f.write("</plannerinput>\n")
  f.write("\n")
  f.write("</world>\n")
  f.write("\n")
  f.close()
  print("Created xml file "+xml_name)


class SphereObject:
  pos = np.array((3))
  sid = -1

def Valid(v1, v2, distance=0.7):
  return np.linalg.norm(v1.pos - v2.pos) > distance

def ValidConfig(start, goal, objs):
  if not Valid(start, goal, 1.0):
    return False

  for obj in objs:
    if not Valid(start, obj):
      return False
    if not Valid(goal, obj):
      return False

  for obj1 in objs:
    for obj2 in objs:
      if obj1 != obj2:
        if not Valid(obj1, obj2):
          return False
  return True

#Attention: some seeds will block the passage between goal and start
np.random.seed(seed=0) 

for k in range(0,10):
  start = SphereObject()
  goal = SphereObject()
  start.pos = sample_spherical(1).flatten()
  goal.pos = -start.pos

  Nobjs = 10
  objs = []
  for sid in range(0, Nobjs):
    s = SphereObject()
    s.sid = sid
    s.pos = start.pos
    objs.append(s)
    while not ValidConfig(start, goal, objs):
      s.pos = sample_spherical(1).flatten()

  xml_name = "02D_Sphere_0"+str(k)+".xml"
  CreateSphereXML(xml_name, start, goal, objs)
  # os.system("cat %s"% xml_name)
