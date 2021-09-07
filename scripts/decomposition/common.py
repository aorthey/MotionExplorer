import os
import sys
sys.path.append('../urdf_create/')
from urdf_create import *

def Objs2URDF(name_in, obj_filenames):
  urdfStr =  '<?xml version=\"1.0\"?>\n'
  urdfStr += '<robot name=\"'+os.path.basename(name_in)+'\">\n'

  lastFileName = ""
  for name in obj_filenames:
    filename = "meshes/" + os.path.basename(name)
    urdfStr += createMesh(filename)
    if lastFileName != "":
      urdfStr += createRigidJoint(filename, lastFileName)
    lastFileName = filename

  urdfStr += '<klampt package_root=\"../../..\" default_acc_max=\"4\" >\n'

  for k in range(0,len(obj_filenames)):
    for j in range(k,len(obj_filenames)):
      n1 = "meshes/" + os.path.basename(obj_filenames[k])
      n2 = "meshes/" + os.path.basename(obj_filenames[j])
      if n1 != n2:
        urdfStr += '<noselfcollision pairs=\"' +n1+" "+n2+ '\" />\n'
  urdfStr += '</klampt>\n'
  urdfStr += '</robot>\n'

  f = open(name_in,'w')
  f.write(urdfStr)
  f.close()
  print("Wrote URDF to ", name_in)
  return urdfStr

