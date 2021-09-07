#!/usr/bin/python3.7
import os
import sys
import pathlib
import trimesh
from common import *

path = pathlib.Path(__file__).parent.absolute()

#######################################################################
# STEP1: Read Obj file and apply transforms
#######################################################################
# part = os.path.join(path, "../../../brickr/berlyn.obj")
# folder, name_in_file = os.path.split(os.path.realpath(part))

# mesh = trimesh.load(part, skip_materials=True)
# mesh.apply_translation(-mesh.centroid)
# mesh.apply_scale(0.1)
# R = trimesh.transformations.rotation_matrix(-0.5*np.pi, [1, 0, 0])
# mesh.apply_transform(R)

# fname = "berlyn_rotated.obj"
# mesh.export(folder + "/" + fname)

#######################################################################
# STEP2: Apply binvox -d resolution to exported object (Externally for now)
#######################################################################
# os.system("binvox/linux/binvox -d 30 "+fname)

#######################################################################
# STEP3: Load in binvox object
#######################################################################
part = os.path.join(path, "../../../brickr/berlyn_rotated30.obj")

mesh = trimesh.load(part, skip_materials=True)
filename = "bgate"
file_extension = ".obj"
ctr = 0
for m in mesh.split():
  name_m = folder + "/meshes/" + filename + "-" + str(ctr) + file_extension
  # m.apply_translation(-m.centroid)
  m.export(name_m, "obj")
  print(name_m, " position:" + m.centroid)
  ctr = ctr + 1

