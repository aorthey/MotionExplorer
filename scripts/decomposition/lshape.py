#!/usr/bin/python3.7
import pybullet as p
import pybullet_data as pd
import os
import sys
import pathlib
import trimesh
from common import *

scale = 0.05
scale_relaxation = 0.7
path = pathlib.Path(__file__).parent.absolute()

#######################################################################
# STEP1: Read Obj file and do convex decompose using VHACD
#######################################################################

part = os.path.join(path, "../../data/terrains/narrow_passage/narrow_passage_0_70.stl") 
folder, name_in_file = os.path.split(os.path.realpath(part))

mesh = trimesh.load(part, skip_materials=True)

filename = "narrow_passage_0_70"
file_extension = ".obj"
ctr = 0
for m in mesh.split():
  name_m = folder + "/meshes/" + filename + "-" + str(ctr) + file_extension
  m.export(name_m, "obj")
  print(name_m)
  ctr = ctr + 1

