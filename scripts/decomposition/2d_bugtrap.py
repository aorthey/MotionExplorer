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

in_part = os.path.join(path, "../../../3D-models/2d/bugtrap.stl")
out_folder = os.path.join(path, "../../data/terrains/puzzles/")
in_folder, name_in_file = os.path.split(os.path.realpath(in_part))

mesh = trimesh.load(in_part, skip_materials=True)

filename = "bugtrap2d"
file_extension = ".obj"
ctr = 0
for m in mesh.split():
  name_m = out_folder + filename + "-" + str(ctr) + file_extension
  m.export(name_m, "obj")
  print(name_m)
  ctr = ctr + 1

