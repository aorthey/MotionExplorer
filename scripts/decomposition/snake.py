#!/usr/bin/python3.7
import pybullet as p
import pybullet_data as pd
import os
import sys
import pathlib
import trimesh
from common import *

scale = 0.1
#######################################################################
# STEP1: Read Obj file and do convex decompose using VHACD
#######################################################################
path = pathlib.Path(__file__).parent.absolute()

part = os.path.join(path, "../../data/objects/puzzles/snake/snake.obj")

folder, name_in_file = os.path.split(os.path.realpath(part))

### Create centered + scaled version
mesh = trimesh.load(part, skip_materials=True)
# mesh1.apply_translation(-mesh1.centroid)
# mesh1.apply_scale(scale)
# name_scaled = folder + "/meshes/" + "snake1-scaled.obj"
# mesh1.export(name_scaled, "obj", write_texture=False, include_texture=False)

# name_urdf = folder + "/snake.urdf"
# Objs2URDF(name_urdf, names)
scale = False
names = []

filename = "snake"
file_extension = ".obj"
ctr = 0
for m in mesh.split():
  name_m = folder + "/meshes/" + filename + "-" + str(ctr) + file_extension
  m.export(name_m, "obj")
  ctr = ctr + 1
  names.append(name_m)

name_urdf = folder + "/snake.urdf"
Objs2URDF(name_urdf, names)
