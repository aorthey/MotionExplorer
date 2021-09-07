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

part = os.path.join(path, "../../data/objects/puzzles/shapesorter/shape_pins.stl")
folder, name_in_file = os.path.split(os.path.realpath(part))

mesh = trimesh.load(part, skip_materials=True)
mesh.apply_translation(-mesh.centroid)
mesh.apply_scale(scale)

filename = "shapesorter"
file_extension = ".obj"
ctr = 0
for m in mesh.split():
  name_m = folder + "/meshes/" + filename + "-" + str(ctr) + file_extension
  m.apply_translation(-m.centroid)
  m.export(name_m, "obj")
  name_urdf = folder + "/" + filename + "-" + str(ctr) + ".urdf"
  Objs2URDF(name_urdf, [name_m])

  # m.apply_scale(scale_relaxation)
  # name_m = folder + "/meshes/" + filename + "-" + str(ctr) + "-relaxed" + file_extension
  # m.export(name_m, "obj")
  # name_urdf = folder + "/" + filename + "-" + str(ctr) + "-relaxed.urdf"
  # Objs2URDF(name_urdf, [name_m])

  ctr = ctr + 1

part = os.path.join(path, "../../data/objects/puzzles/shapesorter/box_lid.stl")
mesh = trimesh.load(part, skip_materials=True)
mesh.apply_translation(-mesh.centroid)
mesh.apply_scale(scale)
name_obj = folder + "/shapesorter_box_lid.obj"
mesh.export(name_obj, "obj")

# part = os.path.join(path, "../../data/objects/puzzles/shapesorter/box_base.stl")
# mesh = trimesh.load(part, skip_materials=True)
# mesh.apply_translation(-mesh.centroid)
# mesh.apply_scale(scale)
# name_obj = folder + "/shapesorter_box_base.obj"
# mesh.export(name_obj, "obj")

# part = os.path.join(path, "../../data/objects/puzzles/shapesorter/side_4__5.stl")
# mesh = trimesh.load(part, skip_materials=True)
# mesh.apply_translation(-mesh.centroid)
# mesh.apply_scale(scale)
# name_obj = folder + "/shapesorter_box_side3.obj"
# mesh.export(name_obj, "obj")
# name_obj = folder + "/shapesorter_box_side4.obj"
# mesh.export(name_obj, "obj")

# part = os.path.join(path, "../../data/objects/puzzles/shapesorter/side_2__3.stl")
# mesh = trimesh.load(part, skip_materials=True)
# mesh.apply_translation(-mesh.centroid)
# mesh.apply_scale(scale)
# name_obj = folder + "/shapesorter_box_side1.obj"
# mesh.export(name_obj, "obj")
# name_obj = folder + "/shapesorter_box_side2.obj"
# mesh.export(name_obj, "obj")
