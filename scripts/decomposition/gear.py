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

part = os.path.join(path, "../../data/objects/puzzles/gear/gear.obj")
folder, name_in_file = os.path.split(os.path.realpath(part))

mesh = trimesh.load(part, skip_materials=True)
mesh.apply_translation(-mesh.centroid)
R = trimesh.transformations.rotation_matrix(0.5*np.pi, [1, 0, 0])
mesh.apply_transform(R)

filename = "gear"
file_extension = ".obj"
name_obj = folder + "/meshes/" + filename + file_extension
mesh.export(name_obj, "obj")

# m.export(name_m, "obj")
name_urdf = folder + "/" + filename + ".urdf"
Objs2URDF(name_urdf, [name_obj])

