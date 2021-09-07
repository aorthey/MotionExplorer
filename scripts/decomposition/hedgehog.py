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

part1 = os.path.join(path, "../../data/objects/puzzles/hedgehog/hedgehog1A.stl")
part2 = os.path.join(path, "../../data/objects/puzzles/hedgehog/hedgehog1B.stl")
env = os.path.join(path, "../../data/objects/puzzles/hedgehog/cage1.stl")

folder, name_in_file = os.path.split(os.path.realpath(part1))

### Create centered + scaled version
names = []
mesh1 = trimesh.load(part1, skip_materials=True)
mesh1.apply_translation(-mesh1.centroid)
mesh1.apply_scale(scale)
name_scaled = folder + "/meshes/" + "hedgehog1-scaled.obj"
names.append(name_scaled)
mesh1.export(name_scaled, "obj", write_texture=False, include_texture=False)

mesh2 = trimesh.load(part2, skip_materials=True)
mesh2.apply_translation(-mesh2.centroid)
mesh2.apply_translation([-0.15,0,0])
mesh2.apply_scale(scale)
R = trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0])
mesh2.apply_transform(R)
mesh2.apply_translation([0,0,-0.88])

name_scaled = folder + "/meshes/" + "hedgehog2-scaled.obj"
names.append(name_scaled)
mesh2.export(name_scaled, "obj", write_texture=False, include_texture=False)

name_urdf = folder + "/hedgehog.urdf"
Objs2URDF(name_urdf, names)

env = os.path.join(path, "../../data/objects/puzzles/hedgehog/cage1.stl")
mesh = trimesh.load(env, skip_materials=True)
# mesh.apply_translation(-mesh.centroid)
mesh.apply_scale(scale)
name_scaled = folder + "/" + "cage.obj"
mesh.export(name_scaled, "obj", write_texture=False, include_texture=False)
