#!/usr/bin/python3.7
import pybullet as p
import pybullet_data as pd
import os
import sys
import pathlib
import trimesh

sys.path.append('../urdf_create/')

scale = 0.1
from urdf_create import *
from common import *

def rotation_matrix_from_vectors(vec1, vec2):
  """ Find the rotation matrix that aligns vec1 to vec2
  :param vec1: A 3d "source" vector
  :param vec2: A 3d "destination" vector
  :return mat: A transform matrix (3x3) which when applied to
  vec1, aligns it with vec2.
  """
  a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
  v = np.cross(a, b)
  c = np.dot(a, b)
  s = np.linalg.norm(v)
  kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
  rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
  return rotation_matrix

#######################################################################
# STEP1: Read Obj file and do convex decompose using VHACD
#######################################################################
path = pathlib.Path(__file__).parent.absolute()
# name_in = os.path.join(path, "../../data/objects/puzzles/alpha-1.0_thinned.obj")
name_in = os.path.join(path, "../../data/objects/puzzles/alpha_shape/alpha-1.0.env.obj")
# name_in = os.path.join(path, "../../data/objects/puzzles/alpha_shape/alpha-1.0.obj")
folder, name_in_file = os.path.split(os.path.realpath(name_in))
filename, file_extension = os.path.splitext(os.path.basename(name_in))

### Create centered + scaled version
mesh = trimesh.load(name_in, skip_materials=True)
mesh.apply_translation(-mesh.centroid)
mesh.apply_scale(scale)
name_scaled = folder + "/" + filename + "-scaled" + file_extension
mesh.export(name_scaled, "obj", write_texture=False, include_texture=False)

### Decompose 

name_out = folder + "/" + filename + "-decomposed" + file_extension
name_log = "log.txt"

p.connect(p.DIRECT)
P = p.vhacd(name_scaled, name_out, name_log, alpha=0.04,resolution=50000 )

#######################################################################
# STEP2: Split using trimesh, export each convex shape to separate obj file
#######################################################################
mesh = trimesh.load(name_out)
ctr = 1
names = []

# mesh.apply_translation(-mesh.centroid)
# mesh.apply_scale(0.1)

scale = False
for m in mesh.split():
  name_m = folder + "/meshes/" + filename + "-" + str(ctr) + file_extension

  if scale:
    S = np.eye(4)
    S[1,1] = 0.3
    S[2,2] = 0.3

    ## translate to centroid
    C = m.centroid

    m.apply_translation(-C)

    X = m.vertices
    X -= np.mean(X, axis = 0)  

    cov = np.cov(X, rowvar = False)
    ev, eig = np.linalg.eig(cov)
    print(ev,eig)

    v = np.array((1,0,0))
    R = rotation_matrix_from_vectors(eig[ev.argmax()], v)
    T = np.eye(4)
    T[:3,:3] = R
    m.apply_transform(T)
    m.apply_transform(S)

    m.apply_transform(np.linalg.inv(T))
    m.apply_translation(+C)

  m.export(name_m, "obj")
  ctr = ctr + 1
  names.append(name_m)

#######################################################################
# STEP3: Add obj files as links into URDF
#######################################################################
name_scaled = folder + "/" + filename + "-scaled.urdf"

# f = open(name_scaled,'w')
Objs2URDF(name_scaled)
# f.write(urdfStr)
# print("Wrote URDF to ", name_scaled)
# f.close()

fname = folder + "/" + filename + ".urdf"
f = open(fname,'w')

urdfStr = Objs2URDF(name_in, names)

f.write(urdfStr)
print("Wrote URDF to ", fname)
f.close()
