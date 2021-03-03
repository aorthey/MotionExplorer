import numpy as np
from shutil import copyfile
import subprocess
from tri_primitives import MetaMesh
import os, sys, re

height = 0.05
radius = 0.3

mesh = MetaMesh()
mesh.AddCylinder(0,0,0, radius, height)

path = "../../data/terrains/primitives/"
os.chdir(path)

fname = "cylinder_height_%.2f_radius_%.2f"%(height, radius)
fname = fname.replace(".", "_")
fname = fname + ".stl"

mesh.write(fname)
# import subprocess
# subprocess.run(["off2tri", fname])


