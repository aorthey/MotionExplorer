import numpy as np
from shutil import copyfile
import subprocess
from tri_primitives import MetaMesh
import os, sys, re

Z_height = 0.1
radius = 0.15
mesh = MetaMesh()
mesh.AddCylinder(0,0,0, radius, Z_height)

path = "../../data/terrains/primitives/"
os.chdir(path)
fname = "manipulatable_disk.off"
mesh.write(fname)
# import subprocess
# subprocess.run(["off2tri", fname])


