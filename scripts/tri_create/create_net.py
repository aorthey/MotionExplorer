import numpy as np
from shutil import copyfile
import subprocess
from tri_primitives import MetaMesh
import os
import sys, re

# centered at (0,0,0) going from (-w,0,0) to (+w,0,0) 
# and from (0,0,0) to (0,0,h)
def CreateNet(w, h, alpha):
  mesh = MetaMesh()

  thickness = 0.015

  N_w = int(2*w/alpha)
  N_h = int(h/alpha)
  
  dh = 0

  print(N_h)

  for k in range(0, N_h):
    dh = float(k*alpha)
    mesh.AddBox(0, 0, dh, w, thickness, thickness)

  for k in range(0, N_w):
    dw = float(k*alpha) - w
    mesh.AddBox(dw, 0, h/2, thickness, thickness, h/2)

  fname = "%.3f"%alpha
  fname = fname.replace(".", "_")

  path = "../../data/terrains/"
  os.chdir(path)
  fname = "net_"+fname+".off"
  mesh.write(fname)

  #output, error = process.communicate()

print(os.path.dirname(os.path.abspath(__file__)))
path = os.getcwd()

CreateNet(5, 5, 0.5)
