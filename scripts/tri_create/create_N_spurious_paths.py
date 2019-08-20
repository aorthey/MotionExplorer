import numpy as np
from shutil import copyfile
import subprocess
from tri_primitives import MetaMesh
import os, sys, re

def CreateRoom(N, alpha):
  mesh = MetaMesh()

  height = 0.1
  length = 8
  length_inner_box = 0.3*length
  thickness = 0.5
  passage_length = 1.3
  top_length = 4*passage_length
  xright = 0.5*length+thickness
  xleft = -xright

  #(1) bottom element
  mesh.AddBox(0, 0, 0, 0.5*length+2*thickness, thickness, height)
  y = thickness+0.5*passage_length
  mesh.AddBox(xleft, y, 0, thickness, 0.5*passage_length, height)
  mesh.AddBox(xright, y, 0, thickness, 0.5*passage_length, height)
  #(3) N passage elements
  for k in range(1,N):
    yoffset = thickness + passage_length + 2*(k-1)*passage_length
    yk = yoffset + 0.5*passage_length
    mesh.AddBox(0, yk, 0, 0.5*length_inner_box, 0.5*passage_length, height)
    yk = yoffset + passage_length
    mesh.AddBox(xleft, yk, 0, thickness, passage_length, height)
    mesh.AddBox(xright, yk, 0, thickness, passage_length, height)

  #(N+1) top element
  yoffset = thickness + passage_length + 2*(N-1)*passage_length
  ytop = yoffset + top_length + thickness
  mesh.AddBox(0, ytop, 0, 0.5*length+2*thickness, thickness, height)
  y = yoffset + 0.5*top_length
  mesh.AddBox(xleft, y, 0, thickness, 0.5*top_length, height)
  mesh.AddBox(xright, y, 0, thickness, 0.5*top_length, height)
  y = yoffset + 0.25*top_length
  mesh.AddBox(0, y, 0, 0.5*length_inner_box, 0.25*top_length, height)

  path = "../../data/terrains/"
  os.chdir(path)
  fname = "spurious_path_N.off"
  mesh.write(fname)

path = os.getcwd()

d = 0.1
N = 8
CreateRoom(N, d)
os.chdir(path)
