import numpy as np
from shutil import copyfile
import subprocess
from tri_primitives import MetaMesh
import os, sys, re

def CreateTriRoomDoublePassage(alpha):
  mesh = MetaMesh()

  Z_height = 0.1
  width_wall = 0.2
  length_wall = 1.5

  length_bttm = 1
  max_x = 3
  length_passage = 0.5
  width_mid = 0.5*(max_x - length_passage - width_wall)
  x_mid = length_passage + width_mid

  #left wall
  mesh.AddBox(-max_x,0,0,width_wall,length_wall,Z_height)
  mesh.AddBox(-x_mid,-length_wall+width_wall,0,width_mid,width_wall,Z_height)
  mesh.AddBox(-x_mid,+length_wall-width_wall,0,width_mid,width_wall,Z_height)

  #right wall
  mesh.AddBox(+max_x,0,0,width_wall,length_wall,Z_height)
  mesh.AddBox(+x_mid,-length_wall+width_wall,0,width_mid,width_wall,Z_height)
  mesh.AddBox(+x_mid,+length_wall-width_wall,0,width_mid,width_wall,Z_height)

  ##narrow passage
  width_passage_box = 0.25*(length_wall - alpha)
  y = length_wall - width_passage_box
  mesh.AddBox(0,+y,0,length_passage,width_passage_box,Z_height)
  mesh.AddBox(0,-y,0,length_passage,width_passage_box,Z_height)
  mesh.AddBox(0,0,0,length_passage,width_passage_box,Z_height)

  fname = "%.3f"%alpha
  fname = fname.replace(".", "_")

  path = "../../data/terrains/narrow_passage/"
  os.chdir(path)
  fname = "narrow_passage_double_passage.off"
  mesh.write(fname)

CreateTriRoomDoublePassage(0.35)
