import numpy as np
from tri_primitives import MetaMesh

def CreateTriRoom(alpha):
  mesh = MetaMesh()

  Z_height = 0.1
  width_wall = 0.25
  length_wall = 2

  length_bttm = 2
  max_x = 5
  length_passage = 1.5
  width_mid = 0.5*(max_x - length_passage - width_wall)
  x_mid = length_passage + width_mid

  mesh.AddBox(-max_x,0,0,width_wall,length_wall,Z_height)
  mesh.AddBox(-x_mid,-length_wall+width_wall,0,width_mid,width_wall,Z_height)
  mesh.AddBox(-x_mid,+length_wall-width_wall,0,width_mid,width_wall,Z_height)

  mesh.AddBox(+max_x,0,0,width_wall,2,Z_height)
  mesh.AddBox(+x_mid,-length_wall+width_wall,0,width_mid,width_wall,Z_height)
  mesh.AddBox(+x_mid,+length_wall-width_wall,0,width_mid,width_wall,Z_height)

  ##narrow passage
  width_passage_box = 0.5*(length_wall - alpha)
  y = alpha + width_passage_box
  mesh.AddBox(0,+y,0,length_passage,width_passage_box,Z_height)
  mesh.AddBox(0,-y,0,length_passage,width_passage_box,Z_height)

  fname = "%.2f"%alpha
  fname = fname.replace(".", "_")

  fname = "../../data/terrains/narrow_passage/narrow_passage_"+fname+".off"

  mesh.write(fname)

import os
print os.path.dirname(os.path.abspath(__file__))
print os.getcwd()

for d in np.linspace(0.1,1,10):
  CreateTriRoom(d)
