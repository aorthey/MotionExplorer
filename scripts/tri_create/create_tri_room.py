import numpy as np
from shutil import copyfile


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

import os, sys, re
print os.path.dirname(os.path.abspath(__file__))
print os.getcwd()

## radius for articulated chain is 0.1, i.e. the passage with 0.1 should contain
## a zero measure solution which we would not find using rrt. Everything above
## should be solvable, and most of the interesting cases should be in between 0.1
## and 0.3. Above 0.3 the cases are too easy to solve (and trivial
## stratification should dominate)
#for d in np.linspace(0.15,0.35):
f = "../../data/experiments/11D_planar_snake_narrow_passage.xml"
for d in np.arange(0.15,0.45,0.01):
  #CreateTriRoom(d)
  alpha = "{:.2f}".format(d)
  alpha = alpha.replace('.','_')
  fname = f.replace('narrow_passage',alpha)
  copyfile(f, fname)

  with open(fname, "r") as sources:
    lines = sources.readlines()
  with open(fname, "w") as sources:
    for line in lines:
      env = 'narrow_passage_'+alpha+'.tri'
      sources.write(re.sub(r'narrow_passage_0_20.tri', env, line))

  #11D_planar_snake_0_20.xml
