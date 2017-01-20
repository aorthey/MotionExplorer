import numpy as np
import os.path
import trimesh

#filename = "../../Klampt/data/terrains/drc_ladder.tri"
filename = "converted/drc_ladder_mod1.tri"
###############################################################################
## IO file to tri/vertices
###############################################################################
f = open(filename, "r")
# #vertices, then vertices, #triangles, then triangles

Nvertices = int(f.readline())
vertices = np.zeros((Nvertices,3))
i=0
for line in f:
  num = line.split()
  l = len(num)
  if l == 1:
    Ntris = int(num[0])
    break
  else:
    vertices[i,0]=num[0]
    vertices[i,1]=num[1]
    vertices[i,2]=num[2]
    i+=1

tris = np.zeros((Ntris,3)).astype(int)

i=0
for line in f:
  num = line.split()
  tris[i,0]=num[0]
  tris[i,1]=num[1]
  tris[i,2]=num[2]
  i+=1
f.close()

###############################################################################
## tri/vertices to trimesh
###############################################################################
print "Nvertices:",Nvertices
print "vertices:",vertices
print "Ntris:",Ntris
print "tri:",tris


basename = os.path.basename(filename)
basename = os.path.splitext(basename)

meshfilename = 'converted/'+basename[0]+'.stl'

mesh = trimesh.Trimesh(vertices, tris)
mesh.export(meshfilename)

print "INPUT:",filename
print "OUTPUT:",meshfilename
