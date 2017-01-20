import numpy as np
import os.path
import trimesh

filename = "converted/drc_ladder_mod1.stl"

basename = os.path.basename(filename)
basename = os.path.splitext(basename)

trifilename = 'converted/'+basename[0]+'.tri'

print "INPUT:",filename
print "OUTPUT:",trifilename

mesh = trimesh.load(filename)
print mesh
vertices = mesh.vertices 
tris = mesh.faces

Nvertices = vertices.shape[0]
Ntris = tris.shape[0]

print "Nvertices:",Nvertices
print "vertices:",vertices
print "Ntris:",Ntris
print "tri:",tris

###############################################################################
## IO file to tri/vertices
###############################################################################
f = open(trifilename, "w")

f.write(str(Nvertices)+'\n')

for v in vertices:
  f.write(str(v[0])+' ')
  f.write(str(v[1])+' ')
  f.write(str(v[2])+'\n')

f.write(str(Ntris)+'\n')

for t in tris:
  f.write(str(t[0])+' ')
  f.write(str(t[1])+' ')
  f.write(str(t[2])+'\n')

#vertices = np.zeros((Nvertices,3))
#i=0
#for line in f:
#  num = line.split()
#  l = len(num)
#  if l == 1:
#    Ntris = int(num[0])
#    break
#  else:
#    vertices[i,0]=num[0]
#    vertices[i,1]=num[1]
#    vertices[i,2]=num[2]
#    i+=1
#
#tris = np.zeros((Ntris,3)).astype(int)
#
#i=0
#for line in f:
#  num = line.split()
#  tris[i,0]=num[0]
#  tris[i,1]=num[1]
#  tris[i,2]=num[2]
#  i+=1
#f.close()
#
################################################################################
### tri/vertices to trimesh
################################################################################
#
#
f.close()
