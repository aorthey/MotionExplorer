import numpy as np
import os.path
import trimesh

class ConverterStlToTri():
  def __init__(self, input_file_name, output_file_name):

    mesh = trimesh.load(input_file_name)
    vertices = mesh.vertices 
    tris = mesh.faces

    Nvertices = vertices.shape[0]
    Ntris = tris.shape[0]

    print "Nvertices:",Nvertices
    #print "vertices:",vertices
    print "Ntris:",Ntris
    #print "tri:",tris

    ###############################################################################
    ## IO file to tri/vertices
    ###############################################################################
    f = open(output_file_name, "w")

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

    f.close()

if __name__ == '__main__':
  input_file_name = "converted/drc_ladder_mod1.stl"
  output_file_name = "converted/drc_ladder_mod1.tri"
  ConverterStlToTri(input_file_name, output_file_name)
