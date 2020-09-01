import numpy as np
import os.path
import trimesh


class ConverterTriToStl():
  def __init__(self, input_file_name, output_file_name):
    ###############################################################################
    ## IO file to tri/vertices
    ###############################################################################
    f = open(input_file_name, "r")
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
    #print "vertices:",vertices
    print "Ntris:",Ntris
    #print "tri:",tris

    mesh = trimesh.Trimesh(vertices, tris)
    mesh.export(output_file_name)

if __name__ == '__main__':
  input_file_name = "converted/drc_ladder_mod1.tri"
  output_file_name = "converted/drc_ladder_mod1.stl"
  ConverterTriToStl(input_file_name, output_file_name)
  pass
