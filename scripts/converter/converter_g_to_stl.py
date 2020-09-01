import numpy as np
import os.path
import trimesh


class ConverterGToStl():
  def __init__(self, input_file_name, output_file_name):
    ###############################################################################
    ## IO file to tri/vertices
    ###############################################################################
    f = open(input_file_name, "r")

    headLine = f.readline()
    footLine = f.readline()

    H = headLine.split()
    Nvertices = int(H[1])
    Ntris = int(H[2])

    vertices = np.zeros((Nvertices,3))
    tris = np.zeros((Ntris,3)).astype(int)
    i=0
    for line in f:
      num = line.split()
      l = len(num)
      if l == 1:
        Ntris = int(num[0])
        break
      else:
        if i < Nvertices:
          vertices[i,0]=float(num[0])
          vertices[i,1]=float(num[1])
          vertices[i,2]=float(num[2])
        else:
          if i-Nvertices == 0:
            print(num)
          tris[i-Nvertices,0]=int(num[0])-1
          tris[i-Nvertices,1]=int(num[1])-1
          tris[i-Nvertices,2]=-int(num[2])-1

        i+=1

    vmean = np.mean(vertices, axis=0)
    vertices = vertices-vmean

    ################################################################################
    ### tri/vertices to trimesh
    ################################################################################
    print("%d vertices and %d triangles" % (Nvertices, Ntris))

    mesh = trimesh.Trimesh(vertices, tris)
    mesh.export(output_file_name)

if __name__ == '__main__':
  input_file_name = "tests/trap.g"
  output_file_name = "tests/trap.stl"
  ConverterGToStl(input_file_name, output_file_name)
  pass
