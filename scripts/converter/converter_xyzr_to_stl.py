import numpy as np
import os.path
import trimesh
import sys


class ConverterXYZRToStl():
  def __init__(self, input_file_name, output_file_name):
    ###############################################################################
    ## IO file to tri/vertices
    ###############################################################################
    f = open(input_file_name, "r")
    # file with sphere specifications
    # each row: x y z r

    Nspheres = int(f.readline())
    spheres = np.zeros((Nspheres,4))
    i=0
    mesh = None
    for line in f:
      l = line.split()
      N = len(l)
      if N != 4:
        print("Error at line: %s", line)
        break
      else:
        pos = (float(l[0]),float(l[1]),float(l[2]))
        radius = float(l[3])
        s = trimesh.creation.icosphere(2, radius)
        T = trimesh.transformations.translation_matrix(pos)
        s.apply_transform(T)
        print("Processing sphere %d/%d" %(i,Nspheres))
        if mesh is None:
          mesh = s
        else:
          mesh = trimesh.Trimesh.union(mesh, s)
        i+=1


    mesh.export(output_file_name)

    print("Nspheres: %d" % Nspheres)


if __name__ == '__main__':
  input_file_name = "tests/907758.xyzr"
  output_file_name = "tests/907758.stl"
  ConverterXYZRToStl(input_file_name, output_file_name)
  pass
