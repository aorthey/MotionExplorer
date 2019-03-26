import openmesh

class MetaMesh:
  def __init__(self):
    self.mesh = openmesh.TriMesh()

  def write(self, fname):
    openmesh.write_mesh(fname, self.mesh)
    print "Wrote Mesh to",fname

  def AddBox(self, x,y,z,width,length,height):
    vhandle = []
    vl0 = self.mesh.add_vertex([x-width, y-length, -height])
    vl1 = self.mesh.add_vertex([x+width, y-length, -height])
    vl2 = self.mesh.add_vertex([x+width, y+length, -height])
    vl3 = self.mesh.add_vertex([x-width, y+length, -height])
    vhandle.append(vl0)
    vhandle.append(vl1)
    vhandle.append(vl2)
    vhandle.append(vl3)

    vh0 = self.mesh.add_vertex([x-width, y-length, +height])
    vh1 = self.mesh.add_vertex([x+width, y-length, +height])
    vh2 = self.mesh.add_vertex([x+width, y+length, +height])
    vh3 = self.mesh.add_vertex([x-width, y+length, +height])
    vhandle.append(vh0)
    vhandle.append(vh1)
    vhandle.append(vh2)
    vhandle.append(vh3)

    face_vhandles = []
    face_vhandles.append(vhandle[0])
    face_vhandles.append(vhandle[1])
    face_vhandles.append(vhandle[3])
    self.mesh.add_face(face_vhandles)

    face_vhandles = []
    face_vhandles.append(vhandle[1])
    face_vhandles.append(vhandle[2])
    face_vhandles.append(vhandle[3])
    self.mesh.add_face(face_vhandles)

    #=======================

    face_vhandles = []
    face_vhandles.append(vhandle[7])
    face_vhandles.append(vhandle[6])
    face_vhandles.append(vhandle[5])
    self.mesh.add_face(face_vhandles)

    face_vhandles = []
    face_vhandles.append(vhandle[7])
    face_vhandles.append(vhandle[5])
    face_vhandles.append(vhandle[4])
    self.mesh.add_face(face_vhandles)

    #=======================

    face_vhandles = []
    face_vhandles.append(vhandle[1])
    face_vhandles.append(vhandle[0])
    face_vhandles.append(vhandle[4])
    self.mesh.add_face(face_vhandles)

    face_vhandles = []
    face_vhandles.append(vhandle[1])
    face_vhandles.append(vhandle[4])
    face_vhandles.append(vhandle[5])
    self.mesh.add_face(face_vhandles)

    #=======================

    face_vhandles = []
    face_vhandles.append(vhandle[2])
    face_vhandles.append(vhandle[1])
    face_vhandles.append(vhandle[5])
    self.mesh.add_face(face_vhandles)

    face_vhandles = []
    face_vhandles.append(vhandle[2])
    face_vhandles.append(vhandle[5])
    face_vhandles.append(vhandle[6])
    self.mesh.add_face(face_vhandles)

    #=======================

    face_vhandles = []
    face_vhandles.append(vhandle[3])
    face_vhandles.append(vhandle[2])
    face_vhandles.append(vhandle[6])
    self.mesh.add_face(face_vhandles)

    face_vhandles = []
    face_vhandles.append(vhandle[3])
    face_vhandles.append(vhandle[6])
    face_vhandles.append(vhandle[7])
    self.mesh.add_face(face_vhandles)

    #=======================

    face_vhandles = []
    face_vhandles.append(vhandle[0])
    face_vhandles.append(vhandle[3])
    face_vhandles.append(vhandle[7])
    self.mesh.add_face(face_vhandles)

    face_vhandles = []
    face_vhandles.append(vhandle[0])
    face_vhandles.append(vhandle[7])
    face_vhandles.append(vhandle[4])
    self.mesh.add_face(face_vhandles)

