import openmesh
import numpy as np

class MetaMesh:
  def __init__(self):
    self.mesh = openmesh.TriMesh()

  def write(self, fname):
    openmesh.write_mesh(fname, self.mesh)
    print("Wrote Mesh to",fname)

  def AddCylinder(self, x, y, z, radius, height):
    vhandle = []

    N=16
    zLow = z - 0.5*height
    zHigh = z + 0.5*height
    vcenter_bottom = self.mesh.add_vertex([x, y, zLow])
    vcenter_top = self.mesh.add_vertex([x, y, zHigh])
    for i in range(0,N):
      T = 2*np.pi/N
      x1 = radius*np.cos(i*T) + x
      y1 = radius*np.sin(i*T) + x
      x2 = radius*np.cos((i+1)*T) + y
      y2 = radius*np.sin((i+1)*T) + y
      vl0 = self.mesh.add_vertex([x1,y1,zLow])
      vl1 = self.mesh.add_vertex([x1,y1,zHigh])
      vl2 = self.mesh.add_vertex([x2,y2,zHigh])
      vl3 = self.mesh.add_vertex([x2,y2,zLow])
      vhandle.append(vl0)
      vhandle.append(vl1)
      vhandle.append(vl2)
      vhandle.append(vl3)

      face_vhandles = []
      face_vhandles.append(vl0)
      face_vhandles.append(vl1)
      face_vhandles.append(vl2)
      self.mesh.add_face(face_vhandles)

      face_vhandles = []
      face_vhandles.append(vl2)
      face_vhandles.append(vl3)
      face_vhandles.append(vl0)
      self.mesh.add_face(face_vhandles)

      face_vhandles = []
      face_vhandles.append(vcenter_bottom)
      face_vhandles.append(vl0)
      face_vhandles.append(vl3)
      self.mesh.add_face(face_vhandles)

      face_vhandles = []
      face_vhandles.append(vcenter_top)
      face_vhandles.append(vl2)
      face_vhandles.append(vl1)
      self.mesh.add_face(face_vhandles)

  def AddCylinderWithCut(self, x, y, z, radius, height, cutFrom=np.pi/2-np.pi/24,
      cutTo=np.pi/2):
    vhandle = []

    N=24
    zLow = z - 0.5*height
    zHigh = z + 0.5*height
    vcenter_bottom = self.mesh.add_vertex([x, y, zLow])
    vcenter_top = self.mesh.add_vertex([x, y, zHigh])
    for i in range(0,N):
      T = 2*np.pi/N

      if i*T >= cutFrom and i*T <= cutTo: 
        x1 = radius*np.cos(i*T) + x
        y1 = radius*np.sin(i*T) + y
        x2 = radius*np.cos((i+1)*T) + x
        y2 = radius*np.sin((i+1)*T) + y
        vl0 = self.mesh.add_vertex([x1,y1,zLow])
        vl1 = self.mesh.add_vertex([x1,y1,zHigh])
        vl2 = self.mesh.add_vertex([x2,y2,zLow])
        vl3 = self.mesh.add_vertex([x2,y2,zHigh])

        face_vhandles = []
        face_vhandles.append(vcenter_top)
        face_vhandles.append(vl1)
        face_vhandles.append(vl0)
        self.mesh.add_face(face_vhandles)

        face_vhandles = []
        face_vhandles.append(vcenter_bottom)
        face_vhandles.append(vcenter_top)
        face_vhandles.append(vl0)
        self.mesh.add_face(face_vhandles)

        face_vhandles = []
        face_vhandles.append(vcenter_top)
        face_vhandles.append(vl2)
        face_vhandles.append(vl3)
        self.mesh.add_face(face_vhandles)

        face_vhandles = []
        face_vhandles.append(vcenter_top)
        face_vhandles.append(vcenter_bottom)
        face_vhandles.append(vl2)
        self.mesh.add_face(face_vhandles)
        continue

      x1 = radius*np.cos(i*T) + x
      y1 = radius*np.sin(i*T) + x
      x2 = radius*np.cos((i+1)*T) + y
      y2 = radius*np.sin((i+1)*T) + y
      vl0 = self.mesh.add_vertex([x1,y1,zLow])
      vl1 = self.mesh.add_vertex([x1,y1,zHigh])
      vl2 = self.mesh.add_vertex([x2,y2,zHigh])
      vl3 = self.mesh.add_vertex([x2,y2,zLow])
      vhandle.append(vl0)
      vhandle.append(vl1)
      vhandle.append(vl2)
      vhandle.append(vl3)

      face_vhandles = []
      face_vhandles.append(vl0)
      face_vhandles.append(vl1)
      face_vhandles.append(vl2)
      self.mesh.add_face(face_vhandles)

      face_vhandles = []
      face_vhandles.append(vl2)
      face_vhandles.append(vl3)
      face_vhandles.append(vl0)
      self.mesh.add_face(face_vhandles)

      face_vhandles = []
      face_vhandles.append(vcenter_bottom)
      face_vhandles.append(vl0)
      face_vhandles.append(vl3)
      self.mesh.add_face(face_vhandles)

      face_vhandles = []
      face_vhandles.append(vcenter_top)
      face_vhandles.append(vl2)
      face_vhandles.append(vl1)
      self.mesh.add_face(face_vhandles)

  def AddBox(self, x,y,z,width,length,height):
    vhandle = []
    vl0 = self.mesh.add_vertex([x-width, y-length, z-height])
    vl1 = self.mesh.add_vertex([x+width, y-length, z-height])
    vl2 = self.mesh.add_vertex([x+width, y+length, z-height])
    vl3 = self.mesh.add_vertex([x-width, y+length, z-height])
    vhandle.append(vl0)
    vhandle.append(vl1)
    vhandle.append(vl2)
    vhandle.append(vl3)

    vh0 = self.mesh.add_vertex([x-width, y-length, z+height])
    vh1 = self.mesh.add_vertex([x+width, y-length, z+height])
    vh2 = self.mesh.add_vertex([x+width, y+length, z+height])
    vh3 = self.mesh.add_vertex([x-width, y+length, z+height])
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

