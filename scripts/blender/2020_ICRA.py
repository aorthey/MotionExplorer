import bpy
import bmesh
import numpy as np
import re
from math import *
from mathutils import *

import sys, os 
dirname = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dirname)

from bpy_utils import *
import bpy_utils
from read_utils import *

### Using blender 2.82a
## How to use:
### ./blender --background --python fiber_bundle_IJRR.py &&
###  convert -trim ~/image.png ~/image.png && display ~/image.png

# cameraLocation = Vector((+3,0,+2.3))
cameraLocation = Vector((+3,-1.5,+2.5))
cameraFocusPoint = Vector((0,0,0))

colorGoalState = (0.9, 0.0, 0.0, 1.0)

#https://docs.blender.org/api/blender_python_api_2_78_release/bpy.types.Material.html
materialGreenLight = bpy.data.materials.new(name="LightGreen")
materialGreenLight.diffuse_color = (1.0, 1.0, 1.0, 1.0)
materialGreenLight.metallic = 0.0
materialGreenLight.specular_intensity = 0.0
materialGreenLight.specular_color = (1.0, 1.0, 1.0)


materialRestriction = bpy.data.materials.new(name="PathRestrictionColoring")
# materialMagenta.diffuse_color = (0.505, 0.0, 0.77, 0.5)
materialRestriction.diffuse_color = (0.8, 0.4, 1.0, 1.0)
materialRestriction.metallic = 0.9
materialRestriction.specular_intensity = 0.9

def drawScene(filename, scn, resolution, thick):
    global cameraLocation
    dirname = os.path.dirname(os.path.realpath(__file__))
    sys.path.append(dirname)

    fname = os.path.abspath(dirname+"/../../data/roadmaps/"+filename+".roadmap")
    pname = os.path.abspath(dirname+"/../../data/paths/"+filename+".path")

    ###########################################################################
    ### DELETE ALL OBJECTS
    ###########################################################################
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    X = getRoadmapStates(fname)
    print("Loaded %d states from file %s." % (X.shape[0],fname))
    E = getRoadmapEdges(fname)
    print("Loaded %d edges from file %s." % (E.shape[0],fname))
    P = getRoadmapStates(pname)
    print("Loaded %d path states from file %s." % (P.shape[0],fname))

    ###########################################################################
    ### RESTRICTION
    ###########################################################################

      # major_radius = 3
      # circleThickness = 0.3
      # resolution = 72
      # annulus_strip_mesh = annulus_mesh(resolution, major_radius -
      #     circleThickness, major_radius + circleThickness)
      # annulus_strip_obj = bpy.data.objects.new("annulus", annulus_strip_mesh)
      # annulus_strip_obj.location = Vector((+3, +3, -6))

      # bpy.context.collection.objects.link(annulus_strip_obj)
      # bpy.context.view_layer.objects.active = annulus_strip_obj
      # mod = annulus_strip_obj.modifiers.new("edge split", 'EDGE_SPLIT')

      # annulus_strip_obj.data.materials.append(materialMagenta)

    ###########################################################################
    ### INITIAL STATE
    ###########################################################################
    if re.search("level0", filename):

      bpy_utils.torusMajorRadius = bpy_utils.torusMajorRadius - bpy_utils.torusMinorRadius

      cameraLocation = Vector((+3,-1.5,+2.5))
      addStateTorus(X[0,0], 0, color=colorStartState, size=0.4)
      addStateTorus(X[1,0], 0, color=colorGoalState, size=0.4)
      for x in X[2:]:
        addStateTorus(x[0], 0, color=colorStartState, size=0.2)

      for e in E:
        addEdgeTorus(e, color=colorStartState, width=0.01)

      addEdgeTorus(P, color=colorStartState, width=0.10,
          material=materialGreenLight)

      radius = torusMajorRadius
      circleThickness = 0.3
      resolution = 72
      annulus_strip_mesh = annulus_mesh(resolution, radius -
          circleThickness, radius + circleThickness)
      annulus_strip_obj = bpy.data.objects.new("annulus", annulus_strip_mesh)
      annulus_strip_obj.location = Vector((+0, +0, -0.1))

      bpy.context.collection.objects.link(annulus_strip_obj)
      bpy.context.view_layer.objects.active = annulus_strip_obj
      mod = annulus_strip_obj.modifiers.new("edge split", 'EDGE_SPLIT')

      annulus_strip_obj.data.materials.append(materialMagenta)
    else:
      offsetRoadmap = 0.0
      if re.search("SMLR", filename):
        offsetRoadmap = 0.1
        fromU = float(X[0,0])
        fromV = 0.0
        toU = float(X[1,0])
        toV = 2*np.pi
        offset = 0.0
        restriction_torus_mesh = torusRestriction_mesh(torusLocation, fromU, toU,
            fromV, toV, offset)
        restriction_torus_obj = bpy.data.objects.new("arrow", restriction_torus_mesh)
        # restriction_torus_obj.location = offsetAnnulusLeft + Vector((0,-0.8,0))
        bpy.context.collection.objects.link(restriction_torus_obj)
        restriction_torus_obj.data.materials.append(materialRestriction)

      addTorus(torusLocation, offset = -0.1)
      addStateTorus(X[0,0], X[0,1], color=colorStartState, size=0.4, offset=offsetRoadmap)
      addStateTorus(X[1,0], X[1,1], color=colorGoalState, size=0.4, offset=offsetRoadmap)
      for x in X[2:]:
        addStateTorus(x[0], x[1], color=colorStartState, size=0.2, offset=offsetRoadmap)

      for e in E:
        addEdgeTorus(e, color=colorStartState, width=0.01, offset=offsetRoadmap)

      addEdgeTorus(P, color=colorStartState, width=0.10,
          material=materialGreenLight, offset=offsetRoadmap)

    addLightSource()
    addCamera(cameraLocation, cameraFocusPoint)

    ### SAVE IMAGE
    bpy.context.scene.render.film_transparent = True
    bpy.context.scene.render.image_settings.color_mode = 'RGBA'
    bpy.context.scene.render.filepath = dirname+"/"+filename+'.png'
    bpy.ops.render.render(write_still = True)

    bpy.context.scene.render.filepath = dirname+"/"+filename+'.png'

    ff = bpy.context.scene.render.filepath
    os.system("convert -trim %s %s"%(ff,ff))

    print(bpy.context.scene.render.filepath)

if __name__ == "__main__":

  filenames = ["2020ICRA/02D_torus_SMLR", 
      "2020ICRA/02D_torus_SPARStwo", 
      "2020ICRA/02D_torus_PRMstar", 
      "2020ICRA/02D_torus_SMLR_level0"]
  for filename in filenames:
    drawScene(filename, bpy.context.scene, 72, mobiusStripThickness)
  # filename = "2020ICRA/02D_torus_SMLR_level0"
  # drawScene(filename, bpy.context.scene, 72, mobiusStripThickness)
