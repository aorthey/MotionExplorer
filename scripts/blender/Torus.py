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

materialRestriction = bpy.data.materials.new(name="PathRestrictionColoring")
# materialMagenta.diffuse_color = (0.505, 0.0, 0.77, 0.5)
materialRestriction.diffuse_color = (0.8, 0.4, 1.0, 1.0)
materialRestriction.metallic = 0.9
materialRestriction.specular_intensity = 0.9

def drawScene(filename):
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
    E = getRoadmapEdges(fname)
    P = getRoadmapStates(pname)
    print("Loaded %d states from file %s." % (X.shape[0],fname))
    print("Loaded %d edges from file %s." % (E.shape[0],fname))
    print("Loaded %d path states from file %s." % (P.shape[0],fname))

    ###########################################################################
    ### INITIAL STATE
    ###########################################################################
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
    # addStateTorus(X[1,0], X[1,1], color=colorGoalState, size=0.4, offset=offsetRoadmap)
    # for x in X[2:]:
    #   addStateTorus(x[0], x[1], color=colorStartState, size=0.2, offset=offsetRoadmap)

    # for e in E:
    #   addEdgeTorus(e, color=colorStartState, width=0.01, offset=offsetRoadmap)

    # addEdgeTorus(P, color=colorStartState, width=0.10,
    #     material=materialGreenLight, offset=offsetRoadmap)

    # addLightSourceSun(Vector((0,0,20)))
    # addCamera(cameraLocation, cameraFocusPoint)

    ### SAVE IMAGE
    renderEngine = bpy.context.scene.render
    renderEngine.film_transparent = True
    # renderEngine.ffmpeg.format = "PNG"
    renderEngine.image_settings.file_format = "PNG"

    renderEngine.image_settings.color_mode = 'RGBA'
    renderEngine.filepath = dirname+"/"+filename+'.png'
    bpy.ops.render.render(write_still = True)

    renderEngine.filepath = dirname+"/"+filename+'.png'

    ff = renderEngine.filepath
    os.system("convert -trim %s %s"%(ff,ff))

    print(renderEngine.filepath)

if __name__ == "__main__":

  filenames = [ "2020ICRA/02D_torus_SPARStwo", 
      "2020ICRA/02D_torus_PRMstar"]
  for filename in filenames:
    drawScene(filename)#, bpy.context.scene, 72, mobiusStripThickness)
