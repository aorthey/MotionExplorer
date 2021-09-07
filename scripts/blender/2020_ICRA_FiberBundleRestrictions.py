import bpy
import bmesh
import numpy as np
import re
from math import *
from mathutils import *
import random 


import sys, os 
dirname = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dirname)

from bpy_utils import *
import bpy_utils
from read_utils import *

### Using blender 2.82a
## How to use:
### ./blender --background --python fiber_bundle_IJRR.py

# cameraLocation = Vector((+3,0,+2.3))
cameraLocation = Vector((+3,-1.5,+0.5))
cameraFocusPoint = Vector((0,0,-1))

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

resolution = 72
offsetRoadmap = 0.1 #min distance of structures to torus

def drawCircle(location, circleRadius, modus, X, E, P):
    oldTorusRadius = bpy_utils.torusMajorRadius
    bpy_utils.torusMajorRadius = circleRadius*(bpy_utils.torusMajorRadius -
        bpy_utils.torusMinorRadius)
    radius = bpy_utils.torusMajorRadius + bpy_utils.torusMinorRadius

    circleThickness = 0.3
    annulus_strip_mesh = annulus_mesh(resolution, radius -
        circleThickness, radius + circleThickness)
    annulus_strip_obj = bpy.data.objects.new("annulus", annulus_strip_mesh)
    annulus_strip_obj.location = location - Vector((0,0,0.1))

    bpy.context.collection.objects.link(annulus_strip_obj)
    bpy.context.view_layer.objects.active = annulus_strip_obj
    mod = annulus_strip_obj.modifiers.new("edge split", 'EDGE_SPLIT')

    annulus_strip_obj.data.materials.append(materialMagenta)

    if modus == 0:
      addStateTorus(X[0,0], 0, location, color=colorStartState, size=0.4)
    elif modus == 1:
      addStateTorus(X[0,0], 0, location, color=colorStartState, size=0.4)
      addStateTorus(X[1,0], 0, location, color=colorGoalState, size=0.4)
      addEdgeTorus(P, location, color=colorStartState, width=0.10,
          material=materialGreenLight)
    elif modus == 2:
      addStateTorus(X[0,0], 0, location, color=colorStartState, size=0.4)
      addStateTorus(X[1,0], 0, location, color=colorGoalState, size=0.4)
      for x in X[2:]:
        addStateTorus(x[0], 0, location, color=colorStartState, size=0.2)
      for e in E:
        addEdgeTorus(e, location, color=colorStartState, width=0.01)

    bpy_utils.torusMajorRadius = oldTorusRadius

def drawRestrictionOnTorus(modus, X, E, P):
    offset = 0.0
    if modus == 0:
      fromU = float(X[0,0] - 0.03)
      fromV = 0.0
      toU = float(X[0,0] + 0.03)
      toV = 2*np.pi
      restriction_torus_mesh = torusRestriction_mesh(torusLocation, fromU, toU,
          fromV, toV, offset)
      restriction_torus_obj = bpy.data.objects.new("fiber", restriction_torus_mesh)
      bpy.context.collection.objects.link(restriction_torus_obj)
      restriction_torus_obj.data.materials.append(materialRestriction)
    elif modus == 1:
      fromU = float(X[0,0])
      fromV = 0.0
      toU = float(X[1,0])
      toV = 2*np.pi
      restriction_torus_mesh = torusRestriction_mesh(torusLocation, fromU, toU,
          fromV, toV, offset)
      restriction_torus_obj = bpy.data.objects.new("pathrestriction", restriction_torus_mesh)
      bpy.context.collection.objects.link(restriction_torus_obj)
      restriction_torus_obj.data.materials.append(materialRestriction)
    elif modus == 2:
      offset = 0.0
      ctr = 0
      ctrMax = len(E)
      for e in E:
        fromU = float(e[0,0])
        toU = float(e[-1,0])

        d = abs(fromU - toU)
        if d > np.pi:
          if fromU < toU:
            fromU = fromU + 2*np.pi
          else:
            toU = toU + 2*np.pi

        if fromU > toU:
          tmp = toU
          toU = fromU
          fromU = tmp

        d = fromU + np.pi/2
        if d < 0:
          d = d + 2*np.pi

        offsetScale = 1.5
        if d < np.pi:
          offsetRoadmap = offsetScale*(d)/(np.pi)
        else:
          offsetRoadmap = offsetScale*((2*np.pi - 0.7 - d))/(np.pi)
        offsetRoadmap = abs(offsetRoadmap)

        ctr = ctr + 1

        print(fromU, toU, offsetRoadmap)
        
        fromV = 0.0
        toV = 2*np.pi

        restriction_torus_mesh = torusRestriction_mesh(torusLocation, fromU, toU,
            fromV, toV, offsetRoadmap)
        restriction_torus_obj = bpy.data.objects.new("restriction" +
            str(abs(fromU)) +
            str(abs(toU)), restriction_torus_mesh)
        bpy.context.collection.objects.link(restriction_torus_obj)
        restriction_torus_obj.data.materials.append(materialRestriction)

    if modus == 2:
        addStateTorus(X[0,0], X[0,1], color=colorStartState, size=0.4, offset=offsetRoadmap)
        addStateTorus(X[1,0], X[1,1], color=colorGoalState, size=0.4, offset=offsetRoadmap)

def drawScene(X2_name, X1_name, modus):
    global cameraLocation
    dirname = os.path.dirname(os.path.realpath(__file__))
    sys.path.append(dirname)

    X1_fname = os.path.abspath(dirname+"/../../data/roadmaps/2020ICRA/"+X1_name+".roadmap")
    X2_fname = os.path.abspath(dirname+"/../../data/roadmaps/2020ICRA/"+X2_name+".roadmap")
    X1_pname = os.path.abspath(dirname+"/../../data/paths/2020ICRA/"+X1_name+".path")
    X2_pname = os.path.abspath(dirname+"/../../data/paths/2020ICRA/"+X2_name+".path")

    filename = "bundle_" + X2_name

    if modus == 0:
      filename += "_fiber"
    elif modus == 1:
      filename += "_pathrestriction"
    elif modus == 2:
      filename += "_graphrestriction"
    elif modus == 3:
      filename = X2_name
    else:
      print("Unknown modus")
      sys.exit(0)

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    X2 = getRoadmapStates(X2_fname)
    X1 = getRoadmapStates(X1_fname)

    E2 = getRoadmapEdges(X2_fname)
    E1 = getRoadmapEdges(X1_fname)

    P2 = getRoadmapStates(X2_pname)
    P1 = getRoadmapStates(X1_pname)

    ###########################################################################
    ### DELETE ALL OBJECTS
    ###########################################################################
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    ###########################################################################
    ### INITIAL STATE
    ###########################################################################
    if modus == 2:
      location = Vector((+9,-4,-1))
    else:
      location = Vector((+9,-4.5,-0.5))

    if modus == 3:
      drawCircle(location, 0.3, 1, X1, E1, P1)
      drawRestrictionOnTorus(1, X1, E1, P1)
    else:
      drawCircle(location, 0.3, modus, X1, E1, P1)
      drawRestrictionOnTorus(modus, X1, E1, P1)

    addTorus(torusLocation, offset = -0.1)

    addStateTorus(X2[0,0], X2[0,1], color=colorStartState, size=0.4, offset=offsetRoadmap)
    addStateTorus(X2[1,0], X2[1,1], color=colorGoalState, size=0.4, offset=offsetRoadmap)

    if modus == 3:
      for x in X2[2:]:
        addStateTorus(x[0], x[1], color=colorStartState, size=0.2, offset=offsetRoadmap)

      for e in E2:
        addEdgeTorus(e, color=colorStartState, width=0.01, offset=offsetRoadmap)

      addEdgeTorus(P2, color=colorStartState, width=0.10,
          material=materialGreenLight, offset=offsetRoadmap)


    addLightSourceSun(Vector((0,0,20)))
    addLightSourceSun(Vector((0,0,0)))
    addLightSourceArea(Vector((0,0,-1)))

    addCamera(cameraLocation, cameraFocusPoint)

    ### SAVE IMAGE
    bpy.context.scene.render.film_transparent = True
    bpy.context.scene.render.image_settings.color_mode = 'RGBA'
    bpy.context.scene.render.filepath = dirname+"/2020ICRA/"+filename+'.png'
    bpy.ops.render.render(write_still = True)


    ff = bpy.context.scene.render.filepath
    os.system("convert -trim %s %s"%(ff,ff))

    print(bpy.context.scene.render.filepath)

if __name__ == "__main__":

  filenames = [
      "02D_torus_SMLR_level0",
      "02D_torus_SMLR"]

  for modus in range(0,4):
    drawScene(filenames[1], filenames[0], modus)
