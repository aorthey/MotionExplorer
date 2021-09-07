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

cameraLocation = Vector((+15,-15,0))
# cameraLocation = Vector((+10,+0,+10))
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

def drawScene(filename, scn):
    global cameraLocation
    dirname = os.path.dirname(os.path.realpath(filename))
    basename = os.path.basename(os.path.realpath(filename))
    sys.path.append(dirname)

    ###########################################################################
    ### DELETE ALL OBJECTS
    ###########################################################################
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    bpy.ops.wm.collada_import(filepath=filename)

    # addLightSourceSun(Vector((+15,-15,0)))
    addLightSourceArea(Vector((-5,0,0)))

    addCamera(cameraLocation, cameraFocusPoint)

    ## NOTE: need to rotate camera
    direction = cameraFocusPoint - cameraLocation
    rot_quat = direction.to_track_quat('-Z', 'Y')
    rot_quat = rot_quat.to_matrix().to_4x4()

    cameraLocation = cameraLocation.to_tuple()

    roll = radians(90)  # select your desired amount of rotation
    camera_roll = Matrix.Rotation(roll, 4, 'Z')
    bpy.context.scene.camera.matrix_world = rot_quat @ camera_roll
    bpy.context.scene.camera.location = cameraLocation

    # rot_quat = direction.to_track_quat('-Z', 'Y')

    # quat = rot_quat.to_matrix().to_4x4()
    # rollMatrix = Matrix.Rotation(15, 4, 'Z')
    # bpy.context.scene.camera.matrix_world = quat @ rollMatrix
    
   # rotation_euler = rot_quat.to_euler()
    # bpy.context.scene.camera.rotation_euler[1] = 0
    # bpy.context.scene.camera.rotation_euler[2] = 0


    ### SAVE IMAGE
    bpy.context.scene.render.film_transparent = True
    bpy.context.scene.render.image_settings.color_mode = 'RGBA'
    bpy.context.scene.render.filepath = dirname + '/blender'+basename+'.png'

    # bpy.ops.export_mesh.stl("EXEC_DEFAULT", dirname+'/blender'+basename+'.stl')

    bpy.ops.render.render(write_still = True)

    ff = bpy.context.scene.render.filepath
    os.system("convert -trim %s %s"%(ff,ff))
    os.system("display %s"%(ff))

    print(bpy.context.scene.render.filepath)

if __name__ == "__main__":
  filenames = ["/home/aorthey/git/manipulation-planning/examples/19-multi/z.vid/world0.dae"]
  for filename in filenames:
    drawScene(filename, bpy.context.scene)
