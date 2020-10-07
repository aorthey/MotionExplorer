import bpy
import bmesh
import numpy as np
from math import *
from mathutils import *
import mathutils as M, random as R, bpy_extras
# from export_svg_280 import ExporterToSVG

### Using blender 2.82a
## How to use:
### ./blender --background --python fiber_bundle_IJRR.py &&
###  convert -trim ~/image.png ~/image.png && display ~/image.png

diameterState = 0.45
mobiusStripThickness = 0.1
pathThickness = 0.3
circleThickness = 0.3

cameraFocusPoint = Vector((0,-8,-7))
cameraLocation = Vector((-15,-8,+10))
distanceCamera = 35
offsetAnnulusLeft = Vector((-3, +1, -15))
offsetAnnulusRight = Vector((-3,-15, -15))

torusMinorRadius = 2.3
torusMajorRadius = 4.1
torusLocation = (0,-14,0)

colorStartState = (0.0, 1.0, 0.0, 1.0)
colorGoalState = (0.7, 0.0, 0.0, 1.0)

minor_radius = 2
major_radius = 5

materialGreen = bpy.data.materials.new(name="Green")
materialGreen.diffuse_color = (0.0, 1.0, 0.0, 1.0)
materialGreen.metallic = 0.0
materialGreen.specular_intensity = 0.0

materialMagenta = bpy.data.materials.new(name="Magenta")
materialMagenta.diffuse_color = (0.45, 0.05, 0.72, 1.0)
materialMagenta.diffuse_color = (0.575, 0.12, 1.0, 1.0)
materialMagenta.diffuse_color = (0.205, 0.0, 0.37, 1.0)
materialMagenta.metallic = 0.9
materialMagenta.specular_intensity = 0.9

def arrow_mesh(position, length, width=-1, headlength=-1, headwidth = -1):
    verts = []
    faces = []

    if width < 0:
      width = 0.15*length
    if headlength < 0:
      headlength = 0.3*length
    if headwidth < 0:
      headwidth = 2*width

    v1 = position
    v2 = Vector((position[0], position[1] - 0.5*headwidth, position[2] + headlength))
    v3 = Vector((position[0], position[1] - 0.5*width, position[2] + headlength))
    v4 = Vector((position[0], position[1] - 0.5*width, position[2] + length))

    v5 = Vector((position[0], position[1] + 0.5*headwidth, position[2] + headlength))
    v6 = Vector((position[0], position[1] + 0.5*width, position[2] + headlength))
    v7 = Vector((position[0], position[1] + 0.5*width, position[2] + length))


    #   3---6
    #   |   |
    #   |   |
    #1--2---5--4
    # \       /
    #  \     /
    #   \   /
    #    \ /
    #     0

    # verts.extend([v1, v2])
    verts.extend([v1, v2, v3, v4, v5, v6, v7])
    faces.append([0, 1, 2])
    faces.append([0, 2, 5])
    faces.append([0, 5, 4])
    faces.append([2, 3, 6])
    faces.append([2, 6, 5])

    mesh = bpy.data.meshes.new("arrow")
    mesh.from_pydata(verts, [], faces)

    for p in mesh.polygons:
        p.use_smooth=True

    return mesh


def annulus_mesh(resolution, inner_radius, outer_radius):
    verts = []
    faces = []

    for i in range(resolution):
        theta = 2*pi * i/resolution
        rot = Matrix.Rotation(theta, 3, [0,0,1])
        v1 = rot @ Vector([inner_radius, 0, 0])
        v2 = rot @ Vector([outer_radius, 0, 0])

        i1 = len(verts)
        verts.extend([v1,v2])

        if i+1<resolution:
            ia = i1+2
            ib = i1+3
        else:
            ia = 0
            ib = 1

        faces.append( [i1,i1+1,ib,ia])

    mesh = bpy.data.meshes.new("annulus")
    mesh.from_pydata(verts, [], faces)

    for p in mesh.polygons:
        p.use_smooth=True

    return mesh

def mobius_mesh(resolution, thick):

    verts = []
    faces = []

    for i in range(resolution):

        theta = 2*pi * i/resolution
        phi = pi * i/resolution

        rot1 = Matrix.Rotation(phi, 3, [0,1,0])
        rot2 = Matrix.Rotation(theta, 3, [0,0,1])
        c1 = Vector([major_radius, 0, 0])
        v1 = rot2 @ (c1 + rot1 @ Vector([-thick / 2, 0, minor_radius]) )
        v2 = rot2 @ (c1 + rot1 @ Vector([thick / 2, 0, minor_radius]) )
        v3 = rot2 @ (c1 + rot1 @ Vector([thick / 2, 0, -minor_radius]) )
        v4 = rot2 @ (c1 + rot1 @ Vector([-thick / 2, 0, -minor_radius]) )

        i1 = len(verts)
        verts.extend([v1,v2,v3,v4])

        if i+1<resolution:
            ia = i1+4
            ib = i1+5
            ic = i1+6
            id = i1+7
        else:
            ia = 2
            ib = 3
            ic = 0
            id = 1

        # faces.append( [i1+j for j in range(4) ])
        faces.append( [i1,i1+1,ib,ia])
        faces.append( [i1+1,i1+2,ic,ib])
        faces.append( [i1+2,i1+3,id,ic])
        faces.append( [i1+3,i1,ia,id])

    mesh = bpy.data.meshes.new("mobius")
    mesh.from_pydata(verts, [], faces)

    for p in mesh.polygons:
        p.use_smooth=True

    return mesh

def addPath():
    ## create curve OBJ
    curve = bpy.data.curves.new(name="L1FF", type='CURVE')
    curve.dimensions = '3D'
    path_obj = bpy.data.objects.new("path", curve)
    path_obj.location = (0,0,0)

    ### create milestones
    poly = curve.splines.new('POLY')
    X = np.arange(0.45,-0.01,-0.01)
    poly.points.add(2 + len(X) - 1)
    poly.points[0].co = getVectorOnStrip4D(0.45, 0.1)
    poly.points[1].co = getVectorOnStrip4D(0.45, 0.9)
    ctr = 2
    for x in X:
      poly.points[ctr].co = getVectorOnStrip4D(x, 0.9)
      ctr = ctr+1

    bpy.context.collection.objects.link(path_obj)

    curve.fill_mode = 'FULL'
    curve.bevel_depth = pathThickness

    path_obj.data.materials.append(materialGreen)


    # mesh = bpy.data.meshes.new("path_mesh")
    # mesh.from_pydata(verts, edges, [])

def addPathAnnulus():
    ## create curve OBJ
    curve = bpy.data.curves.new(name="PATH_ANNULUS", type='CURVE')
    curve.dimensions = '3D'
    path_obj = bpy.data.objects.new("path", curve)
    path_obj.location = (0,0,0)

    ### create milestones
    poly = curve.splines.new('POLY')
    X = np.arange(0.45,-0.01,-0.01)
    poly.points.add(len(X) - 1)
    ctr = 0
    for x in X:
      theta = x*(2*pi)
      rot = Matrix.Rotation(theta, 3, [0,0,1])
      v = (rot @ Vector([major_radius, 0, 0])) + offsetAnnulusLeft
      poly.points[ctr].co = (v[0],v[1],v[2],1)
      ctr = ctr+1

    bpy.context.collection.objects.link(path_obj)

    curve.fill_mode = 'FULL'
    curve.bevel_depth = pathThickness

    path_obj.data.materials.append(materialGreen)

def addPathTorusAnnulus():
    curve = bpy.data.curves.new(name="PATH_TORUS_ANNULUS", type='CURVE')
    curve.dimensions = '3D'
    path_obj = bpy.data.objects.new("path_torus_annulus", curve)
    path_obj.location = (0,0,0)

    ### create milestones
    poly = curve.splines.new('POLY')
    X = np.arange(0.55,0.85,+0.01)
    poly.points.add(len(X) - 1)
    ctr = 0
    for x in X:
      theta = x*(2*pi)
      rot = Matrix.Rotation(theta, 3, [0,0,1])
      v = (rot @ Vector([major_radius, 0, 0])) + offsetAnnulusRight
      poly.points[ctr].co = (v[0],v[1],v[2],1)
      ctr = ctr+1

    bpy.context.collection.objects.link(path_obj)

    curve.fill_mode = 'FULL'
    curve.bevel_depth = pathThickness

    path_obj.data.materials.append(materialGreen)

def update_camera(camera, focus_point=Vector((0.0, 0.0, 0.0)), distance=10.0):
    """
    Focus the camera to a focus point and place the camera at a specific distance from that
    focus point. The camera stays in a direct line with the focus point.

    :param camera: the camera object
    :type camera: bpy.types.object
    :param focus_point: the point to focus on (default=``Vector((0.0, 0.0, 0.0))``)
    :type focus_point: Vector
    :param distance: the distance to keep to the focus point (default=``10.0``)
    :type distance: float
    """
    looking_direction = camera.location - focus_point
    rot_quat = looking_direction.to_track_quat('Z', 'Y')

    camera.rotation_mode = 'XYZ'
    camera.rotation_euler = rot_quat.to_euler()
    camera.location = camera.location + rot_quat @ Vector((0.0, 0.0, distance))



def getVectorOnStrip4D(x_1, x_2):
    v = getVectorOnStrip(x_1, x_2)
    return (v[0],v[1],v[2],1)

def getVectorOnStrip(x_1, x_2):
    theta = x_1*(2*pi)
    phi = x_1*pi
    dx = -minor_radius+x_2*(2*minor_radius)

    rot1 = Matrix.Rotation(phi, 3, [0,1,0])
    rot2 = Matrix.Rotation(theta, 3, [0,0,1])
    c1 = Vector([major_radius, 0, 0])
    return rot2 @ (c1 + rot1 @ Vector([0, 0, dx]) )


def addSphere(pos, name, color=(1.0, 1.0, 1.0, 1.0)):
    xi_mesh = bpy.data.meshes.new("init_state_mesh_"+name)
    xi_obj = bpy.data.objects.new("init_state_"+name, xi_mesh)
    xi_obj.location = xi_obj.location + pos

    bpy.context.collection.objects.link(xi_obj)

    ## select active object
    bpy.context.view_layer.objects.active = xi_obj
    xi_obj.select_set(True)

    ## add mesh
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16,
        diameter=diameterState)
    bm.to_mesh(xi_mesh)
    bm.free()

    bpy.ops.object.modifier_add(type='SUBSURF')
    bpy.ops.object.shade_smooth()

    ## add color
    mat = bpy.data.materials.new("PKHG")
    mat.diffuse_color = color
    mat.metallic = 0.0
    mat.specular_intensity = 0.0
    xi_obj.active_material = mat


def addState(x_1, x_2, name, color=(1.0, 1.0, 1.0, 1.0)):

    pos = getVectorOnStrip(x_1, x_2)
    addSphere(pos, name, color)

def addStateTorus(u, v, color=(1.0,1.0,1.0,1.0)):
    r0 = torusMajorRadius
    r1 = torusMinorRadius
    x = ((r0 + r1*cos(2*pi*v))*cos(2*pi*u), \
         (r0 + r1*cos(2*pi*v))*sin(2*pi*u), \
         r1*sin(2*pi*v))
    pos = Vector((x[0]+torusLocation[0],x[1]+torusLocation[1],x[2]+torusLocation[2]))
    addSphere(pos, "x_on_torus", color)

def addStateAnnulusLeft(x_1, name, color=(1.0, 1.0, 1.0, 1.0)):
    addStateAnnulus(x_1, name, color, offsetAnnulusLeft)

def addStateAnnulusRight(x_1, name, color=(1.0, 1.0, 1.0, 1.0)):
    addStateAnnulus(x_1, name, color, offsetAnnulusRight)

def addStateAnnulus(x_1, name, color=(1.0, 1.0, 1.0, 1.0), offset =
    offsetAnnulusLeft):

    theta = x_1*(2*pi)
    rot = Matrix.Rotation(theta, 3, [0,0,1])
    pos = (rot @ Vector([major_radius, 0, 0])) + offset
    addSphere(pos, name, color)


def addLightSource():

    light_data = bpy.data.lights.new(name="light_2.80", type='SUN')
    light_data.energy = 5
    light_object = bpy.data.objects.new(name="light_2.80",
      object_data=light_data)
    bpy.context.collection.objects.link(light_object)
    light_object.location = (0, 0, 20)

    light_2 = bpy.data.lights.new(name="light_source", type='AREA')
    light_2.energy = 400
    light_2.size = 13
    light_2_object = bpy.data.objects.new(name="light_source",
      object_data=light_2)
    bpy.context.collection.objects.link(light_2_object)
    light_2_object.location = offsetAnnulusLeft + Vector((0, 0, +1))

    light_3 = bpy.data.lights.new(name="light_source", type='AREA')
    light_3.energy = 400
    light_3.size = 13
    light_3_object = bpy.data.objects.new(name="light_source",
      object_data=light_2)
    bpy.context.collection.objects.link(light_3_object)
    light_3_object.location = offsetAnnulusRight + Vector((0, 0, +1))

def addCamera(location):
    cam = bpy.data.cameras.new("Camera")
    cam_ob = bpy.data.objects.new("Camera", cam)
    bpy.context.collection.objects.link(cam_ob)
    cam_ob.location = location
    bpy.context.scene.camera = cam_ob
    bpy.context.view_layer.objects.active = cam_ob
    update_camera(cam_ob, focus_point=cameraFocusPoint, distance=distanceCamera)

def torusRestriction_mesh(torusLocation):
    r0 = torusMajorRadius
    r1 = torusMinorRadius
    U = np.arange(0.55, 0.86, 0.01)
    V = np.arange(0,1.01,0.01)
    ctr = 0
    verts = []
    faces = []
    for u in U:
      M = len(verts)
      for v in V:
        x = ((r0 + r1*cos(2*pi*v))*cos(2*pi*u), \
             (r0 + r1*cos(2*pi*v))*sin(2*pi*u), \
             r1*sin(2*pi*v))
        x = Vector((x[0]+torusLocation[0],x[1]+torusLocation[1],x[2]+torusLocation[2]))
        verts.extend([x])

      N = len(V)
      if ctr < len(U) - 1:
          for k in range(M,M + N - 1):
            faces.append( [k,k+1,k+N])
            faces.append( [k+1,k+1+N, k+N])

      ctr = ctr + 1

    mesh = bpy.data.meshes.new("trestrict")
    mesh.from_pydata(verts, [], faces)

    for p in mesh.polygons:
        p.use_smooth=True

    return mesh


def addTorus(position):

    torus_mesh = bpy.ops.mesh.primitive_torus_add(major_radius = torusMajorRadius,
        minor_radius = torusMinorRadius-0.05,
        major_segments = 16, minor_segments = 8, location=position)
    torus_obj = bpy.context.object
    torus_obj.name = 'Torus'
    torus_obj.data.materials.append(materialMagenta)

def drawScene(scn, resolution, thick):

    ###########################################################################
    ### DELETE ALL OBJECTS
    ###########################################################################
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    ###########################################################################
    ### Add Light
    ###########################################################################
    addLightSource()

    ###########################################################################
    ### Add Camera
    ###########################################################################
    addCamera(cameraLocation)

    ###########################################################################
    ### MOBIUS STRIP
    ###########################################################################

    mobius_strip_mesh = mobius_mesh(resolution, thick)
    mobius_strip_obj = bpy.data.objects.new("mobius strip", mobius_strip_mesh)
    bpy.context.collection.objects.link(mobius_strip_obj)
    bpy.context.view_layer.objects.active = mobius_strip_obj
    mod = mobius_strip_obj.modifiers.new("edge split", 'EDGE_SPLIT')

    mobius_strip_obj.data.materials.append(materialMagenta)
    # mobius_strip_obj.show_transparent = True

    ###########################################################################
    ### ANNULUS LEFT
    ###########################################################################
    annulus_strip_mesh = annulus_mesh(resolution, major_radius -
        circleThickness, major_radius + circleThickness)
    annulus_strip_obj = bpy.data.objects.new("annulus", annulus_strip_mesh)
    annulus_strip_obj.location = offsetAnnulusLeft

    bpy.context.collection.objects.link(annulus_strip_obj)
    bpy.context.view_layer.objects.active = annulus_strip_obj
    mod = annulus_strip_obj.modifiers.new("edge split", 'EDGE_SPLIT')

    annulus_strip_obj.data.materials.append(materialMagenta)

    ###########################################################################
    ### ANNULUS RIGHT
    ###########################################################################
    annulus_r_strip_mesh = annulus_mesh(resolution, major_radius -
        circleThickness, major_radius + circleThickness)
    annulus_r_strip_obj = bpy.data.objects.new("annulus", annulus_r_strip_mesh)
    annulus_r_strip_obj.location = offsetAnnulusRight

    bpy.context.collection.objects.link(annulus_r_strip_obj)
    bpy.context.view_layer.objects.active = annulus_r_strip_obj
    mod = annulus_r_strip_obj.modifiers.new("edge split", 'EDGE_SPLIT')

    annulus_r_strip_obj.data.materials.append(materialMagenta)

    ###########################################################################
    ### ARROW
    ###########################################################################
    arrow_strip_mesh = arrow_mesh(Vector((0,0,+7.5)), length = 2.0, width = 0.5)
    arrow_strip_obj = bpy.data.objects.new("arrow", arrow_strip_mesh)
    arrow_strip_obj.location = offsetAnnulusLeft + Vector((0,-0.8,0))
    bpy.context.collection.objects.link(arrow_strip_obj)
    bpy.context.view_layer.objects.active = arrow_strip_obj

    arrow_r_strip_mesh = arrow_mesh(Vector((0,0,+8.0)), length = 2.0, width = 0.5)
    arrow_r_strip_obj = bpy.data.objects.new("arrow", arrow_r_strip_mesh)
    arrow_r_strip_obj.location = offsetAnnulusRight + Vector((0,0.8,0))
    bpy.context.collection.objects.link(arrow_r_strip_obj)
    bpy.context.view_layer.objects.active = arrow_r_strip_obj


    addTorus(torusLocation)

    restriction_torus_mesh = torusRestriction_mesh(torusLocation)
    restriction_torus_obj = bpy.data.objects.new("arrow", restriction_torus_mesh)
    # restriction_torus_obj.location = offsetAnnulusLeft + Vector((0,-0.8,0))
    bpy.context.collection.objects.link(restriction_torus_obj)
    restriction_torus_obj.data.materials.append(materialGreen)

    ###########################################################################
    ### INITIAL STATE
    ###########################################################################

    addState(0.45, 0.1, "xi", color=colorStartState)
    addState(0, 0.9, "xg", color=colorGoalState)

    addStateAnnulusLeft(0.45, "xi", color=colorStartState)
    addStateAnnulusLeft(0, "xg", color=colorGoalState)

    addStateAnnulusRight(0.55, "xi", color=colorStartState)
    addStateAnnulusRight(0.85, "xg", color=colorGoalState)

    addStateTorus(0.55, 0.1, color=colorStartState)
    addStateTorus(0.85, 0.4, color=colorGoalState)

    addPath()
    addPathAnnulus()

    addPathTorusAnnulus()

    ### RESET ACTIVE OBJECT TO MOBIUS STRIP
    bpy.context.view_layer.objects.active = mobius_strip_obj

    ### SAVE IMAGE
    bpy.context.scene.render.film_transparent = True
    bpy.context.scene.render.filepath = '/home/aorthey/image.svg'
    sceneR = bpy.context.scene


    # exporter = ExporterToSVG('/home/aorthey/image.svg')
    # exporter.execute(bpy.context, bpy.data, bpy.ops)

    # bpy.ops.render.render(write_still = True)


drawScene(bpy.context.scene, 72, mobiusStripThickness)
