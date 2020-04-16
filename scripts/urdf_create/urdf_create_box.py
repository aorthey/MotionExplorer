import os 
import numpy as np
from math import cos,sin,pi,atan2
from urdf_create import *
from urdf_create_primitives import *

#box parameters
width = 1.0
length = 2.0
height = 0.7
thickness = 0.05


sideLength = 0.8*width/2
sideHeight = 0.9*height
padding = 1.3*thickness

lengthZside = 0.33*length
topPadding = padding
topLength = length-4*topPadding
topWidth = width-topPadding
topHeight = 0.3*height

def AddBottomSides():
    ### Bottom
    hstr   = createCuboid("bottom", 0, 0, 0, length, width, thickness)

    ###############################################################################
    #### SIDE B
    ### Hinge (Bottom-SideB)
    hstr  += createCylinder("bottom-sideB", 0, -width/2-thickness/2, 0, thickness/2,
        length)
    hstr  += createRigidJoint("bottom", "bottom-sideB")
    y = -thickness/2 - height/2
    hstr  += createCuboid("sideB", 0, y, 0, length, height, thickness)
    hstr  += createRevoluteJointX("bottom-sideB", "sideB", x=0, y=-width/2-thickness/2)
    ###############################################################################

    ###############################################################################
    ### SideA
    ### Hinge (Bottom-SideA)
    # hstr  += createCylinder("bottom-sideA", 0, width/2+thickness/2, 0, thickness/2,
    #     length)
    # hstr  += createRigidJoint("bottom", "bottom-sideA")

    # y = thickness/2 + height/2
    # hstr  += createCuboid("sideA", 0, y, 0, length, height, thickness)
    # hstr  += createRevoluteJointX("bottom-sideA", "sideA", x=0, y=width/2+thickness/2)


    hstr  += createCylinder("bottom-sideALEFT", -lengthZside, width/2+thickness/2, 0, thickness/2,
        lengthZside-0.1)
    hstr  += createRigidJoint("bottom", "bottom-sideALEFT")

    y = thickness/2 + height/2
    hstr  += createCuboid("sideALEFT", -lengthZside, y, 0, lengthZside, height, thickness)
    hstr  += createRevoluteJointX("bottom-sideALEFT", "sideALEFT", x=0, y=width/2+thickness/2)


    hstr  += createCylinder("bottom-sideARIGHT", +lengthZside, width/2+thickness/2, 0, thickness/2,
        lengthZside-0.1)
    hstr  += createRigidJoint("bottom", "bottom-sideARIGHT")

    y = thickness/2 + height/2
    hstr  += createCuboid("sideARIGHT", +lengthZside, y, 0, +lengthZside, height, thickness)
    hstr  += createRevoluteJointX("bottom-sideARIGHT", "sideARIGHT", x=0, y=width/2+thickness/2)
    ###############################################################################
    ###############################################################################

    #LEFT SIDE
    hstr  += createRotatedCylinder("sideA-left", -length/2-thickness/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, height)
    hstr  += createRigidJoint("sideALEFT", "sideA-left", y=height/2+thickness/2)

    hstr  += createCuboid("sideA1", -thickness/2-sideLength/2, 0, 0,
        sideLength, sideHeight, thickness)
    hstr  += createRevoluteJointY("sideA-left", "sideA1", x=-length/2-thickness/2)

    #RIGHT SIDE
    hstr  += createRotatedCylinder("sideA-right", +length/2+thickness/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, height)
    hstr  += createRigidJoint("sideARIGHT", "sideA-right", y=height/2+thickness/2)

    hstr  += createCuboid("sideA2", +thickness/2+sideLength/2, 0, 0,
        sideLength, sideHeight, thickness)
    hstr  += createRevoluteJointY("sideA-right", "sideA2", x=+length/2+thickness/2)

    #LEFT SIDE B
    hstr  += createRotatedCylinder("sideB-left", -length/2-thickness/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, height)
    hstr  += createRigidJoint("sideB", "sideB-left", y=-height/2-thickness/2)

    hstr  += createCuboid("sideB1", -thickness/2-sideLength/2, 0, 0,
        sideLength, sideHeight, thickness)
    hstr  += createRevoluteJointY("sideB-left", "sideB1", x=-length/2-thickness/2)

    #RIGHT SIDE B
    hstr  += createRotatedCylinder("sideB-right", +length/2+thickness/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, height)
    hstr  += createRigidJoint("sideB", "sideB-right", y=-height/2-thickness/2)

    hstr  += createCuboid("sideB2", +thickness/2+sideLength/2, 0, 0,
        sideLength, sideHeight, thickness)
    hstr  += createRevoluteJointY("sideB-right", "sideB2", x=+length/2+thickness/2)
    return hstr

def AddTopPreside():
    hstr  = createCylinder("zbottom-ZsideA", 0, width/2+thickness/2, 0, thickness/2,
        lengthZside-0.05)
    hstr  += createRigidJoint("bottom", "zbottom-ZsideA",0,0,0,"")

    y = thickness/2 + height/2
    hstr  += createCuboid("ZsideA", 0, y, 0, lengthZside-0.05, height, thickness)
    hstr  += createRevoluteJointX("zbottom-ZsideA", "ZsideA", x=0, y=width/2+thickness/2)

    y = height/2 + thickness/2
    hstr  += createCylinder("ZsideA-top", 0, y, 0, thickness/2, lengthZside-0.05)
    hstr  += createRigidJoint("ZsideA", "ZsideA-top", x=0, y=y)
    return hstr

def AddTop():
    ###############################################################################
    ## BOTTOM LEFT
    ###############################################################################
    ### Hinge (SideA-Top)

    ### SideA
    ### Hinge (Bottom-SideA)

    ### Top
    y = thickness/2 + width/2
    hstr  = createCuboid("Ztop", 0, thickness/2+topWidth/2, 0, topLength, topWidth, thickness)
    hstr  += createRevoluteJointX("ZsideA-top", "Ztop", x=0, y=height/2 + thickness/2)

    hstr  += createCylinder("ZT3-cylinder", 0, topWidth/2+thickness/2, 0,
        thickness/2, topLength-topPadding)
    hstr  += createRigidJoint("Ztop", "ZT3-cylinder", x=0,
        y=topWidth/2+thickness/2)

    hstr  += createCuboid("ZT3", 0, topHeight/2+thickness/2, 0,
        topLength-topPadding, topHeight, thickness)
    hstr  += createRevoluteJointX("ZT3-cylinder", "ZT3", x=0, y=topWidth/2 + thickness/2)
    return hstr

def AddTopSides():
    #LEFT SIDE
    sideLength = 0.1*height
    sideWidth = 0.6*topWidth
    # sideLength = 0.3*height
    # sideWidth = 0.5*topWidth
    hstr  = createRotatedCylinder("ZTop-left", -topLength/2-thickness/2,
        topWidth/2 + thickness/2,0,
        1.57, 0, 0,
        thickness/2, sideWidth)
    hstr  += createRigidJoint("Ztop", "ZTop-left")

    hstr  += createCuboid("ZT1", -thickness/2-sideLength/2,
        topWidth/2+thickness/2, 0,
        sideLength, sideWidth, thickness)
    hstr  += createRevoluteJointY("ZTop-left", "ZT1", x=-thickness/2-topLength/2)
    #RIGHT SIDE
    hstr  += createRotatedCylinder("ZTop-right", +topLength/2+thickness/2,
        topWidth/2 + thickness/2,0,
        1.57, 0, 0,
        thickness/2, sideWidth)
    hstr  += createRigidJoint("Ztop", "ZTop-right")

    hstr  += createCuboid("ZT2", +thickness/2+sideLength/2,
        topWidth/2+thickness/2, 0,
        sideLength, sideWidth, thickness)
    hstr  += createRevoluteJointY("ZTop-right", "ZT2", x=+thickness/2+topLength/2)
    return hstr

def AddLeftClamping():
    ###############################################################################
    ## BOTTOM LEFT
    ###############################################################################
    hstr  = createCuboid("padB1", -length/2-padding/2, 0, 0,
        padding, width, thickness)
    hstr  += createRigidJoint("bottom", "padB1")

    hstr  += createRotatedCylinder("padB-cylinder", -length/2-padding-thickness/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, width)
    hstr  += createRigidJoint("padB1", "padB-cylinder") 

    hstr  += createCuboid("B1", -thickness/2 - height/2, 0, 0,
         height, width, thickness)
    hstr  += createRevoluteJointY("padB-cylinder", "B1", x=-length/2 -
        padding-thickness/2)

    hstr  += createRotatedCylinder("padB2-cylinder", -thickness/2-height/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, width)
    hstr  += createRigidJoint("B1", "padB2-cylinder", x=-thickness/2-height/2) 

    doublePadding = padding + (padding-thickness)
    hstr  += createCuboid("padB2", -thickness/2 - doublePadding/2, 0, 0,
         doublePadding, width, thickness)
    hstr  += createRevoluteJointY("padB2-cylinder", "padB2", x=-thickness/2-height/2)

    ### Add B2 plus cylinder joint
    hstr  += createRotatedCylinder("B2-cylinder", -thickness/2-doublePadding/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, width)
    hstr  += createRigidJoint("padB2", "B2-cylinder", x=-thickness/2-doublePadding/2) 

    hstr  += createCuboid("B2", -thickness/2 - (height - doublePadding)/2, 0, 0,
         height - doublePadding, width, thickness)
    hstr  += createRevoluteJointY("B2-cylinder", "B2", x=-thickness/2-doublePadding/2)
    return hstr

def AddRightClamping():
    ###############################################################################
    ## BOTTOM LEFT
    ###############################################################################
    hstr  = createCuboid("padB3", +length/2+padding/2, 0, 0,
        padding, width, thickness)
    hstr  += createRigidJoint("bottom", "padB3")

    hstr  += createRotatedCylinder("padB3-cylinder", +length/2+padding+thickness/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, width)
    hstr  += createRigidJoint("padB3", "padB3-cylinder") 

    hstr  += createCuboid("B3", +thickness/2 + height/2, 0, 0,
         height, width, thickness)
    hstr  += createRevoluteJointY("padB3-cylinder", "B3", x=+length/2 +
        padding+thickness/2)

    hstr  += createRotatedCylinder("padB4-cylinder", +thickness/2+height/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, width)
    hstr  += createRigidJoint("B3", "padB4-cylinder", x=+thickness/2+height/2) 

    doublePadding = padding + (padding-thickness)
    hstr  += createCuboid("padB4", +thickness/2 + doublePadding/2, 0, 0,
         doublePadding, width, thickness)
    hstr  += createRevoluteJointY("padB4-cylinder", "padB4", x=+thickness/2+height/2)

    ### Add B2 plus cylinder joint
    hstr  += createRotatedCylinder("B4-cylinder", +thickness/2+doublePadding/2, 0, 0, 
        1.57, 0, 0,
        thickness/2, width)
    hstr  += createRigidJoint("padB4", "B4-cylinder", x=+thickness/2+doublePadding/2) 

    hstr  += createCuboid("B4", +thickness/2 + (height - doublePadding)/2, 0, 0,
         height - doublePadding, width, thickness)
    hstr  += createRevoluteJointY("B4-cylinder", "B4", x=+thickness/2+doublePadding/2)
    return hstr


###############################################################################
robot_name = 'box/box'
fname = getPathname(robot_name)
robot_name = 'box'

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr = AddBottomSides()
hstr += AddLeftClamping()
hstr += AddRightClamping()
hstr += AddTopPreside()
hstr += AddTop()
hstr += AddTopSides()

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)
###############################################################################
robot_name = 'box/box_bottom_left_right_top_sides'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr = AddBottomSides()
hstr += AddLeftClamping()
hstr += AddRightClamping()
hstr += AddTopPreside()
hstr += AddTop()

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)
###############################################################################
robot_name = 'box/box_bottom_left_right_top'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr = AddBottomSides()
hstr += AddLeftClamping()
hstr += AddRightClamping()
hstr += AddTopPreside()

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)

###############################################################################
robot_name = 'box/box_bottom_left_right'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr = AddBottomSides()
hstr += AddLeftClamping()
hstr += AddRightClamping()

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)

###############################################################################
robot_name = 'box/box_bottom_left'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr = AddBottomSides()
hstr += AddLeftClamping()

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)
###############################################################################
robot_name = 'box/box_bottom'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr = AddBottomSides()

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)

###############################################################################
robot_name = 'box/box_bottom_only'
fname = getPathname(robot_name)

f = open(fname,'w')
f.write('<?xml version="1.0"?>\n')
f.write('<robot name="'+robot_name+'">\n')

hstr   = createCuboid("bottom", 0, 0, 0, length, width, thickness)

f.write(hstr)
f.write('  <klampt package_root="../../.." default_acc_max="4" >\n')
f.write('  </klampt>\n')
f.write('</robot>')
f.close()

print("\nCreated new file >>",fname)
