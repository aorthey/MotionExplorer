import sys
import os
import numpy as np
from xml.dom.minidom import parse

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
from matplotlib.patches import FancyBboxPatch

from matplotlib.lines import Line2D 
from matplotlib.ticker import MaxNLocator

from matplotlib import rc
rc('font',**{'family':'cm','sans-serif':['Helvetica']})
# for Palatino and other serif fonts use:
# rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)
fontsize = 30
rc('text.latex', preamble=r'\usepackage{amsmath},\usepackage{amssymb}') 
rc('mathtext', **{'fontset':'cm', 'sf':'cm', 'it':'cm:italic'})
# rc('font', family='cm', size=fontsize)

# mathtext.it  : sans:italic
# mathtext.bf  : sans:bold
# mathtext.sf  : sans
# mathtext.fontset : cm



borderWidth = 2
colorMidLines = (0.2,0.2,0.2)
colorBundleRectangles = (0.9, 0.9, 0.9)
colorFiberRectangles = (1.0, 1.0, 1.0)

def GetFiberType(tbase, tbundle):
  ftype=""
  # print(tbase, tbundle)
  if tbundle[0] == "R":
    if tbase[0] == "R":
      r1 = int(tbundle[1])
      r2 = int(tbase[1])
      ftype = "R"+str(int(r1-r2))
  elif tbundle == "SE3":
    if tbase == "R3":
      ftype = "SO3"
  elif tbundle == "SE2":
    if tbase == "R2":
      ftype = "SO2"
  elif tbundle == "TSE2":
    if tbase == "R2":
      ftype = "SO2RN"

  elif tbundle[0:4] == "SE2R":
    rn = tbundle[4:]
    if tbase == "R2":
      ftype = "SO2R"+rn
    elif tbase == "SE2":
      ftype = "R"+rn
    elif tbase[0:4] == "SE2R":
      ftype = "SE2R"+rn
  elif tbundle == "SE3RN":
    if tbase == "R3":
      ftype = "SO3RN"
    elif tbase == "SE3":
      ftype = "RN"
  elif tbundle == "Bundle":
    if tbase == "Base":
      ftype = "Fiber"
  else:
    print("Could not determine fiber type")
    print(tbase, tbundle)
    sys.exit(0)

  if ftype=="":
    print("Could not determine fiber type")
    print(tbase, tbundle)
    sys.exit(0)

  return ftype


def typeToLatex(typespace):
  if typespace[0] == "R":
    rn = int(typespace[1])
    length = rn
    text = r'$\mathrm{\mathbb{R}}^{'+str(rn)+'}$'
  elif typespace[0:4] == "SE2R":
    print(typespace)
    if typespace[4] == "N":
      rn = 6
      length = 3 + rn
    else:
      rn = int(typespace[4:])
      length = 3+rn
    text = r'$SE(2) \times \mathrm{\mathbb{R}}^{'+str(rn)+'}$'
  elif typespace[0:4] == "SO2R":
    rn = int(typespace[4])
    length = 1+rn
    text = r'$SO(2) \times \mathrm{\mathbb{R}}^{'+str(rn)+'}$'
  elif typespace[0:4] == "SE3R":
    rn = int(typespace[4])
    length = 6 + rn
    text = r'$SE(3) \times \mathrm{\mathbb{R}}^{'+str(rn)+'}$'
  elif typespace == "SO2RN":
    rn = 6
    length = rn + 1
    text = r'$SO(2)\times \mathrm{\mathbb{R}}^{'+str(rn)+'}$'
  elif typespace == "SE2":
    length = 3
    text = r'$SE(2)$'
  elif typespace == "SO2":
    length = 1
    text = r'$SO(2)$'
  elif typespace == "SE3":
    length = 6
    text = r'$SE(3)$'
  elif typespace == "SO3":
    length = 3
    text = r'$SO(3)$'
  elif typespace == "TSE2":
    length = 6
    text = r'$TSE(2)$'
  elif typespace == "TSE3":
    length = 12
    text = r'$TSE(3)$'
  elif typespace == "Bundle":
    length = 6
    text = r'{\text{Bundle}}'
  elif typespace == "Base":
    length = 3
    text = r'{\text{Base}}'
  elif typespace == "Fiber":
    length = 3
    text = r'{\text{Fiber}}'
  else:
    print("not knowing type",typespace)
    sys.exit(0)
  return [length, text]

class RectangleSpace(object):

  def __init__(self, rid, typespace, sid, fontsize):
    self._dashedOnly = False
    self._rid = rid
    self._typespace = typespace
    self._sid = sid
    self._fontsize = fontsize
    self._fiberSpace = False
    
    self._xoffset = 0
    [self._length, self._text] = typeToLatex(typespace)

  def setXOffset(self, xoff):
    self._xoffset = xoff

  def setXY(self, x, y, xstep, ystep):
    self._x = x
    self._y = y
    self._xstep = xstep
    self._ystep = ystep
    self._height = 0.5*self._ystep

  def SetPatch(self, ax):
    offset = 0.05*self._length*self._xstep
    tx = self._x + 0.4*self._length*self._xstep - offset - self._xoffset
    ty = self._y + 0.3*self._height

    if self._sid:
      x1 = self._x
      x2 = x1 + self._length*self._xstep
      x3 = x1 + self._fulllength*self._xstep
      y1 = self._y
      y2 = y1 - (self._ystep - self._height)
      length_x = (self._fulllength-self._length)*self._xstep

      if self._dashedOnly:
        x2 = x1
        length_x = self._length*self._xstep

      rect = patches.Rectangle(
              (x2,y1), length_x, self._height,
              linestyle="--", linewidth= borderWidth, 
              facecolor=colorFiberRectangles, fill=False, edgecolor='k')
      ax.add_patch(rect)


      ### Inside of Fiber Bundle Rectangle
      if length_x > 0:
        ##non empty projection
        [tmp, self._fibertext] = typeToLatex(self._fibertype)
        tx2 = x2 + 0.4*length_x - offset - self._xoffset
        if self._fibertype[0] == "S":
          tx2 = x2 + 0.2*length_x - offset - self._xoffset
        ax.text(tx2, ty, self._fibertext, {'color': 'k', 'fontsize':
          self._fontsize},
            bbox=dict(boxstyle='square, pad=0.0', facecolor=colorFiberRectangles, edgecolor='None', alpha=0.9))

        for k in range(1,int(self._length)):
          xl = [x2 + k*self._xstep, x2 + k*self._xstep]
          yl = [self._y, self._y+self._height]
          line = Line2D(xl, yl, linewidth=borderWidth, linestyle="--",
              color=colorMidLines)
          ax.add_line(line)

      ### Draw Arrow Between Non-fiber simplifications
      dy = self._ystep - self._height
      offset = 0.1*dy
      y1 = self._y + self._height + dy - offset

      ### Base Projection Arrows
      if not self._dashedOnly:
        fcolor = (0.0,0.0,0.0)
        xa = x1 + 0.5*(x2-x1)
        ya = y1

        plt.arrow(xa, ya, 0, -dy + 2*offset, width = 0.01*dy,
            length_includes_head = True, head_length = 0.3*dy,
            head_starts_at_zero = False,
            facecolor=fcolor,
            edgecolor=(0.0,0.0,0.0),
            head_width = 0.15*dy)

      ### Fiber Projection Arrows
      if length_x > 0:
        fcolor = (1.0,1.0,1.0)
        xa = x2 + 0.5*length_x
        ya = y1

        # ystart = y1
        # yend = y1 - dy + 2*offset
        yend = y1
        ystart = y1 - dy + 2*offset

        plt.arrow(xa, ystart, 0, yend - ystart, width = 0.01*dy,
            length_includes_head = True, head_length = 0.3*dy,
            head_starts_at_zero = False,
            facecolor=fcolor,
            edgecolor=(0.0,0.0,0.0),
            head_width = 0.15*dy)

    ### Draw Rectangle for Base and Bundle Spaces
    if not self._dashedOnly:
      rect = patches.Rectangle(
              (self._x,self._y), self._length*self._xstep, self._height,
              linewidth=borderWidth, fill=True, edgecolor='k',
              facecolor=colorBundleRectangles)
      ax.add_patch(rect)
      ax.text(tx, ty, self._text, 
          {'color': 'k', 'fontsize': self._fontsize},
          bbox=dict(boxstyle='square, pad=0.0', facecolor=colorBundleRectangles, edgecolor='None', alpha=0.9)
          )

      for k in range(1,int(self._length)):
        xl = [self._x + k*self._xstep, self._x + k*self._xstep]
        yl = [self._y, self._y+self._height]
        line = Line2D(xl, yl, linewidth=0.5*borderWidth, linestyle="-",
            color=colorMidLines)
        ax.add_line(line)

  def setDashed(self):
    self._dashedOnly = True

def PlotFiberDiagram(fname, fontsize, ystep=20, xstep=10, xoffset=0):
  doc = parse(fname)
  hierarchies = doc.getElementsByTagName("hierarchy")
  hctr = 1

  for hierarchy in hierarchies:
    levels = hierarchy.getElementsByTagName("level")

    fig = plt.figure(0)
    fig.patch.set_facecolor('white')
    ax = plt.gca()

    height = 0.5 * ystep
    Nx = 0

    rectangleSpaces = []
    y = ystep*(len(levels)-1) + 2*borderWidth
    dx = 0
    dy = ystep*(len(levels)-1) + height + 2*borderWidth

    rectanglesLastLevel = []
    for level in levels[::-1]:
      robots = level.getElementsByTagName("robot")
      if not robots:
        print("No robot found for file ",fname)
        return

      x = 0.0

      Nr = len(robots)
      for robot in robots:
        rid = int(robot.getAttribute("id"))
        t = str(robot.getAttribute("type"))
        sid = robot.getAttribute("simplification_of_id")
        rect = RectangleSpace(rid, t, sid, fontsize)
        rect.setXOffset(xoffset)
        rect.setXY(x, y, xstep, ystep)

        if sid:
          sid = int(sid)
          for rectLL in rectanglesLastLevel:
            if sid == rectLL._rid:
              rect._x = rectLL._x
              rect._fulllength = rectLL._length
              rect._fibertype = GetFiberType(rect._typespace, rectLL._typespace)

        rectangleSpaces.append(rect)
        x = x + (rect._length * xstep)

      if len(robots) < len(rectanglesLastLevel):
        # print("Adding Additional Robots")
        for rectLL in rectanglesLastLevel:
          found = False
          for robot in robots:
            sid = robot.getAttribute("simplification_of_id")
            if sid:
              sid = int(sid)
            if rectLL._rid == sid:
              found = True
              break
          if not found:
            if not rectLL._fiberSpace:
              # print("Robot",rectLL._rid,"projects to empty set")
              rect = RectangleSpace(rectLL._rid, rectLL._typespace, rectLL._rid,
                  fontsize)
              rect._fiberSpace = True
              rect.setXOffset(xoffset)
              rect._fulllength = rectLL._length
              rect.setXY(rectLL._x, y, rectLL._xstep, rectLL._ystep)
              rect.setDashed()
              rect._fibertype = rectLL._typespace
              rectangleSpaces.append(rect)
              Nr = Nr+1

      if x>dx:
        dx = x
      y = y - ystep

      rectanglesLastLevel.clear()
      for k in range(1,Nr+1):
        rectanglesLastLevel.append(rectangleSpaces[-k])

      # print("Added Rectangles:")
      # for rect in rectanglesLastLevel:
      #   print(rect._rid,"->",rect._sid)

    for rect in rectangleSpaces:
      rect.SetPatch(ax)

    plt.axis([0,dx,0,dy])
    plt.tight_layout()
    ax.set_aspect('equal')
    plt.axis('off')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)

    pngname = fname[:-4]+".png"

    if len(hierarchies) > 1:
      pngname = fname[:-4]+str(hctr)+".png"
      hctr=hctr+1

    plt.savefig(pngname, bbox_inches='tight', pad_inches=0)
    # plt.show()
    plt.clf()
    plt.cla()
    plt.close()


######################################################################
####def PlotFiberDiagram(fname, fontsize, ystep, xstep, xoffset=0):
######################################################################
folder = '../data/experiments/IJRR2020'

# PlotFiberDiagram('../data/experiments/WAFR2020/12D_drones_tree.xml', 25, 20, 8)
# for file in os.listdir(folder):
#   if file.endswith(".xml"):
#       fname = os.path.join(folder, file)
#       print(fname)
#       PlotFiberDiagram(fname, fontsize, ystep = 20, xstep = 10)

PlotFiberDiagram('../data/experiments/IJRR2020_EXT/testfiber.xml', 
    fontsize=35)
# PlotFiberDiagram('../data/experiments/IJRR2020/63D_SE2RN_R2_mobile_manipulators.xml',
#     fontsize=5,
#     ystep=120, 
#     xstep=50,
#     xoffset=-0)
# PlotFiberDiagram('../data/experiments/IJRR2020/68D_SE2RN_SE2RM_PR2.xml',
#     fontsize=15)
