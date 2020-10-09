import xml.etree.ElementTree as ET
import sys, os
import numpy as np

def getRoadmapStates(fname):
    root = ET.parse(fname).getroot()

    x = []
    for state in root.findall('state'):
      x.append( np.fromstring( state.text, dtype=np.float, sep=' ' ) )

    x = np.array(x)
    return x[:,1:]

def getRoadmapEdges(fname):
    root = ET.parse(fname).getroot()

    x = []
    for edge in root.findall('edge'):
      e = []
      for state in edge.findall('state'):
        e.append( np.fromstring( state.text, dtype=np.float, sep=' ' ) )
      e = np.array(e)
      x.append(e[:,1:])

    x = np.array(x)
    return x
