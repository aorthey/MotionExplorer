from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
import matplotlib.patches as patches

boundLow=0
boundHigh=2
 
def isInsideBox(x, y, xcenter, ycenter, xlength, ylength):
  xl = xcenter-0.5*xlength
  xu = xcenter+0.5*xlength
  yl = ycenter-0.5*ylength
  yu = ycenter+0.5*ylength
  return (x > xl and x < xu) and (y > yl and y < yu)

class Constraint():
  box_constraints = []
  def __init__(self):
    self.box_constraints.append([0.5,0.27,1,0.5])
    self.box_constraints.append([1.5,0.5,0.5,0.5])
    self.box_constraints.append([1.25,1.5,0.8,0.5])


  def draw(self):
    ax = plt.gca()

    for c in self.box_constraints:
      xl = c[0]-0.5*c[2]
      yl = c[1]-0.5*c[3]
      rect = patches.Rectangle((xl, yl), c[2], c[3], linewidth=2, edgecolor='r',
          facecolor='none')
      ax.add_patch(rect)

  def isStateValid(self,state):
    x = state[0]
    y = state[1]
    for c in self.box_constraints:
      if isInsideBox(x, y, c[0],c[1],c[2],c[3]):
        return False
    return True
 
def drawVertex(v,style='og'):
  state = v.getState()
  plt.plot(state[0], state[1], style)

def drawStartVertex(v):
  state = v.getState()
  plt.plot(state[0], state[1], 'og', markersize=20)

def drawGoalVertex(v):
  state = v.getState()
  plt.plot(state[0], state[1], 'or', markersize=15)

def drawEdge(v1, v2, style='-g'):
  s1 = v1.getState()
  s2 = v2.getState()
  plt.plot([s1[0],s2[0]],[s1[1],s2[1]],style)

def drawPath(p):
  N = p.getStateCount()
  for n in range(0,N-1):
    s1 = p.getState(n)
    s2 = p.getState(n+1)
    plt.plot([s1[0],s2[0]],[s1[1],s2[1]],'-m',linewidth=6)

if __name__ == "__main__":
  # create an SE2 state space
  space = ob.RealVectorStateSpace(2)

  # set lower and upper bounds
  bounds = ob.RealVectorBounds(2)
  bounds.setLow(boundLow)
  bounds.setHigh(boundHigh)
  space.setBounds(bounds)

  axes = plt.gca()
  axes.set_xlim([boundLow,boundHigh])
  axes.set_ylim([boundLow,boundHigh])

  # create a simple setup object
  ss = og.SimpleSetup(space)
  c = Constraint()
  ss.setStateValidityChecker(ob.StateValidityCheckerFn(c.isStateValid))

  start = ob.State(space)
  start()[0] = 0
  start()[1] = 0

  goal = ob.State(space)
  goal()[0] = 2
  goal()[1] = 1.5

  ss.setStartAndGoalStates(start, goal)

  si = ss.getSpaceInformation()
  si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocMyValidStateSampler))

  planner = og.RRTstar(si)
  planner = og.PRM(si)
  ss.setPlanner(planner)

  solved = ss.solve(0.05)

  c.draw()

  if solved:
    ss.simplifySolution()
    pd = ob.PlannerData(ss.getSpaceInformation())
    ss.getPlannerData(pd)
    pd.computeEdgeWeights()

    Vn = pd.numVertices()
    for n in range(0,Vn):
      v = pd.getVertex(n)
      drawVertex(v)

    Ve = pd.numEdges()
    for i in range(0,Vn):
      for j in range(0,Vn):
        if pd.edgeExists(i,j):
          v1 = pd.getVertex(i)
          v2 = pd.getVertex(j)
          drawEdge(v1, v2)


    p = ss.getSolutionPath()
    drawPath(p)

    vi = pd.getStartVertex(0)
    drawStartVertex(vi)
    vg = pd.getGoalVertex(0)
    drawGoalVertex(vg)

  plt.show()

