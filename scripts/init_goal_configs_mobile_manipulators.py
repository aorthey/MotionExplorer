import random
import numpy as np

def printRobot(rid, xi, yi, xg, yg):
  qinit="13  "+str(xi)+" "+str(yi)+" 0 0 0 0 0 0 -0.1 0 +0.0 0 0"
  qgoal="13  "+str(xg)+" "+str(yg)+" 0 0 0 0 0 0 -0.5 0 -0.5 0 0"
  print("<agent id=\"", rid, "\" \nqinit=\"", qinit, "\" \nqgoal=\"", qgoal, "\"/>", sep="")

pts = []
N = 8
theta = 0.0
radius = 3
for rid in range(0,2*N,2):
  xi = round(radius*np.cos(theta), 2)
  yi = round(radius*np.sin(theta), 2)
  xg = round(radius*np.cos(theta+np.pi), 2)
  yg = round(radius*np.sin(theta+np.pi), 2)
  X = np.array([xi,yi,xg,yg])

  theta = theta + 2*np.pi/N
  pts.append(X)
  printRobot(rid, xi, yi, xg, yg)

