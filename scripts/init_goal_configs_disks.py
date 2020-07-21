import random
import numpy as np

def printRobot(rid, xi, yi, xg, yg):
  qinit="6  "+str(xi)+" "+str(yi)+" 0 0 0 0"
  qgoal="6  "+str(xg)+" "+str(yg)+" 0 0 0 0"
  print("<agent id=\"", rid, "\" qinit=\"", qinit, "\" qgoal=\"", qgoal, "\"/>", sep="")
    
def IsCollisionFree(pts, X, threshold=0.6):
  for P in pts:
    PI = P[0:2]
    PG = P[2:4]
    XI = X[0:2]
    XG = X[2:4]
    if(np.linalg.norm(XI-PI) < threshold or
        np.linalg.norm(XG-PG) < threshold):
      return False
  return True

pts = []
for rid in range(0,25):
  found=False
  while found==False:
    xi = round(random.uniform(4,5.5),2)
    yi = round(random.uniform(-1,8),2)
    xg = round(random.uniform(-4,-5),2)
    yg = round(random.uniform(-1,8),2)
    X = np.array([xi,yi,xg,yg])
    found = IsCollisionFree(pts, X)

  pts.append(X)
  printRobot(rid, xi, yi, xg, yg)

