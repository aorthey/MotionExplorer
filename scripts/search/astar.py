import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import sys

from enum import Enum

CellStatus = Enum('CellStatus', 'Free Occupied Robot Path')

## state space is array of size WxH
W = 30
H = 10

## 4-neighbor set construction
def getNeighbors(x,y):
  Xn = []
  if x > 0:
    Xn.append([x-1,y])
  if x < H-1:
    Xn.append([x+1,y])
  if y > 0:
    Xn.append([x,y-1])
  if y < W-1:
    Xn.append([x,y+1])
  return Xn

def search_Astar(X, x1, y1, x2, y2):
  Xn = getNeighbors(x1, y1)
  print(Xn)

def drawCell(ax, x, y, color='k'):
  rect = patches.Rectangle((y/W,x/H),1/W,1/H,facecolor=color)
  ax.add_patch(rect)

# X = np.random.randint(1, 3, size=(H,W))
X = (np.random.rand(H,W)>0.8).astype(int) + 1

X[0,0] = CellStatus.Robot.value
X[H-1,W-1] = CellStatus.Robot.value

fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax  = fig.gca()

for i in range(0,H):
  for j in range(0,W):
    if X[i,j] == CellStatus.Occupied.value:
      drawCell(ax, i, j, 'k')
    elif X[i,j]==CellStatus.Robot.value:
      drawCell(ax, i, j, 'r')
    elif X[i,j]==CellStatus.Path.value:
      drawCell(ax, i, j, 'm')

search_Astar(X,0,0,H-1,W-1)

plt.show()
