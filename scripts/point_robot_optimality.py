import numpy as np
import sys
import matplotlib.pyplot as plt

#Let I=[0,1].
#every problem instance is uniquely described by x_0, x_1>x_0, x_I \in [x_0,x_1], x_G, all
#elements of I

epsilon = 1e-10
nd = 10
step_size = 1.0/nd

###############################################################################
## (1) generate all possible problem instances
###############################################################################
ctr0 = 0
## x_0 \in [0,1]
Ix0 = np.linspace(0,1,nd+1)
X = []
for x_0 in Ix0:
  ## x_1 \in [x_0, 1]
  nx1 = nd+1-ctr0
  Ix1 = np.linspace(x_0,1,nx1)
  #print "########################"
  #print x_0,Ix1
  ctr0 = ctr0 + 1
  ctr1 = 0
  for x_1 in Ix1:
    ## x_I \in [x_0, x_1]
    nx2 = ctr1+1
    Ix2 = np.linspace(x_0,x_1,nx2)
    ctr1 = ctr1 + 1
    #print "[",x_0, ",", x_1, "]", Ix2
    for x_I in Ix2:
      Ix3 = np.linspace(0,1,nd+1)
      for x_G in Ix3:
        #print "[",x_0, ",", x_1, "]", x_I, x_G
        X.append([x_0,x_1,x_I,x_G])


X = np.array(X)
print "#"*80
print "generated",X.shape[0]," problem instances"
print "#"*80

###############################################################################
## (2) for each problem instance compute number of steps
###############################################################################

def solveStrategy1_right_left(x_input):
  x_0 = x_input[0]
  x_1 = x_input[1]
  x_I = x_input[2]
  x_G = x_input[3]

  x = x_I
  direction = 1
  steps = 0
  #print "[",x_0,x_1,"]"," start:",x_I," goal:",x_G
  while abs(x-x_G)>epsilon:
    #print "x=",x
    if x-epsilon > x_1:
      direction = -direction
      x = x_I
    elif x+epsilon < x_0:
      return [steps, False]
    steps = steps+1
    x = x + direction*step_size
  return [steps, True]

def solveStrategy1_left_right(x_input):
  x_0 = x_input[0]
  x_1 = x_input[1]
  x_I = x_input[2]
  x_G = x_input[3]

  x = x_I
  direction = -1
  steps = 0
  while abs(x-x_G)>epsilon:
    if x+epsilon < x_0:
      direction = -direction
      x = x_I
    elif x-epsilon > x_1:
      return [steps, False]
    steps = steps+1
    x = x + direction*step_size
  return [steps, True]

def solveStrategy2(x_input):
  x_0 = x_input[0]
  x_1 = x_input[1]
  x_I = x_input[2]
  x_G = x_input[3]

  steps = 0
  #print "[",x_0,x_1,"]"," start:",x_I," goal:",x_G

  X_nn = np.array([x_I])

  x = x_I
  while True:
    ## random sample
    x_R = np.random.uniform()

    ## nearest in X_nn
    x_nearest = -1
    d = +np.infty

    #print "X_nn:",X_nn
    for x_nn in X_nn:
      d_nn = abs(x_R-x_nn)
      if d_nn < d:
        d_nn = d
        x_nearest = x_nn

    if x_nearest == -1:
      print "invalid"
      sys.exit(0)

    ## connect x_nearest to x_R
    direction = np.sign(x_R - x_nearest)

    #print "x_nearest:",x_nearest," to",x_R, "direction:",direction
    #print x_R,direction
    x = x_nearest
    while x-epsilon < x_1 and x+epsilon > x_0:
      if abs(x-x_G)<epsilon:
        return [steps, True]
      np.append(X_nn,x)
      X_nn = np.unique(X_nn)
      steps = steps+1
      x = x + direction*step_size
      x = np.around(x, decimals=1)
      #print x,x_G,abs(x-x_G)

  return [steps, False]

### RRT + goal bias
def solveStrategy3(x_input):
  x_0 = x_input[0]
  x_1 = x_input[1]
  x_I = x_input[2]
  x_G = x_input[3]

  steps = 0

  X_nn = np.array([x_I])

  x = x_I
  while True:
    ## random sample
    t = np.random.uniform()
    if t < 0.05:
      x_R = x_G
    else:
      x_R = np.random.uniform()


    ## nearest in X_nn
    x_nearest = -1
    d = +np.infty

    #print "X_nn:",X_nn
    for x_nn in X_nn:
      d_nn = abs(x_R-x_nn)
      if d_nn < d:
        d_nn = d
        x_nearest = x_nn

    if x_nearest == -1:
      print "invalid"
      sys.exit(0)

    ## connect x_nearest to x_R
    direction = np.sign(x_R - x_nearest)

    #print "x_nearest:",x_nearest," to",x_R, "direction:",direction
    #print x_R,direction
    x = x_nearest
    while x-epsilon < x_1 and x+epsilon > x_0:
      if abs(x-x_G)<epsilon:
        return [steps, True]
      np.append(X_nn,x)
      X_nn = np.unique(X_nn)
      steps = steps+1
      x = x + direction*step_size
      x = np.around(x, decimals=1)
      #print x,x_G,abs(x-x_G)

  return [steps, False]


solveable = 0
unsolveable = 0
X_solveable = []
for x in X:
  x_G = x[3]
  if x_G>x[1] or x_G < x[0]:
    unsolveable = unsolveable+1
  else:
    solveable = solveable + 1
    X_solveable.append(x)
print solveable+unsolveable, " problem instances with ", solveable, " solveable and ", unsolveable, " unsolveable problems."
print "#"*80

X_solveable = np.array(X_solveable)

#print solveStrategy2(X_solveable[1167])
#sys.exit(0)
Q1 = []
Q2 = []
Q3 = []
Q4 = []
for k in range(0,X_solveable.shape[0]):
  if k%100==0:
    print "solving ", k,"/",X_solveable.shape[0]
  Q1.append(solveStrategy1_left_right(X_solveable[k]))
  Q2.append(solveStrategy1_right_left(X_solveable[k]))
  Q3.append(solveStrategy2(X_solveable[k]))
  Q4.append(solveStrategy3(X_solveable[k]))

Q1 = np.array(Q1)
Q2 = np.array(Q2)
Q3 = np.array(Q3)
Q4 = np.array(Q4)

J = np.argsort(np.minimum(Q1[:,0],Q2[:,0]))
Q1=Q1[J]
Q2=Q2[J]
Q3=Q3[J]
Q4=Q4[J]
X_solveable=X_solveable[J]
print J

plt.plot(Q1[:,0],"-k")
plt.plot(Q2[:,0],"-b")
#plt.plot(Q3[:,0],"-r")
#plt.plot(Q4[:,0],"-m")

K = np.argsort(-Q4[:,0])[1:10]

print Q4[K]
print Q1[K]
print X_solveable[K]


ax = plt.gca()
#ax.set_ylim([0,12])
plt.show()
