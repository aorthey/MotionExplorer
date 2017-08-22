import pickle as pk
from geometry_msgs.msg import Quaternion
from tf.transformations import *
import sys
import numpy as np

fname = '../data/misc/rrt-wall-0-time-3-62.tau'

fh = open(fname,"rb")                                                                                                             
tau = []
while 1:
  try:
    q = pk.load(fh) 
    tau.append(q)
  except EOFError:
    break
fh.close()
tau = tau[0]

path = []
for q in tau:
  qreal = np.zeros(66)
  qreal[0] = q[0]
  qreal[1] = q[1]
  qreal[2] = q[2]

  euler = euler_from_quaternion(q[3:7])
  #roll = euler[0]
  #pitch = euler[1]
  #yaw = euler[2]
  qreal[3] = euler[0]
  qreal[4] = euler[1]
  qreal[5] = euler[2]

  ictr = np.array(range(7,9)+range(11,13)+range(22,34)+range(36,48)+range(50,56)+range(58,64))
  qreal[ictr] = q[7:]
  path.append(qreal)

def config_str(q,tag):
  s = '    <'+tag+'>'
  s+= str(len(q))+'&#x09;'
  for qi in q:
    s+= str(qi)+' '
  s+= '</'+tag+'>\n'
  return s

fname = '../data/gui/rrt_wall.xml'
fh = open(fname,'w')
fh.write('<?xml version="1.0" ?>\n')
fh.write('<GUI>\n')
fh.write('  <robot>\n')
fh.write('    <name></name>\n')
fh.write(config_str(path[0],'configinit'))
fh.write(config_str(path[1],'configgoal'))
fh.write('  </robot>\n')
    #<sweptvolume>
        #<qitem>33&#x09;-3.3 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 </qitem>
fh.write('  <sweptvolume>\n')
for q in path:
  fh.write(config_str(q,'qitem'))
fh.write('  </sweptvolume>\n')
fh.write('</GUI>\n')
print 'wrote to file:',fname


#qlimit [0] -inf - inf
#qlimit [1] -inf - inf
#qlimit [2] -inf - inf
#qlimit [3] -inf - inf
#qlimit [4] -inf - inf
#qlimit [5] -inf - inf
#qlimit [6] 0 - 0
#qlimit [7] -0.785398 - 0.785398
#qlimit [8] -0.0872665 - 1.0472
#qlimit [9] 0 - 0
#qlimit [10] 0 - 0
#qlimit [11] -0.785398 - 0.785398
#qlimit [12] -0.523599 - 0.785398
#qlimit [13] 0 - 0
#qlimit [14] 0 - 0
#qlimit [15] 0 - 0
#qlimit [16] 0 - 0
#qlimit [17] 0 - 0
#qlimit [18] 0 - 0
#qlimit [19] 0 - 0
#qlimit [20] 0 - 0
#qlimit [21] 0 - 0
#qlimit [22] -3.14159 - 1.0472
#qlimit [23] -0.174533 - 1.65806
#qlimit [24] -1.6057 - 1.6057
#qlimit [25] -2.3911 - 0.0349066
#qlimit [26] -1.6057 - 1.6057
#qlimit [27] -1.6057 - 1.6057
#qlimit [28] -0.785398 - 0.785398
#qlimit [29] -0.785398 - 0.785398
#qlimit [30] -0.785398 - 0.785398
#qlimit [31] -0.785398 - 0.785398
#qlimit [32] -0.785398 - 0.785398
#qlimit [33] -0.785398 - 0.785398
#qlimit [34] 0 - 0
#qlimit [35] 0 - 0
#qlimit [36] -3.14159 - 1.0472
#qlimit [37] -1.65806 - 0.174533
#qlimit [38] -1.6057 - 1.6057
#qlimit [39] -2.3911 - 0.0349066
#qlimit [40] -1.6057 - 1.6057
#qlimit [41] -1.6057 - 1.6057
#qlimit [42] 0 - 0.785398
#qlimit [43] -0.785398 - 0.785398
#qlimit [44] -0.785398 - 0.785398
#qlimit [45] -0.785398 - 0.785398
#qlimit [46] -0.785398 - 0.785398
#qlimit [47] -0.785398 - 0.785398
#qlimit [48] 0 - 0
#qlimit [49] 0 - 0
#qlimit [50] -0.523599 - 0.785398
#qlimit [51] -0.349066 - 0.610865
#qlimit [52] -2.18166 - 0.733038
#qlimit [53] -0.0349066 - 2.61799
#qlimit [54] -1.309 - 0.733038
#qlimit [55] -0.610865 - 0.349066
#qlimit [56] 0 - 0
#qlimit [57] 0 - 0
#qlimit [58] -0.785398 - 0.523599
#qlimit [59] -0.610865 - 0.349066
#qlimit [60] -2.18166 - 0.733038
#qlimit [61] -0.0349066 - 2.61799
#qlimit [62] -1.309 - 0.733038
#qlimit [63] -0.349066 - 0.610865
#qlimit [64] 0 - 0
#qlimit [65] 0 - 0

