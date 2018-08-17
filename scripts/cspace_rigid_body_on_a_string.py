import sys
import numpy as np
from cspace_visualizer import *
from cspace_rigid_body_on_a_string_generate_data import *
from cspace_system_2dof import *
import cspace_system_2dof as dof

grey = '0.6'
font_size = 45
lim=5.0
showSamples = True
maxSamples = 200
maxInfeasibleElements = 1000
output_name = "rigid_body_on_a_string"

qsname = output_name+"_qspace.png"
csname = output_name+"_cspace.png"
qsname_samples = output_name+"_qspace_samples.png"
csname_samples = output_name+"_cspace_samples.png"

#qfname = "../build/qs_dense.roadmap"
#cfname = "../build/cs_dense.roadmap"
qfname = "../build/roadmap_level_0_robot_1.roadmap"
cfname = "../build/roadmap_level_0_robot_0.roadmap"
generateQuotientSpaceDense(qfname, 1000)
generateCSpaceDense(cfname, 60000)

def plotQSPoints():
    
  Q = getPoints(qfname, maxSamples)
  for q in Q:
    x = float(q[0])
    y = float(q[1])
    feasible = q[3]
    sufficient = q[4]
    if not feasible:
      plt.axvline(x, color='k', linewidth=1)
    elif sufficient:
      plt.axvline(x, color=grey, linewidth=1)
    else:
      delta=0.05
      plt.plot([x,x],[-delta,+delta],'-',color=grey, linewidth=1)

def plotCSPoints():
  Q = getPoints(cfname, maxSamples)
  for q in Q:
    x = float(q[0])
    t = float(q[2])
    feasible = q[3]
    if feasible:
      plt.plot(x,t,'o',color=grey, linewidth=1)
    else:
      plt.plot(x,t,'x',color='k', linewidth=1)

def plotQuotientSpaceBackground():
  P1 = np.load(open(r'tmp_rigid_body_on_a_string_QS_dense_1.npy', 'rb'))
  P2 = np.load(open(r'tmp_rigid_body_on_a_string_QS_dense_2.npy', 'rb'))
  plotCSpaceDelaunayGrey(P1,P2,0.15)

def plotCSpaceBackground():
  P1 = np.load(open(r'tmp_rigid_body_on_a_string_CS_dense_1.npy', 'rb'))
  P2 = np.load(open(r'tmp_rigid_body_on_a_string_CS_dense_2.npy', 'rb'))
  plotCSpaceDelaunayGrey(P1,P2,0.15)


###############################################################################
## QUOTIENT-SPACE
###############################################################################
fig = plt.figure(0)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'x',fontsize=font_size)
ax.set_ylabel(r'\theta',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=15)
plt.axis([-lim,lim,-3.14,3.14])
plotQuotientSpaceBackground()
plt.savefig(qsname, bbox_inches='tight')
if showSamples:
  plotQSPoints()
  plt.savefig(qsname_samples, bbox_inches='tight')

###############################################################################
## CSPACE
###############################################################################
fig = plt.figure(1)
fig.patch.set_facecolor('white')
ax = fig.gca()
ax.set_xlabel(r'x',fontsize=font_size)
ax.set_ylabel(r'\theta',rotation=1.57,fontsize=font_size)
ax.tick_params(axis='both', which='major', pad=15)
plt.axis([-lim,lim,-3.14,3.14])
plotCSpaceBackground()
plt.savefig(csname, bbox_inches='tight')
if showSamples:
  plotQSPoints()
  plt.savefig(csname_samples, bbox_inches='tight')
plt.show()
