import sys
import numpy as np
from xml.dom.minidom import parse

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import matplotlib.cm as cm
import matplotlib.colors as colors
from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)
rc('font', family='serif', size=30)

fname = '../data/experiments/WAFR2020/06D_bhattacharya_square.xml'

doc = parse(fname)
hierarchy = doc.getElementsByTagName("hierarchy")[0]
levels = hierarchy.getElementsByTagName("level")
for level in levels:
  robots = level.getElementsByTagName("robot")
  for robot in robots:
    rid = int(robot.getAttribute("id"))
    t = str(robot.getAttribute("type"))
    sid = robot.getAttribute("simplification_of_id")
    if sid:
      sid = int(sid)
    print(rid, t, sid)
  print("------")


# planners = doc.getElementsByTagName("planner")
# nr_planners = int(doc.getElementsByTagName("number_of_planners")[0].firstChild.data)
# runcount = int(doc.getElementsByTagName("run_count")[0].firstChild.data)
# timelimit = float(doc.getElementsByTagName("max_time")[0].firstChild.data)
# print "planners:",nr_planners," runs:",runcount


# fig = plt.figure(0)
# fig.patch.set_facecolor('white')
# ax = fig.gca()



# ax.set_xlabel(r'\theta_1',fontsize=font_size)
# ax.set_ylabel(r'\theta_2',rotation=1.57,fontsize=font_size)
# ax.tick_params(axis='both', which='major', pad=10, labelsize=0.8*font_size)
# lim=3.14
# plt.axis([-lim,lim,-lim,lim])
# ax.annotate(r'x_1', x1loc)
# ax.annotate(r'x_2', x2loc)
# plt.plot(p1[0],p1[1],'o',color='black',markersize=10)
# plt.plot(p3[0],p3[1],'o',color='black',markersize=10)
# ax.xaxis.set_major_locator(MaxNLocator(integer=True))
# ax.yaxis.set_major_locator(MaxNLocator(integer=True))
# plt.savefig("2dof_cspace_1.png", bbox_inches='tight')

