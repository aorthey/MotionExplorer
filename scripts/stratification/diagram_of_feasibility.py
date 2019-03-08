import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
import scipy.special as sy

import numpy as np
from xml.dom.minidom import parse
import re
import os
import sys

fname = '../../data/benchmarks/last.xml'
fname_base, fname_ext = os.path.splitext(fname)

number_of_strata = 5

###################################################################################
#GET DATA
###################################################################################
doc = parse(fname)
name = doc.getElementsByTagName("name")[0]
print name

planners = doc.getElementsByTagName("planner")
nr_planners = int(doc.getElementsByTagName("number_of_planners")[0].firstChild.data)
runcount = int(doc.getElementsByTagName("run_count")[0].firstChild.data)
timelimit = float(doc.getElementsByTagName("max_time")[0].firstChild.data)
print "planners:",nr_planners," runs:",runcount

vtimes = np.zeros((runcount,nr_planners))
vnodes = np.zeros((runcount,nr_planners))
vsuccess = np.zeros((runcount,nr_planners))
vnames = []

for planner in planners:
  name = planner.getElementsByTagName("name")[0].firstChild.data
  print "name:", name
  p = re.compile('(.+)\((\d+)\)')
  m = p.match(name)
  idxs = 4
  A = np.zeros((number_of_strata,idxs))
  if m:
    Ln = np.array(list(m.group(2)), dtype=int)
    for l in Ln:
      A[l-1,:] = l

    fname_pdf = fname_base +m.group(2)+ ".pdf"
    pp = PdfPages(fname_pdf)

    runs = planner.getElementsByTagName("run")
    c_ctr = 0
    Aruns = np.full((number_of_strata,idxs),0,dtype=object)
    for run in runs:
      sid = run.getAttribute("number")
      nodes = run.getElementsByTagName("nodes")[0].firstChild.data
      if run.getElementsByTagName("levels"):
        levels = int(run.getElementsByTagName("levels")[0].firstChild.data)
        snodes = run.getElementsByTagName("sampled_nodes_per_level")[0]
        all_nodes = snodes.getElementsByTagName("nodes")
        all_feasible_nodes = snodes.getElementsByTagName("feasible_nodes")
        print levels
        ctr = 0
        Aidx = np.argwhere(A[:,0]>0)
        for anode in all_nodes:
          nrNodes = anode.firstChild.data
          A[Aidx[ctr],1] = int(nrNodes)
          ctr = ctr+1
        ctr = 0
        for fnode in all_feasible_nodes:
          nrNodes = fnode.firstChild.data
          A[Aidx[ctr],2] = int(nrNodes)
          ctr = ctr+1
        print A[:,1:3]

        Aruns[:,1:3] = Aruns[:,1:3] + A[:,1:3]
    Aruns = Aruns/len(runs)
    Aruns[:,0] = A[:,0]
    for k in range(0,len(Aruns)):
      if Aruns[k,0] > 0:
        Aruns[k,3] = float(Aruns[k,2])/float(Aruns[k,1])
        print float(Aruns[k,2])/float(Aruns[k,1])
    print Aruns

    if len(Aruns[:,0]>0)>0:
      fig = plt.figure(0)
      ax = fig.gca()
      fig.patch.set_facecolor('white')
      ax.set_xlabel('Percentage of Feasible Samples')
      ax.set_ylabel('Dimension')
      ax.set_xlim([0,1])
      ax.set_ylim([0,number_of_strata])
      plt.tight_layout()
      plt.title('Feasibility of Stratification')
      currentAxis = plt.gca()
      for k in range(1,number_of_strata):
        ax.axhline(k,color='k',linestyle='--',alpha=0.2)

      lastLvl = 0
      X = 1
      for k in range(0,len(Aruns)):
          A = Aruns[k,:]
          if A[0] > 0:
            X = A[3]*X
            print X
            currentAxis.add_patch(Rectangle((1-X, lastLvl), X, A[0]-lastLvl,
                                  alpha=1, edgecolor="black", facecolor='w',hatch=r"//"))
            lastLvl=A[0]
      pp.savefig(plt.gcf())
      plt.close()
      pp.close()
      # plt.show()

