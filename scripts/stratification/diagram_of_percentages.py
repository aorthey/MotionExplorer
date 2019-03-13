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
fname_pdf = fname_base + ".pdf"
pp = PdfPages(fname_pdf)


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
number_of_strata = int(doc.getElementsByTagName("max_levels")[0].firstChild.data)
print "planners:",nr_planners," runs:",runcount

vtimes = np.zeros((runcount,nr_planners))
vnodes = np.zeros((runcount,nr_planners))
vsuccess = np.zeros((runcount,nr_planners))
vnames = []

percentages=[]
vtimes = np.zeros((runcount,nr_planners))
p_ctr = 0
for planner in planners:
  name = planner.getElementsByTagName("name")[0].firstChild.data
  print "name:", name
  p = re.compile('(.+)\((\d+)\)')
  m = p.match(name)

  idxs = 4
  ##A contains for each strata its levels, the number of nodes per level, and
  ## the number of feasible nodes
  A = np.zeros((number_of_strata,idxs))
  c_ctr = 0
  if m:
    vnames.append(name)
    Ln = np.array(list(m.group(2)), dtype=int)
    for l in Ln:
      A[l-1,:] = l

    runs = planner.getElementsByTagName("run")
    Aruns = np.full((number_of_strata,idxs),0,dtype=object)
    for run in runs:
      sid = run.getAttribute("number")
      nodes = run.getElementsByTagName("nodes")[0].firstChild.data
      time = run.getElementsByTagName("time")[0].firstChild.data
      if run.getElementsByTagName("levels"):
        levels = int(run.getElementsByTagName("levels")[0].firstChild.data)
        snodes = run.getElementsByTagName("sampled_nodes_per_level")[0]
        all_nodes = snodes.getElementsByTagName("nodes")
        all_feasible_nodes = snodes.getElementsByTagName("feasible_nodes")
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

        Aruns[:,1:3] = Aruns[:,1:3] + A[:,1:3]
        vtimes[c_ctr,p_ctr] = time
      c_ctr=c_ctr+1

    Aruns = Aruns/len(runs)
    Aruns[:,0] = A[:,0]
    for k in range(0,len(Aruns)):
      if Aruns[k,0] > 0:
        Aruns[k,3] = float(Aruns[k,2])/float(Aruns[k,1])

    percentages.append(float(np.sum(Aruns[:,2]))/float(np.sum(Aruns[:,1])))

  p_ctr=p_ctr+1

vtimes = vtimes[:,~np.all(vtimes==0,axis=0)]
times = np.mean(vtimes,axis=0)
percentages = np.array(percentages)
yL = np.max(percentages)

fig = plt.figure(0)
fig.patch.set_facecolor('white')
plt.title('Feasibility vs. Time per Algorithm')

ax1color='#ffffff'
ax2color='tab:red'

ax1 = fig.gca()
ax1.set_ylabel('Percentage of Feasible Samples')
ax1.set_xlabel('Algorithm')
ax1.axhline(yL,color='k',linestyle='--',alpha=0.2)
ax1.plot(percentages,'-ok')

ax1.set_ylim([0,0.5])
ax1.xaxis.set_ticks(np.arange(0, len(percentages)))
plannerLabelRotation=80
xtickNames = plt.setp(ax1,xticklabels=vnames)
plt.setp(xtickNames, rotation=plannerLabelRotation)

ax2 = ax1.twinx()
ax2.set_ylabel('Time (s)', color=ax2color)
ax2.tick_params(axis='y', labelcolor=ax2color)
ax2.plot(times,'-o',color=ax2color)


plt.tight_layout()
pp.savefig(plt.gcf())
pp.close()
plt.show()
plt.close()
