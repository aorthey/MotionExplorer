import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MaxNLocator
from scipy.stats import norm
import matplotlib.mlab as mlab
import scipy.special as sy
from scipy import interpolate

import numpy as np
from xml.dom.minidom import parse
import re
import os
import sys

fname = '../../data/benchmarks/last.xml'
fname_base, fname_ext = os.path.splitext(fname)
fname_pdf = fname_base + "_last_strata_histogram.pdf"
pp = PdfPages(fname_pdf)

number_of_strata = 8
###################################################################################
#GET DATA
###################################################################################
doc = parse(fname)
name = doc.getElementsByTagName("name")[0]

planners = doc.getElementsByTagName("planner")
nr_planners = int(doc.getElementsByTagName("number_of_planners")[0].firstChild.data)
runcount = int(doc.getElementsByTagName("run_count")[0].firstChild.data)
timelimit = float(doc.getElementsByTagName("max_time")[0].firstChild.data)
print "planners:",nr_planners," runs:",runcount

vnames = []
times_per_strata = np.zeros((number_of_strata,nr_planners))
times_per_planner = np.zeros((nr_planners))
#record runtimes of vanilla RRT
times_vanilla = np.zeros((number_of_strata,1))

p_ctr = 0
for planner in planners:
  name = planner.getElementsByTagName("name")[0].firstChild.data
  print "name:", name
  p = re.compile('(.+)\((\d+)\)')
  m = p.match(name)

  if m:
    vnames.append(name)
    Ln = np.array(list(m.group(2)), dtype=int)
    Nstrata = Ln[-1]-1

    runs = planner.getElementsByTagName("run")
    c_ctr = 0
    for run in runs:
      sid = run.getAttribute("number")
      nodes = run.getElementsByTagName("nodes")[0].firstChild.data
      time = float(run.getElementsByTagName("time")[0].firstChild.data)
      times_per_strata[Nstrata,p_ctr] = times_per_strata[Nstrata,p_ctr] + float(time)
      c_ctr=c_ctr+1

    if c_ctr <= 0:
      times_per_strata[Nstrata,p_ctr]=-1
    else:
      times_per_strata[Nstrata,p_ctr]=times_per_strata[Nstrata,p_ctr]/float(c_ctr)
    if len(Ln)<=1:
      print Nstrata,":",times_per_strata[Nstrata,p_ctr]
      times_vanilla[Nstrata]=times_per_strata[Nstrata, p_ctr]
    times_per_planner[p_ctr] = times_per_strata[Nstrata,p_ctr]

  p_ctr=p_ctr+1

times_per_strata = np.where(times_per_strata!=0,times_per_strata,np.nan)
times_per_strata = times_per_strata[-1,:]
p_min = np.nanargmin(times_per_strata)
p_max = np.nanargmax(times_per_strata)
times_per_strata = times_per_strata[~np.isnan(times_per_strata)]


###################################################################################
#Display DATA
###################################################################################
vnames = np.array(vnames)
t_min = float(np.nanmin(times_per_strata))
t_max = float(np.nanmax(times_per_strata))
T = np.linspace(t_min,t_max,len(times_per_strata))
times_per_strata = np.sort(times_per_strata)

fig = plt.figure(0)
ax = fig.add_subplot(111)
print vnames[p_min]
print times_per_strata


nr_bins = 50
n, bins, patches = plt.hist(times_per_strata, nr_bins, normed=1, facecolor='0.8')

(mu, sigma) = norm.fit(times_per_strata)
y = mlab.normpdf( bins, mu, sigma)
l = plt.plot(bins, y, 'k-', linewidth=1)

ymin,ymax = ax.get_ylim()

len_bin = ((t_max-t_min)/nr_bins)
vx = t_max - 0.5*len_bin
vy = 2.0/len(times_per_strata)
tx = 0.9*(t_max-t_min)
ty = 0.5*(ymax-ymin)

ax.annotate(vnames[p_max], xytext=(tx,ty),
    arrowprops=dict(arrowstyle="->"), xy=(vx, vy))
ax.set_ylabel('Percentage of Heuristics')
ax.set_xlabel('Time (s)')
plt.title(r'$\mathrm{Histogram\ of\ Heuristic\ Runtime\ with\ %d\ DoFs:}\ \mu=%.3fs,\ \sigma=%.3fs$' %(number_of_strata, mu, sigma))

pp.savefig(plt.gcf())
pp.close()
plt.show()
plt.close()
