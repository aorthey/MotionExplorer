import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
import scipy.special as sy
from scipy import interpolate

import numpy as np
from xml.dom.minidom import parse
import re
import os
import sys

fname = '../../data/benchmarks/last.xml'
fname_base, fname_ext = os.path.splitext(fname)
fname_pdf = fname_base + ".pdf"
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

  p_ctr=p_ctr+1

times_mean = np.mean(times_per_strata,axis=1)
times_min = np.min(times_per_strata,axis=1)
times_max = np.max(times_per_strata,axis=1)
times_vanilla = np.array(times_vanilla)

print times_vanilla
print times_max

fig = plt.figure(0)
ax = fig.add_subplot(111)

fig.patch.set_facecolor('white')
plt.title('Time per DoF')
# axcolor='#ffffff'
# ax = fig.gca()
ax.set_xlabel('Degrees of Freedom')
ax.set_ylabel('Time (s)')

startD = 4
I = np.arange(startD,len(times_per_strata)+1)
times_vanilla = times_vanilla[startD-1:]
times_max = times_max[startD-1:]
times_min = times_min[startD-1:]
times_mean = times_mean[startD-1:]

T = np.arange(startD, len(times_per_strata)+5)
f_vanilla_extrapolate = interpolate.interp1d(I, times_vanilla.flatten(), fill_value="extrapolate")
f_mean_extrapolate = interpolate.interp1d(I, times_mean.flatten(), fill_value="extrapolate")
f_min_extrapolate = interpolate.interp1d(I, times_min.flatten(), fill_value="extrapolate")
f_max_extrapolate = interpolate.interp1d(I, times_max.flatten(), fill_value="extrapolate")
times_vanilla_extrapolate = f_vanilla_extrapolate(T)
times_mean_extrapolate = f_mean_extrapolate(T)
times_min_extrapolate = f_min_extrapolate(T)
times_max_extrapolate = f_max_extrapolate(T)


plt.plot(I,times_vanilla,'o-',color='k')
plt.plot(I,times_min,'o-',color='k')
plt.plot(I,times_max,'o-',color='k')
plt.plot(I,times_mean,'o-',color='k')
ax.fill_between(I, times_min, times_max, facecolor='0.8')

plt.plot(T,times_vanilla_extrapolate,'--',color='k')
plt.plot(T,times_min_extrapolate,'--',color='k')
plt.plot(T,times_max_extrapolate,'--',color='k')
plt.plot(T,times_mean_extrapolate,'--',color='k')

pp.savefig(plt.gcf())
pp.close()
plt.show()
plt.close()
