import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MaxNLocator
import scipy.special as sy
from scipy import interpolate

import numpy as np
from xml.dom.minidom import parse
import re
import os
import sys

fname = '../../data/benchmarks/last.xml'
#fname = '../../data/benchmarks/planar_manip_255_decompositions.xml'
# fname = '../../data/benchmarks/15D_planar_manipulator_different_dimensions_2019_03_08_18:34:35.xml'
START_AT_BEGINNING = False
EXTRAPOLATED_PTS = 1
###################################################################################
#GET DATA
###################################################################################
doc = parse(fname)
name = doc.getElementsByTagName("name")[0].firstChild.data
fname_dir = os.path.dirname(fname)
fname_pdf = fname_dir + "/" + name + "_runtimes.pdf"
pp = PdfPages(fname_pdf)

planners = doc.getElementsByTagName("planner")
nr_planners = int(doc.getElementsByTagName("number_of_planners")[0].firstChild.data)
number_of_strata = int(doc.getElementsByTagName("max_levels")[0].firstChild.data)
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

##replace 0 with NaN
times_per_strata = np.where(times_per_strata!=0,times_per_strata,np.nan)

times_vanilla = np.array(times_vanilla)
##remove rows with all NaNs
times_vanilla = times_vanilla[~np.isnan(times_per_strata).all(axis=1)]
times_per_strata = times_per_strata[~np.isnan(times_per_strata).all(axis=1)]
print times_per_strata

###################################################################################
#Display DATA
###################################################################################
times_mean = np.nanmean(times_per_strata,axis=1)
times_min = np.nanmin(times_per_strata,axis=1)
times_max = np.nanmax(times_per_strata,axis=1)
p_min = np.nanargmin(times_per_strata,axis=1)
p_max = np.nanargmax(times_per_strata,axis=1)
vnames = np.array(vnames)

print "Best:",vnames[p_min]
print zip(vnames[p_min],times_per_planner[p_min])
print "Worst:",vnames[p_max]
print zip(vnames[p_max],times_per_planner[p_max])

fig = plt.figure(0)
ax = fig.add_subplot(111)

fig.patch.set_facecolor('white')
plt.title('Time per DoF (Runs: %d, Maxtime: %.0fs)'%(runcount,timelimit))
#plt.title(r'$\mathrm{Histogram\ of\ Heuristic\ Runtime\ with\ %d\ DoFs:}\ \mu=%.3fs,\ \sigma=%.3fs$' %(number_of_strata, mu, sigma))
# axcolor='#ffffff'
# ax = fig.gca()
ax.set_xlabel('Degrees of Freedom')
ax.set_ylabel('Time (s)')

##find StartD
startD = 0
while abs(times_max[startD]-times_min[startD])<1e-1:
  startD=startD+1

print "StartD: ",startD

if START_AT_BEGINNING:
  startD = 0

I = np.arange(startD,len(times_per_strata))
times_vanilla = times_vanilla[startD:]
times_max = times_max[startD:]
times_min = times_min[startD:]
times_mean = times_mean[startD:]

T = np.arange(startD, len(times_per_strata)+EXTRAPOLATED_PTS)

f_vanilla_extrapolate = interpolate.interp1d(I, times_vanilla.flatten(), fill_value="extrapolate")
f_mean_extrapolate = interpolate.interp1d(I, times_mean.flatten(), fill_value="extrapolate")
f_min_extrapolate = interpolate.interp1d(I, times_min.flatten(), fill_value="extrapolate")
f_max_extrapolate = interpolate.interp1d(I, times_max.flatten(), fill_value="extrapolate")
times_vanilla_extrapolate = f_vanilla_extrapolate(T)
times_mean_extrapolate = f_mean_extrapolate(T)
times_min_extrapolate = f_min_extrapolate(T)
times_max_extrapolate = f_max_extrapolate(T)

ms = 10
plt.plot(I,times_vanilla,'x-',color='k', markersize=2*ms, label="RRT")
plt.plot(I,times_max,'o-',color='k', markersize=ms, label="QRRT (Max Time Heuristic)")
plt.plot(I,times_mean,'v-',color='k', markersize=ms, label="QRRT (Mean Time Heuristics)")
plt.plot(I,times_min,'s-',color='k', markersize=ms, label="QRRT (Min Time Heuristic)")

bestNames = vnames[p_min]
ax.annotate(bestNames[-1], xy=(I[-1], -1))

#### EXTRAPOLATED FUNCTIONS
plt.plot(T,times_vanilla_extrapolate,'--',color='k')
plt.plot(T,times_min_extrapolate,'--',color='k')
plt.plot(T,times_max_extrapolate,'--',color='k')
plt.plot(T,times_mean_extrapolate,'--',color='k')
ax.fill_between(T, times_min_extrapolate, times_max_extrapolate, facecolor='0.9')

#### LEGEND
legend = ax.legend(loc='upper left', shadow=True)
for label in legend.get_texts():
    label.set_fontsize('large')
for label in legend.get_lines():
    label.set_linewidth(1.5)

#Force Integer Labels
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

pp.savefig(plt.gcf(),pad_inches = 0)
pp.close()
plt.show()
plt.close()
