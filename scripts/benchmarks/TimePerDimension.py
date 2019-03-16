import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MaxNLocator
import scipy.special as sy
from scipy import interpolate

import numpy as np
import os
import sys
from ParseBenchmarkFile import *

fname = '../../data/benchmarks/10D_spatial_snake_twister_stratification_2019_03_15_17:30:20.xml'
fname = '../../data/benchmarks/07D_kuka_windshield_stratifications_2019_03_15_17:30:53.xml'
fname = '../../data/benchmarks/11D_planar_snake_twister_different_hierarchies_2019_03_15_17:32:53.xml'

fname = '../../data/benchmarks/10D_spatial_snake_twister_stratification_2019_03_15_17:35:08.xml'
fname = '../../data/benchmarks/07D_kuka_windshield_stratifications_2019_03_15_19:20:51.xml'
fname = '../../data/benchmarks/11D_planar_snake_twister_different_hierarchies_2019_03_16_00:34:42.xml'
fname = '../../data/benchmarks/last.xml'

benchmark = BenchmarkAnalytica(fname)

START_AT_BEGINNING = True
EPSILON_IGNORE = 1e-1
EXTRAPOLATED_PTS = 1
pp = PdfPages(benchmark.fname_pdf)

times_vanilla = benchmark.TimeAveragePerSingleDimensionalityAlgorithm()
times_mean = benchmark.TimeAveragePerDimensionality()
times_min = benchmark.TimeMinPerDimensionality()
times_max = benchmark.TimeMaxPerDimensionality()

fig = plt.figure(0)
ax = fig.add_subplot(111)

fig.patch.set_facecolor('white')
plt.title('Time per DoF (Runs: %d, Maxtime: %.0fs)'%(benchmark.runcount,benchmark.timelimit))
#plt.title(r'$\mathrm{Histogram\ of\ Heuristic\ Runtime\ with\ %d\ DoFs:}\ \mu=%.3fs,\ \sigma=%.3fs$' %(number_of_strata, mu, sigma))
# axcolor='#ffffff'
# ax = fig.gca()
ax.set_xlabel('Degrees of Freedom')
ax.set_ylabel('Time (s)')

##find StartD
startD = 0

if not START_AT_BEGINNING:
  while abs(times_max[startD]-times_min[startD])<EPSILON_IGNORE:
    startD=startD+1

  print "StartD: ",startD

times_vanilla = times_vanilla[startD:]
times_mean = times_mean[startD:]
times_min = times_min[startD:]
times_max = times_max[startD:]
I = np.arange(1+startD,1+len(times_vanilla)+startD)

T = np.arange(1+startD, 1+startD+len(times_vanilla)+EXTRAPOLATED_PTS)

interp_type = 'quadratic'
f_vanilla_extrapolate = interpolate.interp1d(I, times_vanilla.flatten(), fill_value="extrapolate", kind=interp_type)
f_mean_extrapolate = interpolate.interp1d(I, times_mean.flatten(), fill_value="extrapolate", kind=interp_type)
f_min_extrapolate = interpolate.interp1d(I, times_min.flatten(), fill_value="extrapolate", kind=interp_type)
f_max_extrapolate = interpolate.interp1d(I, times_max.flatten(), fill_value="extrapolate", kind=interp_type)

times_vanilla_extrapolate = f_vanilla_extrapolate(T)
times_mean_extrapolate = f_mean_extrapolate(T)
times_min_extrapolate = f_min_extrapolate(T)
times_max_extrapolate = f_max_extrapolate(T)

ms = 10
plt.plot(I,times_vanilla,'x-',color='k', markersize=2*ms, label="RRT")
plt.plot(I,times_max,'o-',color='k', markersize=ms, label="QRRT (Max Time Heuristic)")
plt.plot(I,times_mean,'v-',color='k', markersize=ms, label="QRRT (Mean Time Heuristics)")
plt.plot(I,times_min,'s-',color='k', markersize=ms, label="QRRT (Min Time Heuristic)")

#bestNames = vnames[p_min]
#ax.annotate(bestNames[-1], xy=(I[-1], -1))

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
