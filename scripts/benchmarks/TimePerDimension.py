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

def TimePerDimension(fname, show=False, START_AT_BEGINNING=False, EXTRAPOLATED_PTS=1):
  benchmark = BenchmarkAnalytica(fname)
  EPSILON_IGNORE = 1e-1
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

  DIMS = benchmark.dimensions_per_level

  while len(DIMS) > len(times_min):
    DIMS = DIMS[1:]

  times_vanilla = times_vanilla[startD:]
  times_mean = times_mean[startD:]
  times_min = times_min[startD:]
  times_max = times_max[startD:]


  I = np.array(DIMS[startD:]).flatten()
  T = np.append(I,DIMS[-1]+EXTRAPOLATED_PTS)
  print I
  print times_vanilla.flatten()

  interp_type = 'quadratic'
  if len(I)>1:
    if len(I)==2:
      interp_type = 'linear'
    f_vanilla_extrapolate = interpolate.interp1d(I, times_vanilla.flatten(), fill_value="extrapolate", kind=interp_type)
    f_mean_extrapolate = interpolate.interp1d(I, times_mean.flatten(), fill_value="extrapolate", kind=interp_type)
    f_min_extrapolate = interpolate.interp1d(I, times_min.flatten(), fill_value="extrapolate", kind=interp_type)
    f_max_extrapolate = interpolate.interp1d(I, times_max.flatten(), fill_value="extrapolate", kind=interp_type)
    times_vanilla_extrapolate = f_vanilla_extrapolate(T)
    times_mean_extrapolate = f_mean_extrapolate(T)
    times_min_extrapolate = f_min_extrapolate(T)
    times_max_extrapolate = f_max_extrapolate(T)
  else:
    EXTRAPOLATED_PTS = 0


  ms = 10
  rrt_label = "QRRT_("+str(DIMS[-1])+") Trivial Stratification"
  plt.plot(I,times_vanilla,'x-',color='k', markersize=2*ms, label=rrt_label)
  plt.plot(I,times_max,'o-',color='k', markersize=ms, label="QRRT (Max Time Heuristic)")
  plt.plot(I,times_mean,'v-',color='k', markersize=ms, label="QRRT (Mean Time Heuristics)")
  plt.plot(I,times_min,'s-',color='k', markersize=ms, label="QRRT (Min Time Heuristic)")

  #bestNames = vnames[p_min]
  #ax.annotate(bestNames[-1], xy=(I[-1], -1))

  #### EXTRAPOLATED FUNCTIONS
  if EXTRAPOLATED_PTS>0:
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
  if show:
    plt.show()
  plt.close()
  del benchmark

if __name__ == '__main__':
  fname = '../../data/benchmarks/last.xml'
  TimePerDimension(fname, show=True, START_AT_BEGINNING=True)

