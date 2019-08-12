import matplotlib.pyplot as plt
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MaxNLocator
import scipy.special as sy
from scipy import interpolate

import numpy as np
import os
import sys
from ParseBenchmarkFile import *

def PlotTimeVsDimension(fname, show=False, START_AT_BEGINNING=False, EXTRAPOLATED_PTS=1, REMOVE_PTS_START = 0, REMOVE_PTS_END = 0):
  benchmark = BenchmarkAnalytica(fname)

  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + "_time_vs_dimension.pdf"
  EPSILON_IGNORE = 1e-1
  pp = PdfPages(fname_pdf)

  times_vanilla = benchmark.TimeAveragePerSingleDimensionalityAlgorithm()

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
  startD = 4

  DIMS = benchmark.dimensions_per_level

  times_vanilla = times_vanilla[startD:]
  I = np.array(DIMS[startD:]).flatten()
  T = np.append(I,DIMS[-1]+EXTRAPOLATED_PTS)
  print I
  print times_vanilla.flatten()

  interp_type = 'quadratic'
  if len(I)>1:
    if len(I)==2:
      interp_type = 'linear'
    f_vanilla_extrapolate = interpolate.interp1d(I, times_vanilla.flatten(), fill_value="extrapolate", kind=interp_type)
    times_vanilla_extrapolate = f_vanilla_extrapolate(T)
  else:
    EXTRAPOLATED_PTS = 0

  ms = 10
  rrt_label = "RRT"
  plt.plot(I,times_vanilla,'o-',color='k', markersize=ms, label=rrt_label)

  #bestNames = vnames[p_min]
  #ax.annotate(bestNames[-1], xy=(I[-1], -1))

  #### EXTRAPOLATED FUNCTIONS
  if EXTRAPOLATED_PTS>0:
    plt.plot(T,times_vanilla_extrapolate,'--',color='k')


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
  fname = '../../data/benchmarks/15D_planar_manipulator_different_dimensions_2019_03_21_14:23:46.xml'
  fname = '../../data/benchmarks/15D_planar_manipulator_different_dimensions_2019_03_21_14:23:46.xml'
  PlotTimeVsDimension(fname, show=True, START_AT_BEGINNING=False)

