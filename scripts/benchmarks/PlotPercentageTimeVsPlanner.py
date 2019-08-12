import matplotlib.pyplot as plt
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
import scipy.special as sy

from ParseBenchmarkFile import *
import numpy as np
import os
import sys

def PlotPercentageTimeVsPlanner(fname = '../../data/benchmarks/last.xml', LAST_LEVEL=True):
  benchmark = BenchmarkAnalytica(fname)
  benchmark.ClipNonLargestDimensionPlanners()

  fname_base, fname_ext = os.path.splitext(fname)
  if LAST_LEVEL:
    percentages = benchmark.GetPercentageOfFeasibleNodesPerPlannerAll()
    fname_pdf = fname_base + "_percentagetime_vs_planner_last_level.pdf"
  else:
    fname_pdf = fname_base + "_percentagetime_vs_planner.pdf"
    percentages = benchmark.GetPercentageOfFeasibleNodesPerPlanner()

  pp = PdfPages(fname_pdf) 

  times = benchmark.GetTimePerPlanner()

  sorted_times_idx = np.argsort(times)
  times = times[sorted_times_idx]
  percentages = percentages[sorted_times_idx]
  pnames = np.array(benchmark.PlannerNames())
  pnames = pnames[sorted_times_idx]

  yL = np.max(percentages)

  fig = plt.figure(0)
  fig.patch.set_facecolor('white')
  D = benchmark.dimensions_per_level
  plt.title(benchmark.benchmark_name+"(Dimensionality: "+str(D)+"dof)")

  ax1color='#ffffff'
  ax2color='tab:red'

  ax1 = fig.gca()
  ax1.set_ylabel('Percentage of Feasible Samples')
  ax1.set_xlabel('Algorithm')
  ax1.axhline(yL,color='k',linestyle='--',alpha=0.2)
  ax1.plot(percentages,'-ok')

  ax1.xaxis.set_ticks(np.arange(0, len(percentages)))
  plannerLabelRotation=80
  xtickNames = plt.setp(ax1,xticklabels=pnames)
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


if __name__ == '__main__':
  PlotPercentageTimeVsPlanner()

