from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
import scipy.special as sy
from matplotlib.ticker import MaxNLocator

from ParseBenchmarkFile import *
from tabulate import tabulate
import numpy as np
import os
import sys

def PlotPercentageVsSubspace(fname = '../../data/benchmarks/last.xml'):
  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + "_percentage_vs_subspaces.pdf"
  pp = PdfPages(fname_pdf) 
  benchmark = BenchmarkAnalytica(fname)
  benchmark.ClipNonLargestDimensionPlanners()

  P = [(p.GetNumberSubspaces(),p.GetAveragePercentageOfFeasibleNodes(),p.AverageTime(),p.name) for p in benchmark.planners]
  P_sorted_subspace = sorted(P, key = lambda x: x[0])
  print tabulate(P_sorted_subspace, headers=['Subspaces','PercentageFeasible','Times','Planner Name'])

  fig = plt.figure()

  ax = fig.gca()
  # ax = fig.add_subplot(111, projection='3d')

  fig.patch.set_facecolor('white')
  D = benchmark.dimensions_per_level
  plt.title(benchmark.benchmark_name+"(Dimensionality: "+str(D)+"dof)")


  X = [p[0] for p in P]
  Y = [p[1] for p in P]

  ax.scatter(X,Y)

  ax.set_ylabel('Percentage of Feasible Samples')
  ax.set_xlabel('Number Subspaces')

  #Force Integer Labels
  ax.xaxis.set_major_locator(MaxNLocator(integer=True))

  # plt.tight_layout()
  pp.savefig(plt.gcf())
  pp.close()
  plt.show()
  plt.close()

def PlotTimePercentageVsSubspace(fname = '../../data/benchmarks/last.xml'):
  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + "_percentage_time_vs_subspaces.pdf"
  pp = PdfPages(fname_pdf) 
  benchmark = BenchmarkAnalytica(fname)
  benchmark.ClipNonLargestDimensionPlanners()

  P = [(p.GetNumberSubspaces(),p.GetAveragePercentageOfFeasibleNodes(),p.AverageTime(),p.name) for p in benchmark.planners]
  P_sorted_subspace = sorted(P, key = lambda x: x[0])
  print tabulate(P_sorted_subspace, headers=['Subspaces','PercentageFeasible','Times','Planner Name'])
  X = [p[0] for p in P]
  Y_percentage = [p[1] for p in P]
  Y_time = [p[2] for p in P]

  fig, (ax1, ax2) = plt.subplots(1,2)
  fig.patch.set_facecolor('white')
  D = benchmark.dimensions_per_level
  plt.title(benchmark.benchmark_name+"(Dimensionality: "+str(D)+"dof)")

  ax1.scatter(X,Y_percentage)
  ax1.set_ylabel('Percentage of Feasible Samples')
  ax1.set_xlabel('Number Subspaces')
  ax1.xaxis.set_major_locator(MaxNLocator(integer=True))

  ax2.scatter(X,Y_time)
  ax2.set_ylabel('Time (s)')
  ax2.set_xlabel('Number Subspaces')
  ax2.xaxis.set_major_locator(MaxNLocator(integer=True))

  # plt.tight_layout()
  pp.savefig(plt.gcf())
  pp.close()
  plt.show()
  plt.close()

if __name__ == '__main__':
  PlotTimePercentageVsSubspace()
