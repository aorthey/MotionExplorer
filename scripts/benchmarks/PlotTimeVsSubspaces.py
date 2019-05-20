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


def PlotTimeVsSubspace(fname = '../../data/benchmarks/last.xml'):
  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + "_time_vs_subspaces.pdf"
  print(fname_pdf)
  pp = PdfPages(fname_pdf) 
  benchmark = BenchmarkAnalytica(fname)
  benchmark.ClipNonLargestDimensionPlanners()

  P = [(p.GetNumberSubspaces(),p.GetAveragePercentageOfFeasibleNodes(),p.AverageTime(),p.name) for p in benchmark.planners]
  P_sorted_subspace = sorted(P, key = lambda x: x[0])
  print(tabulate([P_sorted_subspace], headers=['Subspaces','PercentageFeasible','Times','Planner Name']))

  fig = plt.figure()

  ax = fig.gca()

  fig.patch.set_facecolor('white')


  D = benchmark.dimensions_per_level
  plt.title(benchmark.benchmark_name+"(Dimensionality: "+str(D)+"dof)")

  X = [p[0] for p in P]
  Y = [p[2] for p in P]

  ax.scatter(X,Y)

  ax.set_ylabel('Time(s)')
  ax.set_xlabel('Number Subspaces')


  ##annotate best and worst
  # NumberSubspaces = np.unique([t[0] for t in P])
  # for N in NumberSubspaces:
  #   Pn = filter(lambda x: x[0]==N,P)
  #   Pn = sorted(Pn, key = lambda x: x[1])
  #   name_low = Pn[0][3]+str(Pn[0][2])
  #   name_high = Pn[-1][3]
  #   y_low = Pn[0][1]
  #   y_high = Pn[-1][1]
  #   xn = N
  #   ax.annotate(name_low, xytext=(xn,y_low),
  #       arrowprops=dict(arrowstyle="->"), xy=(xn, y_low))
  #   ax.annotate(name_high, xytext=(xn,y_high),
  #       arrowprops=dict(arrowstyle="->"), xy=(xn, y_high))


  #Force Integer Labels
  ax.xaxis.set_major_locator(MaxNLocator(integer=True))
  pp.savefig(plt.gcf())
  pp.close()
  plt.show()
  plt.close()

if __name__ == '__main__':
  PlotTimeVsSubspace()
