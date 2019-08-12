from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
import scipy.special as sy
from matplotlib.ticker import MaxNLocator

from ParseBenchmarkFile import *
from tabulate import tabulate
import numpy as np
import os
import sys


def PlotCostVsSubspace(fname = '../../data/benchmarks/last.xml'):
  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + "_cost_vs_subspaces.pdf"
  print(fname_pdf)
  pp = PdfPages(fname_pdf) 
  benchmark = BenchmarkAnalytica(fname)
  benchmark.ClipNonLargestDimensionPlanners()

  ### Compute Cost

  #p.AverageTime()/np.sum(p.GetAverageNodesPerLevel())

  P = [(p.name, float(np.sum(p.run_nodes_per_level[:,-1]))/float(np.sum(p.run_time)), p.GetNumberSubspaces()) for p in benchmark.planners]
  print(benchmark.planners[0].run_nodes_per_level)
  P_sorted_subspace = sorted(P, key = lambda x: x[0])
  print(tabulate([P_sorted_subspace],
      headers=['Name','NodesPerTime','Subspaces']))


  fig = plt.figure(0)
  ax = fig.gca()
  fig.patch.set_facecolor('white')


  X = [p[2] for p in P]
  Y = [p[1] for p in P]

  ax.scatter(X,Y)

  ax.set_ylabel('Time(s)')
  ax.set_xlabel('Number Subspaces')
  ax.set_ylim([np.min(Y),np.max(Y)])

  plt.tight_layout()

  D = benchmark.dimensions_per_level
  plt.title(benchmark.benchmark_name+"(Dimensionality: "+str(D)+"dof)")

  #Force Integer Labels
  ax.xaxis.set_major_locator(MaxNLocator(integer=True))
  pp.savefig(plt.gcf())
  pp.close()
  plt.show()
  plt.close()

if __name__ == '__main__':
  PlotCostVsSubspace()
