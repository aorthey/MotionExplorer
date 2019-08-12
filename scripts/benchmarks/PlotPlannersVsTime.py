from import_matplotlib import *
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MaxNLocator
from scipy.stats import norm
import matplotlib.mlab as mlab
import scipy.special as sy
from matplotlib.ticker import MaxNLocator

from scipy.optimize import curve_fit
from scipy.misc import factorial

from scipy import interpolate

import numpy as np
import os
import sys
from ParseBenchmarkFile import *

def PlotPlannersVsTime(fname, show=False, MARK_BEST=False, MARK_WORST=False, MARK_TRIVIAL=True):
  benchmark = BenchmarkAnalytica(fname)
  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + "_last_strata_histogram.pdf"

  SHOW_DISTRIBUTIONS = False
  pp = PdfPages(fname_pdf)

  times_last_dof = benchmark.GetAverageTimePerPlannerPerDimensionalityMatrix()
  times_last_dof = times_last_dof[-1,:]

  time_trivial = 0
  index_trivial = 0
  for i in range(0,len(benchmark.planners)):
    if len(benchmark.planners[i].dimensions_per_level)==1:
      time_trivial = times_last_dof[i]
      index_trivial = i


  t_min = float(np.nanmin(times_last_dof))
  t_max = float(np.nanmax(times_last_dof))
  p_min = np.nanargmin(times_last_dof,axis=0)
  p_max = np.nanargmax(times_last_dof,axis=0)
  times_last_dof = times_last_dof[~np.isnan(times_last_dof)]

  T = np.linspace(t_min,t_max,len(times_last_dof))

  ###################################################################################
  #Display DATA
  ###################################################################################
  times_last_dof = np.sort(times_last_dof)

  fig = plt.figure(0)
  ax = fig.add_subplot(111)

  nr_bins = 50

  if SHOW_DISTRIBUTIONS:
    n, bins, patches = plt.hist(times_last_dof, nr_bins, normed=1, facecolor='0.8')

    ####################################################################################
    ##POisson Distribution Fit
    ####################################################################################
    #bin_middles = 0.5*(bins[1:] + bins[:-1])
    #def poisson(k, lamb):
    #    return (lamb**k/factorial(k)) * np.exp(-lamb)

    #parameters, cov_matrix = curve_fit(poisson, bin_middles, n) 
    #x_plot = np.linspace(t_min, t_max, 100)
    ##plt.plot(x_plot, poisson(x_plot, *parameters), 'r-', lw=2)
    #print parameters

    ###################################################################################
    #Normal Distribution Fit
    ###################################################################################
    (mu, sigma) = norm.fit(times_last_dof)
    y = mlab.normpdf( bins, mu, sigma)
    l = plt.plot(bins, y, 'k-', lw=2)
    ax.set_ylabel('Percentage of Heuristics')
    vy_worst = 2.0/len(times_last_dof)
    vy_best = 2.0/len(times_last_dof)
  else:
    n, bins, patches = plt.hist(times_last_dof, nr_bins, facecolor='0.8')
    ax.set_ylabel('Number of Algorithms')
    vy_worst = n[-1]
    vy_best = n[0]

  ###################################################################################
  #Annotate
  ###################################################################################

  ymin,ymax = ax.get_ylim()
  len_bin = ((t_max-t_min)/nr_bins)
  ty = 0.5*(ymax-ymin)

  if MARK_WORST:
    vx = t_max - 0.5*len_bin
    tx = t_min + 0.7*(t_max-t_min)
    ax.annotate(benchmark.planners[p_max].name, xytext=(tx,ty),
        arrowprops=dict(arrowstyle="->"), xy=(vx, vy_worst))
  
  if MARK_BEST:
    vx = t_min + 0.5*len_bin
    tx = t_min + 0.1*(t_max-t_min)
    ax.annotate(benchmark.planners[p_min].name, xytext=(tx,ty),
        arrowprops=dict(arrowstyle="->"), xy=(vx, vy_best))

  if MARK_TRIVIAL:
    #find number of bin
    bin_idx = len(np.argwhere(bins <= time_trivial))-1

    if bin_idx >= len(n):
      vy = n[-1]
    else:
      vy = n[bin_idx]

    len_bin = ((t_max-t_min)/(nr_bins+1))
    vx = t_min + (bin_idx+0.5)*len_bin
    tx = t_min + 0.5*(t_max-t_min)
    ax.annotate(benchmark.planners[index_trivial].name, xytext=(tx,ty),
        arrowprops=dict(arrowstyle="->"), xy=(vx, vy))

  ax.set_xlabel('Time (s)')
  ax.yaxis.set_major_locator(MaxNLocator(integer=True))

  number_dofs = benchmark.dimensions_per_level[-1]
  #plt.title(r'$\mathrm{Histogram\ of\ Heuristic\ Runtime\ with\ %d\ DoFs:}\ \mu=%.3fs,\ \sigma=%.3fs$' %(number_dofs, mu, sigma))

  pp.savefig(plt.gcf())
  pp.close()
  if show:
    plt.show()
  plt.close()
  del benchmark

if __name__ == '__main__':
  fname = '../../data/benchmarks/last.xml'
  PlotPlannersVsTime(fname, show=True)
