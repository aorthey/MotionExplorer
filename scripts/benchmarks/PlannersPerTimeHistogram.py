import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MaxNLocator
from scipy.stats import norm
import matplotlib.mlab as mlab
import scipy.special as sy

from scipy.optimize import curve_fit
from scipy.misc import factorial


from scipy import interpolate

import numpy as np
import os
import sys
from ParseBenchmarkFile import *

fname = '../../data/benchmarks/last.xml'
benchmark = BenchmarkAnalytica(fname)
fname_base, fname_ext = os.path.splitext(fname)
fname_pdf = fname_base + "_last_strata_histogram.pdf"
pp = PdfPages(fname_pdf)

times_last_dof = benchmark.GetAverageTimePerPlannerPerDimensionalityMatrix()
times_last_dof = times_last_dof[-1,:]
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
n, bins, patches = plt.hist(times_last_dof, nr_bins, normed=1, facecolor='0.8')

###################################################################################
#POisson Distribution Fit
###################################################################################
bin_middles = 0.5*(bins[1:] + bins[:-1])
def poisson(k, lamb):
    return (lamb**k/factorial(k)) * np.exp(-lamb)

parameters, cov_matrix = curve_fit(poisson, bin_middles, n) 
x_plot = np.linspace(t_min, t_max, 100)
plt.plot(x_plot, poisson(x_plot, *parameters), 'r-', lw=2)
print parameters

###################################################################################
#Normal Distribution Fit
###################################################################################
(mu, sigma) = norm.fit(times_last_dof)
y = mlab.normpdf( bins, mu, sigma)
l = plt.plot(bins, y, 'k-', lw=2)

###################################################################################
#Annotate
###################################################################################

ymin,ymax = ax.get_ylim()
len_bin = ((t_max-t_min)/nr_bins)
vx = t_max - 0.5*len_bin
vy = 2.0/len(times_last_dof)
tx = 0.9*(t_max-t_min)
ty = 0.5*(ymax-ymin)

ax.annotate(benchmark.planners[p_max].name, xytext=(tx,ty),
    arrowprops=dict(arrowstyle="->"), xy=(vx, vy))
ax.set_ylabel('Percentage of Heuristics')
ax.set_xlabel('Time (s)')

number_dofs = benchmark.dimensions_per_level[-1]
plt.title(r'$\mathrm{Histogram\ of\ Heuristic\ Runtime\ with\ %d\ DoFs:}\ \mu=%.3fs,\ \sigma=%.3fs$' %(number_dofs, mu, sigma))

pp.savefig(plt.gcf())
pp.close()
plt.show()
plt.close()
