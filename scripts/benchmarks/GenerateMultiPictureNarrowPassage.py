from ParseBenchmarkFile import *
from PlotTimeVsDimension import *
from PlotTimeVsSubspaces import *
from PlotTimePercentageVsSubspaces import *
from PlotPercentageTimeVsPlanner import *
from PlotPlannersVsTime import *
from PlotCostVsSubspace import *

import glob, os, re

dir_xml = "../../data/benchmarks/narrow_passage_03_26/"
dir_xml = "../../data/benchmarks/narrow_passage/"

dir_xml = "../../data/benchmarks/narrow_passage_04_01/"
IDX_FROM_TO = [8,14]

dir_xml = "../../data/benchmarks/narrow_passage_high_values2"
IDX_FROM_TO = [None,None]

os.chdir(dir_xml)
fname_base = os.path.basename(dir_xml)
fname_pdf = fname_base + "_cost_vs_subspaces.pdf"
alpha_vec = []

Nxml = len(glob.glob("*.xml"))

D = []
N = 0
T_max = 0
for fname in sorted(glob.glob("*.xml")):
  # fname_base, fname_ext = os.path.splitext(fname)
  # fname_pdf = fname_base + "_cost_vs_subspaces.pdf"
  # print fname
  # print fname_pdf
  m = re.search('^.*\w_(0_\d\d\d?)', fname)
  alpha = m.group(1)
  alpha = alpha.replace('_','.')
  alpha = float(alpha)
  print alpha
  alpha_vec.append(alpha)
  benchmark = BenchmarkAnalytica(fname)
  N = benchmark.runcount
  T_max = benchmark.timelimit
  Dk = dict([(p.name, p.AverageTime()) for p in benchmark.planners])
  D.append(Dk)


alpha_vec = np.array(alpha_vec)
name_vec = []

D0 = D[0]
timePerAlpha = np.zeros((len(D0),len(D)))

for idx, planner in enumerate(D0):
  print planner
  name_vec.append(planner)
  a = 0
  for d in D:
    for i in d:
      if planner == i:
        print i,d[i]
        timePerAlpha[idx,a] = d[i]
        a = a+1

fig = plt.figure(0)
ax = fig.add_subplot(111)

fig.patch.set_facecolor('white')
ax.set_xlabel('Narrowness of Environment')
ax.set_ylabel('Time (s)')

ax.set_title('Runcount: %d, Timelimit: %.2f'%(N,T_max))

##Note: Arrays are reversed, and will be flipped later on
N = len(alpha_vec)
items_to_display = slice(IDX_FROM_TO[0],IDX_FROM_TO[1])

alpha_vec = np.flip(alpha_vec, 0)
timePerAlpha = np.flip(timePerAlpha, 1)
alpha_vec = alpha_vec[items_to_display]
timePerAlpha = timePerAlpha[:,items_to_display]

for idx, time in enumerate(timePerAlpha):
  if name_vec[idx]=="QRRT_(4)":
    plt.plot(alpha_vec, time, '-ok', label=name_vec[idx], lw=2)
  else:
    plt.plot(alpha_vec, time, '-x', label=name_vec[idx])

xmin, xmax = ax.get_xlim()
plt.xlim(xmax, xmin)

legend = ax.legend(loc='upper left', shadow=True)

#ax.set_yscale('log')
#ax.set_xscale('log')

for label in legend.get_texts():
    label.set_fontsize('large')

for label in legend.get_lines():
    label.set_linewidth(1.5)

pp = PdfPages(fname_pdf) 
pp.savefig(plt.gcf())
pp.close()
plt.show()
