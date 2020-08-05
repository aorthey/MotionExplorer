import numpy as np
import sys
import re
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 30})
plt.rcParams.update({'font.family': 'cmr'})

from matplotlib.backends.backend_pdf import PdfPages
from xml.dom.minidom import parse
import os

def XMLtoPDF(fname, histogram=False):
  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + ".pdf"

  ###################################################################################
  #GET DATA
  ###################################################################################
  doc = parse(fname)
  name = doc.getElementsByTagName("name")[0]

  planners = doc.getElementsByTagName("planner")
  nr_planners = int(doc.getElementsByTagName("number_of_planners")[0].firstChild.data)
  runcount = int(doc.getElementsByTagName("run_count")[0].firstChild.data)
  timelimit = float(doc.getElementsByTagName("max_time")[0].firstChild.data)
  print ("planners:",nr_planners," runs:",runcount)

  vtimes = np.zeros((runcount,nr_planners))
  vnodes = np.zeros((runcount,nr_planners))
  vsuccess = np.zeros((runcount,nr_planners))
  vnames = []

  p_ctr = 0
  for planner in planners:
    name = planner.getElementsByTagName("name")[0].firstChild.data

    name = re.sub("([Ss]tar)", '*', name)

    vnames.append(name)
    runs = planner.getElementsByTagName("run")
    c_ctr = 0
    runtimes = []
    for run in runs:
      sid = run.getAttribute("number")
      time = run.getElementsByTagName("time")[0].firstChild.data
      nodes = run.getElementsByTagName("nodes")[0].firstChild.data
      success = int(run.getElementsByTagName("success")[0].firstChild.data)
      #print "run ",sid," time ",time," nodes ", nodes, " success ","No" if success==0 else "Yes"
      vtimes[c_ctr,p_ctr] = time
      vnodes[c_ctr,p_ctr] = nodes
      vsuccess[c_ctr,p_ctr] = success
      c_ctr=c_ctr+1
    print ("name:", name, 
        " time: ", np.mean(vtimes[:,p_ctr]), 
        "+/-", np.std(vtimes[:,p_ctr]))
    p_ctr=p_ctr+1


  ###################################################################################
  #PLOTTING
  ###################################################################################
  pp = PdfPages(fname_pdf)
  # fig = plt.figure(0)
  fig = plt.figure(figsize=(20,10))
  ax = fig.gca()
  fig.patch.set_facecolor('white')
  # ax.set_xlabel('Algorithm')
  ax.set_ylim([0,timelimit+0.15*timelimit]);
  ax.set_ylabel('Time (s)')

  if vtimes.shape[0]<=1:
    vtimes = np.vstack((vtimes,vtimes))

  means = np.mean(vtimes,axis=0)

  if histogram:
    plt.boxplot(vtimes, notch=0, sym='k+', vert=1, whis=1.5)
  else:
    print(means)
    plt.bar(vnames, means)
    # plt.boxplot(vtimes, notch=0, sym='k+', vert=1, whis=1.5)

  plannerLabelRotation=85
  xtickNames = plt.setp(ax,xticklabels=vnames)
  plt.setp(xtickNames, rotation=plannerLabelRotation)

  ###################################################################################
  #max lowest mean value be visualized as bold
  ###################################################################################
  bestTimeIdx = np.mean(vtimes,axis=0).argmin()
  bestTime = np.mean(vtimes,axis=0).min()
  ##only if bestTime is unique
  if bestTime < timelimit:
    ctr = 0
    for ticklabel in plt.gca().get_xticklabels():
      if ctr == bestTimeIdx:
        ticklabel.set_fontweight('extra bold')
      ctr=ctr+1

  ###################################################################################
  #visualize timelimit and runcount in upper right corner
  ###################################################################################
  # txt = "runcount=%3.0d"%runcount
  # ax.text(0.65, 0.96, txt, horizontalalignment='left', verticalalignment='center', transform = ax.transAxes)
  # if timelimit < 1:
  #   txt = "timelimit=%3.1f"%timelimit+"s"
  # else:
  #   txt = "timelimit=%3.0f"%timelimit+"s"
  # ax.text(0.65, 0.9, txt, horizontalalignment='left', verticalalignment='center', transform = ax.transAxes)
  ax.axhline(timelimit,color='k',linestyle='--')
  txt = "[%3.0d run average]"%runcount
  ax.text(0.65, 0.93, txt, horizontalalignment='left', verticalalignment='center', transform = ax.transAxes)
  ###################################################################################

  plt.tight_layout()
  pp.savefig(plt.gcf(), pad_inches=0.0, bbox_inches='tight')
  pp.close()
  # plt.show()

  # cmd = "apvlv "+fname_pdf
  # os.system(cmd)

if __name__ == '__main__':
  Nargs = len(sys.argv)
  args = str(sys.argv)
  if Nargs>1:
    for fname in sys.argv[1:]:
      XMLtoPDF(fname)
  else:
    fname = "../../data/benchmarks/54D_octopus_2020_05_25_13:54:57.xml"
    fname = "../../data/benchmarks/48D_SE3C_drones_2020_05_25_14:09:49.xml"
    fname = "../../data/benchmarks/72D_SE2RN_R2_mobile_manipulators_2020_05_25_14:17:14.xml"
    fname = "../../data/benchmarks/30D_airport_2020_05_25_14:37:03.xml"
    fname = "../../data/benchmarks/24D_crossing_cars_2020_05_25_14:48:27.xml"
    fname = "../../data/benchmarks/21D_box_folding_2020_05_25_14:56:05.xml"
    fname = "../../data/benchmarks/30D_airport.xml"
    fname = "../../data/benchmarks/last.xml"
    XMLtoPDF(fname)

