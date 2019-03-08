import numpy as np
import sys
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from xml.dom.minidom import parse
import os

def XMLtoPDF(fname):
  fname_base, fname_ext = os.path.splitext(fname)
  fname_pdf = fname_base + ".pdf"

  ###################################################################################
  #GET DATA
  ###################################################################################
  doc = parse(fname)
  name = doc.getElementsByTagName("name")[0]
  print name

  planners = doc.getElementsByTagName("planner")
  nr_planners = int(doc.getElementsByTagName("number_of_planners")[0].firstChild.data)
  runcount = int(doc.getElementsByTagName("run_count")[0].firstChild.data)
  timelimit = float(doc.getElementsByTagName("max_time")[0].firstChild.data)
  print "planners:",nr_planners," runs:",runcount

  vtimes = np.zeros((runcount,nr_planners))
  vnodes = np.zeros((runcount,nr_planners))
  vsuccess = np.zeros((runcount,nr_planners))
  vnames = []

  p_ctr = 0
  for planner in planners:
    name = planner.getElementsByTagName("name")[0].firstChild.data
    print "name:", name
    vnames.append(name)
    runs = planner.getElementsByTagName("run")
    c_ctr = 0
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
    p_ctr=p_ctr+1

  #print vnodes
  #print vsuccess

  ###################################################################################
  #PLOTTING
  ###################################################################################
  pp = PdfPages(fname_pdf)
  fig = plt.figure(0)
  ax = fig.gca()
  fig.patch.set_facecolor('white')
  ax.set_xlabel('Algorithm')
  ax.set_ylim([0,timelimit+0.15*timelimit]);
  ax.set_ylabel('Time (s)')

  if vtimes.shape[0]<=1:
    vtimes = np.vstack((vtimes,vtimes))

  plt.boxplot(vtimes, notch=0, sym='k+', vert=1, whis=1.5)

  plannerLabelRotation=80
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
        ticklabel.set_fontsize('large')
      ctr=ctr+1

  ###################################################################################
  #visualize timelimit and runcount in upper right corner
  ###################################################################################
  txt = "runcount=%3.0d"%runcount
  ax.text(0.85, 0.95, txt, horizontalalignment='center', verticalalignment='center', transform = ax.transAxes)
  print timelimit
  if timelimit < 1:
    txt = "timelimit=%3.1f"%timelimit+"s"
  else:
    txt = "timelimit=%3.0f"%timelimit+"s"
  ax.text(0.85, 0.9, txt, horizontalalignment='center', verticalalignment='center', transform = ax.transAxes)
  ax.axhline(timelimit,color='k',linestyle='--')

  plt.tight_layout()
  pp.savefig(plt.gcf())
  pp.close()

  cmd = "apvlv "+fname_pdf
  os.system(cmd)

if __name__ == '__main__':
  Nargs = len(sys.argv)
  args = str(sys.argv)
  if Nargs>1:
    for fname in sys.argv[1:]:
      XMLtoPDF(fname)
  else:
    fname = "../../data/benchmarks/02D_disk_2018_09_28_14:06:26.xml"
    fname = "../../data/benchmarks/02D_disk_2018_09_28_14:17:30.xml"
    XMLtoPDF(fname)

