import numpy as np
import sys
import re
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 30})
plt.rcParams.update({'font.family': 'cmr'})

from matplotlib.backends.backend_pdf import PdfPages
from xml.dom.minidom import parse
import os
import glob

def createBenchmark(folderName):
  files = glob.glob(folderName + "/*.xml")

  fname_pdf = folderName + "/average.pdf"
  
  meandData = None
  runcount = 0
  timelimit = 0
  vnames = []
  b_names = []
  for file in files:
    print(file)
    benchmark_name, runcount, timelimit, vnames, vtimes = readXML(file)

    b_names.append(benchmark_name)
    #print("vtimes", vtimes)
    #print(vtimes.shape)

    if vtimes.shape[0]<=1:
      vtimes = np.vstack((vtimes,vtimes))

    means = np.mean(vtimes,axis=0)

    #print("vtimes", vtimes)
    #print("means", means)

    if meandData is None:
      meandData = np.zeros(means.shape)
    meandData = meandData + means
  
  print("runcount", runcount)
  print("timelimit", timelimit)
  print("vnames",vnames)

  #print("meandData", meandData)
  meandData = meandData / len(files)
  #print("meandData2" , meandData)
  
  timelimit = 1.25 * meandData.max()

  #vnames = [w.replace('_', '\n') for w in vnames]

  plotBenchmarks(fname_pdf, runcount, timelimit, vnames, meandData, b_names)

###################################################################################
#GET DATA
###################################################################################
def readXML(fname):
  doc = parse(fname)
  name = doc.getElementsByTagName("name")[0]

  benchmark_name = name.firstChild.data
  print("benchmark name", benchmark_name)

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
    #print ("name:", name,  " time: ", np.mean(vtimes[:,p_ctr]), "+/-", np.std(vtimes[:,p_ctr]))
    p_ctr=p_ctr+1
  
  return benchmark_name, runcount, timelimit, vnames, vtimes



###################################################################################
#PLOTTING
###################################################################################


def plotBenchmarks(fname_pdf, runcount, timelimit, vnames, means, benchmark_name):
  pp = PdfPages(fname_pdf)
  # fig = plt.figure(0)
  fig = plt.figure(figsize=(20,10))
  ax = fig.gca()
  fig.patch.set_facecolor('white')
  # ax.set_xlabel('Algorithm')
  ax.set_ylim([0,timelimit+0.15*timelimit])
  ax.set_ylabel('Time (s)')

  #if vtimes.shape[0]<=1:
    #vtimes = np.vstack((vtimes,vtimes))

  #means = np.mean(vtimes,axis=0)

  #if histogram:
    #plt.boxplot(vtimes, notch=0, sym='k+', vert=1, whis=1.5)
  #else:
  print(means)
  plt.bar(vnames, means)
    # plt.boxplot(vtimes, notch=0, sym='k+', vert=1, whis=1.5)

  plannerLabelRotation=85
  xtickNames = plt.setp(ax,xticklabels=vnames)
  plt.setp(xtickNames, rotation=plannerLabelRotation)

  ###################################################################################
  #max lowest mean value be visualized as bold
  ###################################################################################
  bestTimeIdx = means.argmin() # np.mean(vtimes,axis=0).argmin()
  bestTime = means.min() # np.mean(vtimes,axis=0).min()
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
  
  benchmark_name_txt = ', '.join(benchmark_name)
  plt.title(benchmark_name_txt)
  ###################################################################################

  plt.tight_layout()
  pp.savefig(plt.gcf(), pad_inches=0.0, bbox_inches='tight')
  pp.close()
  plt.show()

  cmd = "apvlv "+fname_pdf
  os.system(cmd)










if __name__ == '__main__':
  folderName = "../../data/benchmarks/average_all"
  createBenchmark(folderName)

