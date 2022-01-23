#!/usr/bin/env python3
from ParseBenchmarkFile import *
from collections import defaultdict
import subprocess
import glob
import re

def PlotTable(fnames):
    texName = '../../data/benchmarks/table.tex'
    pdfName = '../../data/benchmarks/table.pdf'

    d = defaultdict(dict)
    planner_names_tmp = []
    env_names_tmp = []

    Nruns = 0
    for f in fnames:
      benchmark = BenchmarkAnalytica(f)
      env = benchmark.benchmark_name
      env = env.replace("_"," ")
      print(f, env)
      if benchmark.runcount > Nruns:
        Nruns = benchmark.runcount
      env_names_tmp.append(env)
      for p in benchmark.planners:
        pname = p.name
        pname = pname.replace("#","\#")
        pname = re.sub('[Ss]tar', '*', pname)
        # pname = re.sub('CForest', 'CForest<RRT>', pname)

        planner_names_tmp.append(pname)
        d[env][pname] = p.AverageTime()

    ## unique list
    planner_names = []
     
    for p in planner_names_tmp:
      if p not in planner_names:
        planner_names.append(p) 

    env_names = []
    for e in env_names_tmp:
      if e not in env_names:
        env_names.append(e)
    env_names = sorted(env_names)

    Nplanner = len(planner_names)
    Nenv = len(env_names)

    ####################################################
    ##### FIND OUT ENVIRONMENTS ########################
    ####################################################
    dir_name = "../../data/experiments/21-Review/"
    print(80*"X")
    topics = defaultdict(dict)
    for r1, d1, f1 in os.walk(dir_name):
      for d2 in d1:
        envs = []
        for root, tmp, files in os.walk(dir_name+d2):
          for f in files:
            env = os.path.splitext(f)[0]
            env = env.replace("_"," ")
            print(dir_name+d2, env)
            envs.append(env)
        tmp = d2.replace("_"," ")
        topics[tmp] = envs

    ## create tex
    f = open(texName, 'w')

    f.write("\\documentclass{article}\n")
    f.write("\\usepackage{tabularx}\n")
    f.write("\\usepackage{rotating}\n")
    f.write("\\usepackage{makecell}\n")
    f.write("\\usepackage[text={174mm,258mm}, papersize={210mm,297mm}, columnsep=12pt, headsep=21pt, centering]{geometry}")
    f.write("\\begin{document}\n\n")

    f.write("\\newcolumntype{V}{>{\\centering\\arraybackslash}m{.033\\linewidth}}\n")
    f.write("\\newcolumntype{W}{>{\\centering\\arraybackslash}m{.036\\linewidth}}\n")
    f.write("\\newcolumntype{Z}{>{\\raggedleft\\arraybackslash}m{.01\\linewidth}}\n")
    f.write("\\newcolumntype{Y}{>{\\centering\\arraybackslash}X}\n")
    f.write("\\newcolumntype{+}{!{\\vrule width 1.2pt}}\n")

    f.write("\\begin{table*}[t]\n")
    f.write("\\centering\n")
    f.write("\\renewcommand{\\cellrotangle}{90}\n")
    f.write("\\renewcommand\\theadfont{\\bfseries}\n")

    f.write("\\settowidth{\\rotheadsize}{\\theadfont 37D Shadowhand ball}\n")

    f.write("\\footnotesize\\centering\n")
    f.write("\\renewcommand{\\arraystretch}{1.2}\n")
    f.write("\\setlength\\tabcolsep{3pt}\n")

    s = "\\begin{tabularx}{\\linewidth}{|ZX+W|W|W+V|V|V+V|V|V+V|V|V+V|V|V+V|V|V+}"
    s += "\\hline\n"
    s += "& & \\multicolumn{18}{Y+}{List of Scenarios} \\\\ \\cline{3-20}\n"
    # s += "\\hline\n"

    s += " & & \n"
    for (ctr,t) in enumerate(sorted(topics)):
      n = max(len(topics[t]),3)
      topic = t.replace("_"," ")
      s += "\\multicolumn{%d}{Y+}{%s} "%(n,topic)
      if ctr < len(topics)-1:
        s += " &\n" 
    s += "\\\\ \\cline{3-20}\n\n"

    s += "\\multicolumn{2}{|>{\\centering}p{2.5cm}+}{\\rothead{Motion Planner}}\n"
    # for t in topics:
    for (ctr,t) in enumerate(sorted(topics)):
      s += " %" #comment
      s += " %s\n"%(t) #comment
      n = len(topics[t])
      if n > 0:
        for p in topics[t]:
          s += " & \\rothead{%s} \n"%(p)
      else:
        s += " & \\rothead{Placeholder} \n"
        s += " & \\rothead{Placeholder} \n"
        s += " & \\rothead{Placeholder} \n"

    s += " \\\\ \\hline\n"

    f.write(s)

    for (ctr,p) in enumerate(planner_names):
      pstr = str(ctr+1) + " & \\mbox{" + str(p) + "} & \n"
      for (ctrt,t) in enumerate(sorted(topics)):
        n = len(topics[t])
        if n > 0:
          for (ctrenv, env) in enumerate(topics[t]):
            if p in d[env]:
              s = d[env].get(p)
              # pstr += (" %.3f " % s)
              bestPlanner = min(d[env],key=d[env].get)
              if env.startswith('02D') and (s < 10.0):
                if p == bestPlanner:
                  pstr += (" \\textbf{%.3f} " % s)
                else:
                  pstr += (" %.3f " % s)
              else:
                if p == bestPlanner:
                  pstr += (" \\textbf{%.2f} " % s)
                else:
                  pstr += (" %.2f " % s)
            else:
              pstr += " - " 
            if ctrenv < len(topics[t])-1:
              pstr += " &" 
          if ctrt < len(topics)-1:
            pstr += " & % "
            pstr += ("%s\n"%t)
        else:
          pstr += " &&"
          if ctrt < len(topics)-1:
            pstr += "&" 
          pstr +=" % "
          pstr += ("%s\n"%t)

      pstr += " \\\\ \n"
      f.write(pstr)

    f.write("\\hline\n")
    f.write("\\end{tabularx}\n")


    f.write("\\caption{Runtime (s) of motion planner on $18$ scenarios in $6$ \
    categories, each averaged over $2$ runs with cut-off time limit of $60$s. \
    Entry $-$ means that planner does not support this planning scenario.}\n")

    f.write("\\end{table*}\n")
    f.write("\\end{document}\n")
    f.close()

    os.system("pdflatex -output-directory %s %s" % (os.path.dirname(texName),
      texName))
    print("Wrote tex file to %s" % texName)
    os.system("apvlv %s" % pdfName)

if __name__ == '__main__':
    fnames = glob.glob("../../data/benchmarks/Review2021/*.xml")
    PlotTable(fnames)
