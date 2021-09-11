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
      if benchmark.runcount > Nruns:
        Nruns = benchmark.runcount
      env_names_tmp.append(env)
      for p in benchmark.planners:
        pname = p.name
        pname = pname.replace("#","\#")
        pname = re.sub('[Ss]tar', '*', pname)

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
        print(dir_name+d2)
        envs = []
        for root, tmp, files in os.walk(dir_name+d2):
          for f in files:
            env = os.path.splitext(f)[0]
            env = env.replace("_"," ")
            envs.append(env)
        topics[d2] = envs

    # for t in topics:
    #   print(t)
    #   for p in topics[t]:
    #     print(p)

    ## create tex
    f = open(texName, 'w')

    f.write("\\documentclass{article}\n")
    f.write("\\usepackage{tabularx}\n")
    f.write("\\usepackage{rotating}\n")
    f.write("\\usepackage{makecell}\n")
    f.write("\\usepackage[text={174mm,258mm}, papersize={210mm,297mm}, columnsep=12pt, headsep=21pt, centering]{geometry}")
    f.write("\\begin{document}\n\n")
    f.write("\\newcolumntype{V}{>{\\centering\\arraybackslash}m{.036\\linewidth}}\n")
    f.write("\\newcolumntype{Z}{>{\\raggedleft\\arraybackslash}m{.014\\linewidth}}\n")
    f.write("\\newcolumntype{Y}{>{\\centering\\arraybackslash}X}\n")
    f.write("\\newcolumntype{+}{!{\\vrule width 1pt}}\n")

    f.write("\\begin{table*}[t]\n")
    f.write("\\centering\n")
    f.write("\\renewcommand{\\cellrotangle}{90}\n")
    f.write("\\renewcommand\\theadfont{\\bfseries}\n")

    f.write("\\settowidth{\\rotheadsize}{\\theadfont 37D Shadowhand ball}\n")

    f.write("\\footnotesize\\centering\n")
    f.write("\\renewcommand{\\arraystretch}{1.2}\n")
    f.write("\\setlength\\tabcolsep{3pt}\n")

    s = "\\begin{tabularx}{\\linewidth}{|ZX+V|V|V+V|V|V+V|V|V+V|V|V+V|V|V+V|V|V|}"
    s += "\\hline\n"
    s += "& & \\multicolumn{18}{Y|}{List of Scenarios} \\\\ \\cline{3-20}"
    s += "\\hline\n"

    s += " & &  "
    for t in topics:
      n = len(topics[t])
      s += "\\multicolumn{%d}{Y+}{%s}& "%(n,t)
    s += "\\\\ \\cline{3-20}"

    # s += " & &  \
    #      \\multicolumn{3}{Y+}{Vehicle Navigation}&  \
    #      \\multicolumn{3}{Y+}{Manipulation}&  \
    #      \\multicolumn{3}{Y+}{Narrow Passages}&  \
    #      \\multicolumn{3}{Y+}{Manifolds}& \
    #      \\multicolumn{3}{Y+}{Projections}& \
    #      \\multicolumn{3}{Y+}{Multi-Robot} \
    #      \\\\ \\cline{3-20}"


    s += "\\multicolumn{2}{|>{\\centering}p{2.5cm}+}{\\rothead{Motion Planner}} &"
    for t in topics:
      s += " %"
      s += " %s\n "%(t)
      for p in topics[t]:
        s += " \\rothead{%s} & \n "%(p)

    s += " \\\\ \\hline"

    # s += " %Vehicle Navigation\n "
    # s += " \\rothead{06D Dubins Car} & \n "
    # s += " \\rothead{06D Drone} & \n "
    # s += " \\rothead{06D ReedsSheppCar} & \n "
    # s += " %Manipulation\n "
    # s += " \\rothead{37D ShadowHand Ball} & \n "
    # s += " \\rothead{07D KUKA Windshield} & \n "
    # s += " \\rothead{07D Franka Emica} & \n "
    # s += " %Narrow Passage\n "
    # s += " \\rothead{06D Bugtrap} & \n "
    # s += " \\rothead{06D Double Lshape} & \n "
    # s += " \\rothead{10D Chain Egress} & \n "
    # s += " %Manifold\n "
    # s += " \\rothead{02D Torus} & \n "
    # s += " \\rothead{02D Sphere} & \n "
    # s += " \\rothead{02D Klein Bottle} &\n "
    # s += " %Projections\n "
    # s += " \\rothead{02D Torus} & \n "
    # s += " \\rothead{02D Sphere} & \n "
    # s += " \\rothead{57D Octopus Animation} &\n "
    # s += " %Multi-Robot\n "
    # s += " \\rothead{18D mobile manipulators} & \n "
    # s += " \\rothead{36D drone formation} & \n "
    # s += " \\rothead{20D Disk Coordination}\n "
    # s += " \\\\ "
    # s += " \\hline"
    f.write(s)


    f.write("0 & \\mbox{QRRT}& 99.99 & 99.99 & 99.99 & 99.99 &  99.99 & 99.99 & 99.99 & 99.99 & 99.99 & 99.99 & 99.99 & 4.45  &  1.86  &  \\textbf{0.55}  &  2.01  & 35.63  &  19.80  &  60.00 \\\\") 

    for (ctr,p) in enumerate(planner_names):
      pstr = str(ctr+1) + " & \\mbox{" + str(p)+"} & "
      for (ctrt,t) in enumerate(topics):
        for (ctrenv, env) in enumerate(topics[t]):
          if p in d[env]:
            s = d[env].get(p)
            # pstr += (" %.3f " % s)
            bestPlanner = min(d[env],key=d[env].get)
            if p == bestPlanner:
              pstr += (" \\textbf{%.3f} " % s)
            else:
              pstr += (" %.3f " % s)
          else:
            pstr += " - " 
          if ctrenv < len(topics[t])-1:
            pstr += " &" 
        if ctrt < len(topics)-1:
          pstr += " &" 


              # bestPlanner = min(d[e],key=d[e].get)
              # if p in d[e]:
              #   t = d[e].get(p)
              #   if p == bestPlanner:
              #     pstr += (" \\textbf{%.3f} " % t)
              #   else:
              #     pstr += (" %.3f " % t)
              # else:
              #   pstr += " - "
      pstr += " \\\\ \n"
      f.write(pstr)

    f.write("\end{tabularx}\n")


    f.write("\\caption{Runtime (s) of motion planner on $7$ scenarios, each \
        averaged over $10$ runs with cut-off time limit of $60$s. Entry $-$ \
        means that planner does not support compound state spaces.}\n")

    f.write("\\end{table*}\n")
    f.write("\end{document}\n")
    f.close()

    os.system("pdflatex -output-directory %s %s" % (os.path.dirname(texName),
      texName))
    print("Wrote tex file to %s" % texName)
    os.system("apvlv %s" % pdfName)

if __name__ == '__main__':
    fnames = glob.glob("../../data/benchmarks/Review2021/*.xml")
    PlotTable(fnames)
