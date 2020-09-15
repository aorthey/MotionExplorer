from ParseBenchmarkFile import *
from collections import defaultdict
import subprocess
import glob
import re

def PlotTable(fnames):
    texName = '../../data/benchmarks/table.tex'
    pdfName = '../../data/benchmarks/table.pdf'

    # fnames = sorted(fnames)

    d = defaultdict(dict)
    planner_names_tmp = []
    env_names_tmp = []

    Nruns = 0
    for f in fnames:
        print(f)
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

    # sys.exit(0)
    ## unique list
    planner_names = []
     
    for p in planner_names_tmp:
      if p not in planner_names:
        planner_names.append(p) 

    env_names = []
    for e in env_names_tmp:
      if e not in env_names:
        env_names.append(e)
    # print(planner_names_tmp)
    # print(planner_names)
    # sys.exit(0)
    env_names = sorted(env_names)
    

    Nplanner = len(planner_names)
    Nenv = len(env_names)

    ## create tex
    f = open(texName, 'w')

    f.write("\documentclass{article}\n")
    # %\usetikzlibrary{...}% tikz package already loaded by 'tikz' option
    f.write("\\usepackage{makecell}\n")
    f.write("\\usepackage{tabulary}\n")
    f.write("\\usepackage{rotating}\n")
    f.write("\\usepackage{booktabs}\n")
    f.write("\\usepackage{multirow}\n")

    f.write("\\begin{document}\n\n")

    f.write("\\begin{table}[!t]\n")
    f.write("\\centering\n")

    # f.write("\\setcellgapes{2pt}\n")
    # f.write("\\makegapedcells\n")
    f.write("\\renewcommand{\\cellrotangle}{90}\n")
    f.write("\\renewcommand\\theadfont{\\bfseries}\n")


    # f.write("\\renewcommand\\theadset{\\linespread{1.0}}\n")
    # f.write("\\settowidth\\rotheadsize{\\theadfont graduate (\\%)}\n")

    klongest = 0
    env_name_length = 0
    for k in range(0, Nenv):
      l = len(env_names[k])
      if l > env_name_length:
        env_name_length = l
        klongest = k

    f.write("\\settowidth{\\rotheadsize}{\\theadfont %s}\n" % (env_names[klongest]))

    f.write("\\newcolumntype{Y}{>{\\raggedleft\\arraybackslash}X}\n")
    # f.write("\\newcolumntype{C}[1]{>{\\centering\\arraybackslash}p{#1}}\n")

    f.write("\\footnotesize\\centering\n")
    f.write("\\renewcommand{\\arraystretch}{1.2}\n")
    f.write("\\setlength\\tabcolsep{3pt}\n")



    # st = "\\begin{tabularx}{\\textwidth}{@{} l *{%d}{Y}| @{}}" % Nenv
    st = "\\begin{tabulary}{\\linewidth}{@{}LL"
    for k in range(0, Nenv):
      st += "C"
    st+="@{}}\n"
    f.write(st)

    # f.write("\\cline{2-%d}\n" % (Nenv+1))
    f.write("\\toprule\n")
    # f.write("\\addlinespace[-10ex]\n")

    # fstr = "&& \\multicolumn{3}{|c|}{Narrow Passage} & \\multicolumn{4}{c|}{Grasping}\\\\ \n"
    # f.write(fstr)

    # estr = "\\multicolumn{1}{c|}{} &"
    estr = "\\multicolumn{2}{>{\\centering}p{2.8cm}}{Runtime in seconds (%d run average)} & "%(Nruns)
    # estr = " && "
    for k in range(0,Nenv):
      e = env_names[k]
      estr += "\\rothead{%s}"%e
      print(estr)
      if k < Nenv-1:
        estr += " & "
    estr += " \\\\ \n"

    f.write(estr)
    f.write("\\midrule\n")

    ctr = 1
    for p in planner_names:
      pstr = str(ctr) + " & \\mbox{" + str(p)+"} & "
      ctr = ctr + 1
      for k in range(0,Nenv):
        e = env_names[k]
        bestPlanner = min(d[e],key=d[e].get)
        if p in d[e]:
          t = d[e].get(p)
          if p == bestPlanner:
            pstr += (" \\textbf{%.2f} " % t)
          else:
            pstr += (" %.2f " % t)
        else:
          pstr += " - "
        if k < Nenv-1:
          pstr += " & "
      pstr += " \\\\ \n"
      f.write(pstr)

    f.write("\\bottomrule\n")
    f.write("\end{tabulary}\n")


    f.write("\\caption{Runtime (s) of motion planner on $7$ scenarios, each \
        averaged over $10$ runs with cut-off time limit of $60$s. Entry $-$ \
        means that planner does not support compound state spaces.}\n")

    f.write("\\end{table}\n")
    f.write("\end{document}\n")
    f.close()

    os.system("pdflatex -output-directory %s %s" % (os.path.dirname(texName),
      texName))
    print("Wrote tex file to %s" % texName)

    os.system("apvlv %s" % pdfName)

if __name__ == '__main__':
    # fnames = glob.glob("../../data/benchmarks/RAL2020v1/*.xml")
    fnames = []
    fnames += glob.glob("../../data/benchmarks/RAL2020v2_Bundle/*.xml")
    fnames += glob.glob("../../data/benchmarks/RAL2020v2_QRRTstar/*.xml")
    fnames += glob.glob("../../data/benchmarks/RAL2020v2_NonBundle/*.xml")
    # fnames = glob.glob("../../data/benchmarks/*.xml")
    PlotTable(fnames)
