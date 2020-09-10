from ParseBenchmarkFile import *
from collections import defaultdict
import subprocess
import re



def PlotTable(fnames):
    texName = '../../data/benchmarks/table.tex'
    pdfName = '../../data/benchmarks/table.pdf'

    fnames = sorted(fnames)

    d = defaultdict(dict)
    planner_names_tmp = []
    env_names = []

    for f in fnames:
        benchmark = BenchmarkAnalytica(f)
        env = benchmark.benchmark_name
        env = env.replace("_"," ")
        env_names.append(env)
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

    f.write("\\begin{document}\n\n")

    f.write("\\begin{table}[h!]\n")
    f.write("\\centering\n")

    # f.write("\\setcellgapes{2pt}\n")
    # f.write("\\makegapedcells\n")
    f.write("\\renewcommand{\\cellrotangle}{90}\n")
    f.write("\\renewcommand\\theadfont{\\bfseries}\n")
    #f.write("\\renewcommand{\\tabcolsep}{1pt}\n")


    # f.write("\\renewcommand\\theadset{\\linespread{1.0}}\n")
    # f.write("\\settowidth\\rotheadsize{\\theadfont graduate (\\%)}\n")

    klongest = 0
    env_name_length = 0
    for k in range(0, Nenv):
      l = len(env_names[k])
      if l > env_name_length:
        env_name_length = l
        klongest = k

    f.write("\\settowidth{\\rotheadsize}{\\theadfont %s}" % (env_names[klongest]))

    f.write("\\newcolumntype{Y}{>{\\raggedleft\\arraybackslash}X}")

    f.write("\\footnotesize\\centering\n")
    f.write("\\renewcommand{\\arraystretch}{1.2}\n")


    # st = "\\begin{tabularx}{\\textwidth}{@{} l *{%d}{Y}| @{}}" % Nenv
    st = "\\begin{tabulary}{\\linewidth}{@{}L"
    for k in range(0, Nenv):
      st += "C"
    st+="@{}}\n"
    f.write(st)

    # f.write("\\cline{2-%d}\n" % (Nenv+1))
    f.write("\\toprule\n")
    # f.write("\\addlinespace[-10ex]\n")

    # estr = "\\multicolumn{1}{c|}{} &"
    estr = " & "
    for k in range(0,Nenv):
      e = env_names[k]
      estr += "\\rothead{%s}"%e
      print(estr)
      if k < Nenv-1:
        estr += " & "
    estr += " \\\\ \n"

    f.write(estr)
    f.write("\\midrule\n")

    for p in planner_names:
      pstr = str(p)+" & "
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
    f.write("\\end{table}\n")
    f.write("\end{document}\n")
    f.close()

    os.system("pdflatex -output-directory %s %s" % (os.path.dirname(texName),
      texName))
    print("Wrote tex file to %s" % texName)

    os.system("apvlv %s" % pdfName)

if __name__ == '__main__':
    fnames = [
        "../../data/benchmarks/06D_bugtrap.xml",
        "../../data/benchmarks/06D_doubleLshape.xml",
        "../../data/benchmarks/10D_snake_outrun.xml"]
    PlotTable(fnames)
