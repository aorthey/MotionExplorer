from ParseBenchmarkFile import *
from collections import defaultdict
import subprocess



def PlotTable(fnames):
    texName = '../../data/benchmarks/table.tex'
    pdfName = '../../data/benchmarks/table.pdf'

    d = defaultdict(dict)
    planner_names = []
    env_names = []

    for f in fnames:
        benchmark = BenchmarkAnalytica(f)
        env = benchmark.benchmark_name
        env = env.replace("_"," ")
        env_names.append(env)
        for p in benchmark.planners:
            planner_names.append(p.name)
            d[env][p.name] = p.AverageTime()

    ## unique list
    planner_names = set(planner_names)
    env_names = sorted(env_names)

    Nplanner = len(planner_names)
    Nenv = len(env_names)
    print(env_names)

    ## create tex
    f = open(texName, 'w')

    f.write("\documentclass{article}\n")
    # %\usetikzlibrary{...}% tikz package already loaded by 'tikz' option
    f.write("\\usepackage{makecell}\n")
    f.write("\\usepackage{tabularx}\n")
    f.write("\\usepackage{rotating}\n")
    f.write("\\usepackage{booktabs}\n")

    f.write("\\begin{document}\n\n")

    f.write("\\begin{table}\n")

    f.write("\\setcellgapes{2pt}\n")
    f.write("\\makegapedcells\n")
    f.write("\\renewcommand{\\cellrotangle}{90}\n")
    f.write("\\renewcommand\\theadfont{\\bfseries}\n")
    f.write("\\renewcommand\\theadset{\\linespread{1.0}}\n")
    # f.write("\\settowidth\\rotheadsize{\\theadfont graduate (\\%)}\n")

    klongest = 0
    env_name_length = 0
    for k in range(0, Nenv):
      l = len(env_names[k])
      if l > env_name_length:
        env_name_length = l
        klongest = k

    f.write("\\settowidth{\\rotheadsize}{\\theadfont %s}" % env_names[klongest])

    # f.write("\\newcolumntype{Y}{>{\\raggedleft\\arraybackslash}X}")

    f.write("\\footnotesize\\centering\n")

    # st = "\\begin{tabularx}{\\textwidth}{@{} l *{%d}{Y}| @{}}" % Nenv
    st = "\\begin{tabularx}{\\textwidth}{l|"
    for k in range(0, Nenv-1):
      st += "c|"
    st+="c}\n"
    f.write(st)

    f.write("\\hline\n")
    # f.write("\\addlinespace[-10ex]\n")

    estr = " & "
    for k in range(0,Nenv):
      e = env_names[k]
      estr += "\\rothead{%s}"%e
      print(estr)
      if k < Nenv-1:
        estr += " & "
    estr += " \\\\ \n"

    f.write(estr)
    f.write("\\hline\n")

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

    f.write("\\hline\n")
    f.write("\end{tabularx}\n")
    f.write("\\end{table}\n")
    f.write("\end{document}\n")
    f.close()

    print("Wrote tex file to %s" % texName)

if __name__ == '__main__':
    fnames = [
        "../../data/benchmarks/06D_bugtrap.xml",
        "../../data/benchmarks/06D_doubleLshape.xml",
        "../../data/benchmarks/10D_snake_outrun.xml"]
    PlotTable(fnames)
