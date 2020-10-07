from ParseBenchmarkFile import *
from collections import defaultdict
import subprocess
import glob
import re

def PlotTable(fnames):
    texName = '../../data/benchmarks/table.tex'
    pdfName = '../../data/benchmarks/table.pdf'

    f = fnames[0]
    benchmark = BenchmarkAnalytica(f)
    runtime = []
    env = ""
    pname = ""
    averagetime = 0.0
    for p in benchmark.planners:
      if p.name == "QMPStar":
      # if p.name == "QMP":
        pname = p.name
        pname = pname.replace("#","\#")
        pname = re.sub('[Ss]tar', '*', pname)
        env = benchmark.benchmark_name
        env = env.replace("_"," ")
        runtime = p.run_time
        averagetime = p.AverageTime()

    N = len(runtime)

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
    f.write("\\newcolumntype{Y}{>{\\raggedleft\\arraybackslash}X}\n")
    # f.write("\\newcolumntype{C}[1]{>{\\centering\\arraybackslash}p{#1}}\n")

    f.write("\\footnotesize\\centering\n")
    f.write("\\renewcommand{\\arraystretch}{1.2}\n")
    f.write("\\setlength\\tabcolsep{3pt}\n")

    st = "\\begin{tabulary}{\\linewidth}{@{}L"
    for k in range(0, N):
      st += "C"
    st+="@{}}\n"
    f.write(st)

    f.write("\\toprule\n")
    fstr = "\\multicolumn{%d}{c}{Performance of %s on %s}\\\\ \n"%((N+1), pname, env)
    f.write(fstr)

    f.write("\\midrule\n")
    estr = " Run & "
    for k in range(0,N):
      estr += "%d" % k
      if k < N-1:
        estr += " & "
    estr += " \\\\ \n"

    f.write(estr)

    f.write("\\midrule\n")
    estr = " Time (s) & "
    for k in range(0,N):
      estr += "%.2f" % runtime[k]
      if k < N-1:
        estr += " & "
    estr += " \\\\ \n"
    f.write(estr)

    f.write("\\bottomrule\n")
    f.write("\end{tabulary}\n")


    f.write("\\caption{Runtime (s) of %s for each run on the %s scenario.  The average runtime over all runs is %.2fs.}\n" %(pname, env, averagetime))

    f.write("\\end{table}\n")
    f.write("\end{document}\n")
    f.close()

    os.system("pdflatex -output-directory %s %s" % (os.path.dirname(texName),
      texName))
    print("Wrote tex file to %s" % texName)

    os.system("apvlv %s" % pdfName)

if __name__ == '__main__':
    # fnames = glob.glob("../../data/benchmarks/RAL2020v2_Bundle/37D_ShadowHand_Scissor_2020_09_12_09:59:24.xml")
    fnames = glob.glob("../../data/benchmarks/RAL2020v2_Bundle/37D_ShadowHand_Metal_2020_09_12_09:31:05.xml")
    PlotTable(fnames)
