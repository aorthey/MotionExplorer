import glob
from PlotTable import *


fnames = glob.glob("../../data/benchmarks/Analyzer/*.xml")

PlotTable(fnames)
