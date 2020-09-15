import glob
from PlotTable import *


fnames = glob.glob("../../data/benchmarks/*xml")

PlotTable(fnames)
