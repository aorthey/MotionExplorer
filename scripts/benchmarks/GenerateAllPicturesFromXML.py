from ParseBenchmarkFile import *
from PlotTimeVsDimension import *
from PlotTimeVsSubspaces import *
from PlotTimePercentageVsSubspaces import *
from PlotPercentageTimeVsPlanner import *
from PlotPlannersVsTime import *

import glob, os

def GenerateAllPicturesFromXML(fname, show_=False):
  TimePerDimension(fname, show=show_)
  PlannersPerTimeHistogram(fname, show=show_)

fname = '../../data/benchmarks/07D_kuka_windshield_stratifications_2019_03_18_21:43:43.xml'
fname = '../../data/benchmarks/11D_planar_snake_twister_different_hierarchies_2019_03_18_13:59:09.xml'
fname = '../../data/benchmarks/10D_spatial_snake_twister_stratification_2019_03_18_14:51:51.xml'
fname = '../../data/benchmarks/10D_spatial_snake_twister_stratification_2019_03_18_14:51:51.xml'
fname = '../../data/benchmarks/10D_spatial_snake_twister_stratification_2019_03_20_18:49:35.xml'
fname = '../../data/benchmarks/07D_kuka_windshield_stratifications_2019_03_18_21:43:43.xml'

PlotPercentageTimeVsPlanner(fname, LAST_LEVEL = True)
sys.exit(0)

os.chdir("../../data/benchmarks/2019-03-21/")
for file in glob.glob("*.xml"):
  print file
  # PlotTimeVsDimension(file)
  # PlotPercentageVsSubspace(file)
  # PlotTimePercentageVsSubspace(file)
  PlotPercentageTimeVsPlanner(file, LAST_LEVEL = True)

#GenerateAllPicturesFromXML(fname)
#TimePerDimension(fname, show=True, START_AT_BEGINNING=True, REMOVE_PTS_START = 3, REMOVE_PTS_END = 0, EXTRAPOLATED_PTS=0)
#PlannersPerTimeHistogram(fname, show=True)

