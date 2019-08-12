from ParseBenchmarkFile import *
from PlotTimeVsDimension import *
from PlotTimeVsSubspaces import *
from PlotTimePercentageVsSubspaces import *
from PlotPercentageTimeVsPlanner import *
from PlotPlannersVsTime import *
from PlotCostVsSubspace import *

import glob, os

## 10D Spatial Free Floating
fname = '../../data/benchmarks/10D_spatial_snake_twister_stratification_2019_03_18_14:51:51.xml'
PlotTimeVsDimension(fname, START_AT_BEGINNING=True, EXTRAPOLATED_PTS=0)
PlotPlannersVsTime(fname, MARK_TRIVIAL=True)

# ## 11D Planar Free Floating
fname = '../../data/benchmarks/11D_planar_snake_twister_different_hierarchies_2019_03_18_13:59:09.xml'
PlotTimeVsDimension(fname, START_AT_BEGINNING=False, EXTRAPOLATED_PTS=1)
PlotPlannersVsTime(fname, MARK_TRIVIAL=True)

# ## 15D Planar Fixed Base
fname = '../../data/benchmarks/15D_planar_manipulator_different_dimensions_2019_03_16_14:55:09.xml'
PlotTimeVsDimension(fname, START_AT_BEGINNING=False, EXTRAPOLATED_PTS=1)
PlotPlannersVsTime(fname, MARK_TRIVIAL=True)

# ## 7D Spatial Fixed Base
fname = '../../data/benchmarks/07D_kuka_windshield_stratifications_2019_03_18_21:43:43.xml'
PlotTimeVsDimension(fname, START_AT_BEGINNING=True, EXTRAPOLATED_PTS=0, REMOVE_PTS_START=3)
# PlotTimeVsDimension(fname, suffix="2", START_AT_BEGINNING=True,
#     EXTRAPOLATED_PTS=0, REMOVE_PTS_START=1, REMOVE_PTS_END=1)
PlotPlannersVsTime(fname, MARK_TRIVIAL=True)

# os.chdir("../../data/benchmarks/")
# for file in glob.glob("*.xml"):
#   print(file)
#   # PlotCostVsSubspace(file)
#   PlotTimeVsDimension(file)
#   # PlotPercentageVsSubspace(file)
#   # PlotTimePercentageVsSubspace(file)
#   # PlotPercentageTimeVsPlanner(file, LAST_LEVEL = True)

#TimePerDimension(fname, show=True, START_AT_BEGINNING=True, REMOVE_PTS_START = 3, REMOVE_PTS_END = 0, EXTRAPOLATED_PTS=0)
#PlannersPerTimeHistogram(fname, show=True)

