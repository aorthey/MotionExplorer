from sql_query import *

files = ['../data/benchmarks/2017_05_18_snake_turbine_irreducible.db','../data/benchmarks/2017_05_18_snake_turbine_complete.db']
for fname in files:
  extractInfoFromSqlDatabase(fname,verbose=False)

#--------------------------------------------------------------------------------
#../data/benchmarks/ompl_benchmark.db  Runs with timelimit= 120.0 seconds
#--------------------------------------------------------------------------------
#1 : [ control_PDST ]solved:  0 / 10  time: nan +/- nan  (nodes: 130053.4 )
#2 : [ control_SST ]solved:  2 / 10  time: 111.0541391 +/- 18.1306066335  (nodes: 9605.8 )
#3 : [ control_KPIECE1 ]solved:  1 / 10  time: 109.4612071 +/- 31.8694880559  (nodes: 103360.4 )
#4 : [ control_RRT ]solved:  0 / 10  time: nan +/- nan  (nodes: 18430.2 )
#--------------------------------------------------------------------------------
#../data/benchmarks/ompl_irreducible_benchmark.db  Runs with timelimit= 120.0 seconds
#--------------------------------------------------------------------------------
#1 : [ control_PDST ]solved:  10 / 10  time: 18.3249546 +/- 19.0171781574  (nodes: 41955.6 )
#2 : [ control_SST ]solved:  4 / 10  time: 92.6527013 +/- 38.3613766654  (nodes: 20330.7 )
#3 : [ control_KPIECE1 ]solved:  10 / 10  time: 2.3396724 +/- 3.52178576075  (nodes: 15788.7 )
#4 : [ control_RRT ]solved:  5 / 10  time: 82.1045014 +/- 45.1503789101  (nodes: 33824.6 )


