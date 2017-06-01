from sql_query import *

files = ['../data/benchmarks/2017_05_25_humanoid_door_irreducible.db','../data/benchmarks/2017_05_25_humanoid_door_complete.db']
for fname in files:
  extractInfoFromSqlDatabase(fname,verbose=False)

#--------------------------------------------------------------------------------
#../data/benchmarks/2017_05_22_humanoid_door_irreducible.db  Runs with timelimit= 1200.0 seconds
#--------------------------------------------------------------------------------
#1 : [ control_PDST ]solved:  4 / 10  time: 940.2857861 +/- 400.227417805 (nodes: 862963.6 ) 
#2 : [ control_SST ]solved:  10 / 10  time: 133.7664131 +/- 80.6842680625 (nodes: 19482.9 ) 
#3 : [ control_KPIECE1 ]solved:  0 / 10  time: nan +/- nan  (nodes: 78812.0 )
#4 : [ control_RRT ]solved:  10 / 10  time: 118.8919848 +/- 107.333117879 (nodes: 29982.5 )

