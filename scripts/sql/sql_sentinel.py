from sql_query import *

files = ['../data/benchmarks/2017_04_15_sentinel_pipes.db','../data/benchmarks/2017_04_15_sentinel_pipes_irreducible.db']
for fname in files:
  extractInfoFromSqlDatabase(fname,verbose=False)

