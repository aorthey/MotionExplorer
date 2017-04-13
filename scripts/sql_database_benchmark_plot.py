import sqlite3
import numpy as np

fname = "../data/benchmarks/2017_04_11_irreduciblepipes.db"

connection = sqlite3.connect(fname)
cursor = connection.cursor()

print 80*"-"
print "Database: ",fname
print 80*"-"

cursor.execute("""SELECT * FROM sqlite_master WHERE type='table';""")
names = list(map(lambda x: x[0], cursor.description))
print "Table Columns    : ",names

print "[Table]"

tables = cursor.execute("""SELECT name FROM sqlite_master WHERE type='table';""").fetchall()

for table in tables:
  tname = table[0]
  print "  "+80*"-"
  print "  [Table:"+tname+"]"
  sql = """SELECT * FROM """+table[0]+""";"""
  cursor.execute(sql).fetchall()
  names = list(map(lambda x: x[0], cursor.description))
  print "    [ColumnNames]",names
  sql = """SELECT Count(*) FROM """+tname
  Nentries =cursor.execute(sql).fetchall()[0][0]
  print "      [Entries:]",Nentries


#--------------------------------------------------------------------------------
#  [Table:plannerConfigs]
#    [ColumnNames] ['id', 'name', 'settings']
#      [Entries:] 4

#    [ColumnNames] ['id', 'experimentid', 'plannerid', 'approximate_solution',
#'correct_solution', 'graph_motions', 'graph_states', 'memory',
#'solution_clearance', 'solution_difference', 'solution_length',
#'solution_segments', 'solved', 'status', 'time', 'valid_segment_fraction']

names = cursor.execute("""SELECT name FROM plannerConfigs;""").fetchall()

plannerids = cursor.execute("""SELECT DISTINCT plannerid FROM runs;""").fetchall()

timelimit = 1200

print 80*"-"
print "Solution Planner Runs with timelimit=",timelimit,"seconds"
print 80*"-"
for i in range(0,len(names)):
  name = names[i][0]
  pid = plannerids[i][0]
  sql = """SELECT time FROM runs WHERE plannerid="""+str(pid)
  timessql = cursor.execute(sql).fetchall()

  times = np.array(timessql)  
  sql = """SELECT correct_solution FROM runs WHERE plannerid="""+str(pid)
  sol = np.array(cursor.execute(sql).fetchall())
  solved = sol.sum()
  runs = sol.shape[0]

  Dtimes = times[times < timelimit]
  Dsolved = len(Dtimes)
  Dmean = Dtimes.mean() 
  Dstd = Dtimes.std() 

  #print pid,": [",name,"]solved: ",solved,"/",runs," time:",times.mean(),"+/-",times.std()
  print pid,": [",name,"]solved: ",Dsolved,"/",runs," time:",Dtimes.mean(),"+/-",Dtimes.std()
  


#times = cursor.execute("""SELECT time FROM runs WHERE plannerid=1""").fetchall()
#for time in times:
  #print time

connection.close()



