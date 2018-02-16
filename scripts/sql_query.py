import sqlite3
import numpy as np

def extractInfoFromSqlDatabase(fname,verbose=True):
  connection = sqlite3.connect(fname)
  cursor = connection.cursor()

  if verbose:
    print 80*"-"
    print "Database: ",fname
    print 80*"-"

  cursor.execute("""SELECT * FROM sqlite_master WHERE type='table';""")
  names = list(map(lambda x: x[0], cursor.description))
  if verbose:
    print "Table Columns    : ",names
    print "[Table]"

  tables = cursor.execute("""SELECT name FROM sqlite_master WHERE type='table';""").fetchall()

  for table in tables:
    tname = table[0]
    sql = """SELECT * FROM """+table[0]+""";"""
    cursor.execute(sql).fetchall()
    names = list(map(lambda x: x[0], cursor.description))
    sql = """SELECT Count(*) FROM """+tname
    Nentries =cursor.execute(sql).fetchall()[0][0]
    if verbose:
      print "  "+80*"-"
      print "  [Table:"+tname+"]"
      print "    [ColumnNames]",names
      print "      [Entries:]",Nentries

  names = cursor.execute("""SELECT name FROM plannerConfigs;""").fetchall()
  plannerids = cursor.execute("""SELECT DISTINCT plannerid FROM runs;""").fetchall()
  timelimit = np.array(cursor.execute("""SELECT timelimit FROM experiments;""").fetchall()).flatten()[0]
  runcount = np.array(cursor.execute("""SELECT runcount FROM experiments;""").fetchall()).flatten()[0]
  totaltime = np.array(cursor.execute("""SELECT totaltime FROM experiments;""").fetchall()).flatten()[0]

  print 80*"-"
  print fname," runs=",runcount,", timelimit=",timelimit,"s, totaltime=",totaltime,"s"
  print 80*"-"
  for i in range(0,len(names)):
    name = names[i][0]
    pid = plannerids[i][0]

    sql = """SELECT time FROM runs WHERE plannerid="""+str(pid)
    timessql = cursor.execute(sql).fetchall()
    times = np.array(timessql)  

    sql = """SELECT solved FROM runs WHERE plannerid="""+str(pid)
    solved = np.array(cursor.execute(sql).fetchall())
    Dsolved = solved.sum()

    #solved = sol.sum()

    sql = """SELECT solved FROM runs WHERE plannerid="""+str(pid)
    sol = np.array(cursor.execute(sql).fetchall())
    solved = sol.sum()
    runs = sol.shape[0]
    Dtimes = times
    DtimesSolved = times[times < timelimit]

    Dsolved = len(Dtimes)
    if Dsolved>0:
      Dmean = Dtimes.mean() 
      Dstd = Dtimes.std() 
    else:
      Dmean = float('nan')
      Dstd = float('nan')

    sql = """SELECT graph_states FROM runs WHERE plannerid="""+str(pid)
    nodessql = cursor.execute(sql).fetchall()
    nodes = np.array(nodessql).mean()

    #sql = """SELECT edge_count FROM runs WHERE plannerid="""+str(pid)
    #edgessql = cursor.execute(sql).fetchall()
    #edges = np.array(nodessql).mean()

    #print pid,": [",name,"]solved: ",solved,"/",runs," time:",times.mean(),"+/-",times.std()
    print pid,": [",'%30s'%name,"] solved: ",Dsolved,\
    "/",runs," time:",'%.2f'%Dmean,"+/-",'%.2f'%Dstd,\
    " (minimum time:",'%.2f'%Dtimes.min(),", maximum time:",'%.2f'%Dtimes.max(),", nodes:",nodes,")"
    print pid,": [",'%30s'%name,"] solved: ",len(DtimesSolved),\
    "/",len(DtimesSolved)," time:",'%.2f'%DtimesSolved.mean(),"+/-",'%.2f'%DtimesSolved.std(),\
    " (minimum time:",'%.2f'%DtimesSolved.min(),", maximum time:",'%.2f'%DtimesSolved.max(),\
        ", nodes:",np.array(nodessql)[times<timelimit].mean(),")"
    
  connection.close()

if __name__ == '__main__':
  fname = "../data/benchmarks/database/ompl_benchmark.db"
  extractInfoFromSqlDatabase(fname)

