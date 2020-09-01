import sqlite3

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

connection.close()

