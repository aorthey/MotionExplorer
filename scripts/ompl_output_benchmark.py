import sqlite3
import sys
import numpy as np
from sql_query import *

if __name__ == '__main__':
  Nargs = len(sys.argv)
  args = str(sys.argv)
  if Nargs>1:
    for fname in sys.argv[1:]:
      extractInfoFromSqlDatabase(fname,verbose=False)
