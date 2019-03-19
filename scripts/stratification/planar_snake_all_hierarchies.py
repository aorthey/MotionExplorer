import matplotlib.pyplot as plt
import scipy.special as sy
import numpy as np
import itertools as it

N = [1,2,3,4,5,6]
ctr = 0

prefix="  "

def PrintLevel(t):
  lprefix = 3*prefix
  if t==1:
    print lprefix+"<level inner_index=\"%d\" type=\"R2\"/>"%(t)
  elif t==2:
    print lprefix+"<level inner_index=\"%d\" type=\"SE2\"/>"%(t)
  elif t==N[-1]:
    print lprefix+"<level inner_index=\"0\" type=\"SE2RN\"/>"
  else:
    print lprefix+"<level inner_index=\"%d\" type=\"SE2RN\"/>"%(t)

def PrintHierarchy(subset, maxT):
  global ctr
  ctr = ctr+1
  print 2*prefix+"<hierarchy>"
  for t in subset:
    PrintLevel(t)
  PrintLevel(maxT)
  print 2*prefix+"</hierarchy>"

for n in N:
  for k in range(1,n):
    for subset in it.combinations(range(1,n), k):
        PrintHierarchy(subset, n)
  PrintHierarchy([],n)
print "Created ",ctr," hierarchies in dimensions ",N
