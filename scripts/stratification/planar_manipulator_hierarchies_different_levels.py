import matplotlib.pyplot as plt
import scipy.special as sy
import numpy as np
import itertools as it

N = {1,2,3,4,5,6,7,8}

ctr = 0
def PrintHierarchy(subset, maxT):
  global ctr
  ctr = ctr+1
  print "<hierarchy>"
  for t in subset:
    print "  <level inner_index=\"%d\" type=\"R%d\"/>"%(t,t)
  print "  <level inner_index=\"%d\" type=\"R%d\"/>"%(maxT,maxT)
  print "</hierarchy>"

for n in N:
  for k in range(1,n):
    for subset in it.combinations(range(1,n), k):
        PrintHierarchy(subset, n)
  PrintHierarchy([],n)
print "Created ",ctr," hierarchies in dimensions ",N
