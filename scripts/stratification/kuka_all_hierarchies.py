import matplotlib.pyplot as plt
import scipy.special as sy
import numpy as np
import itertools as it

N = [1,2,3,4,5,7]
N = np.array(N)

ctr = 0
def PrintHierarchy(subset, maxT):
  global ctr
  ctr = ctr+1
  print "<hierarchy>"
  for t in subset:
    print "  <level inner_index=\"%d\" type=\"R%d\"/>"%(t,t)
  if maxT==7:
    print "  <level inner_index=\"0\" type=\"R%d\"/>"%(maxT)
  else:
    print "  <level inner_index=\"%d\" type=\"R%d\"/>"%(maxT,maxT)
  print "</hierarchy>"

for n in N:
  for k in N[N<=n]:
    for subset in it.combinations(N[N<n], k):
        PrintHierarchy(subset, n)
  PrintHierarchy([],n)
print "Created ",ctr," hierarchies in dimensions ",N
