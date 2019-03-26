import matplotlib.pyplot as plt
import scipy.special as sy
import numpy as np
import itertools as it
import sys


def NumberStratifications(K_subspaces):
  k = np.arange(0,K_subspaces)
  V = sy.binom(K_subspaces-1, k)
  print k,V
  return np.sum(V)

def CumulatedNumberStratifications(K_subspaces):
  if K_subspaces == 1:
    return NumberStratifications(K_subspaces)
  else:
    s = NumberStratifications(K_subspaces) + CumulatedNumberStratifications(K_subspaces-1)
    return s

for k in range(1,9):
  print "N(k="+str(k)+")="+str(NumberStratifications(k))
for k in range(1,9):
  print "N(k="+str(k)+")="+str(CumulatedNumberStratifications(k))
sys.exit(0)

N = 4
t = np.arange(1,N+1)
V = sy.binom(N-1, t-1)
#start at 1, because we always have a single trivial stratification
ctr = 1
for k in range(1,N+1):
  for subset in it.combinations(range(1,N+1), k):
      print(subset)
      ctr = ctr+1

print "Euclidean Space R^N: number of Decompositons is given by sum_k=1^N binom(N,k)"
print V

V = np.transpose(V, axes=0)
plt.bar(t, V)
print "Dimensionality N=",N," Decompositions=",np.sum(V)

plt.xlabel('Number of Levels')
plt.ylabel('Number of Stratifications')
plt.title('Distribution of Stratifications')
plt.show()
