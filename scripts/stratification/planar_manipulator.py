import matplotlib.pyplot as plt
import scipy.special as sy
import numpy as np

N = 8
t = np.arange(1,N+1)
V = sy.binom(N, t)
print V

print "Euclidean Space R^N: number of Decompositons is given by sum_k=1^N binom(N,k)"

#plt.plot(t,V,"-k")
V = np.transpose(V, axes=0)
plt.bar(t, V)
print "Dimensionality N=",N," Decompositions=",np.sum(V)

plt.xlabel('Number of Levels')
plt.ylabel('Number of Decompositions')
plt.title('Dimensions over Decompositions')
plt.show()
