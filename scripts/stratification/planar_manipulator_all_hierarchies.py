import matplotlib.pyplot as plt
import scipy.special as sy
import numpy as np
import itertools as it

N = 5

def PrintHierarchy(subset):
  print "<hierarchy>"
  for t in subset:
    print "<level inner_index=\"%d\" type=\"R%d\"/>"%(t,t)
  print "<level inner_index=\"0\" type=\"R5\"/>"
  print "</hierarchy>"

for k in range(1,N):
  for subset in it.combinations(range(1,N), k):
      PrintHierarchy(subset)

PrintHierarchy([])
    # <hierarchy>
    #   <level inner_index="1" type="R1"/>
    #   <level inner_index="2" type="R2"/>
    #   <level inner_index="3" type="R3"/>
    #   <level inner_index="4" type="R4"/>
    #   <level inner_index="0" type="R5"/>
    # </hierarchy>
