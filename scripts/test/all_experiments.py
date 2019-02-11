import subprocess
import os
import re
import sys
import numpy as np

fname = "/home/aorthey/git/orthoklampt/build/"

def Execute(name, N, max_time):
  times = []
  for i in range(0,N):
    try:
      out = subprocess.check_output("./planner_standalone ../data/experiments/"+name,cwd=fname, shell=True, stderr=open(os.devnull, 'wb'))
    except Exception, e:
      out = str(e.output)
    m = re.search(r'(planner time.*:).*(\d+\.\d+)',out)

    if m is None:
      times.append(max_time)
    else:
      t = float(m.group(2))
      times.append(t)
  times = np.array(times)
  sN = len(times[times < max_time])
  if sN < N:
    sys.stdout.write('*FAILED |')
  else:
    sys.stdout.write('    OK  |')

  print name,":",sN,"/",N," successful. Times: (Average:",np.mean(times), "Worst:",np.max(times), "Max:", max_time,")"
  return [sN>=N,sN,N,np.mean(times)]



print '#'*80
print "Test of 2D Rigid Bodies"
print '#'*80
Execute("02D_disk.xml", 5, 1.0)
Execute("02D_disk_bugtrap.xml", 5, 0.2)
Execute("02D_disk_narrow.xml", 5, 1.0)
Execute("02D_disk_decomposed.xml", 5, 1.0)
Execute("03D_corner.xml", 5, 0.5)
Execute("03D_misleading.xml", 5, 0.5)
Execute("03D_nonsimple.xml", 5, 1.0)
print '#'*80
