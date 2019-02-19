import subprocess
import os
import re
import sys
import numpy as np
import mmap


dir_name = "/home/aorthey/git/orthoklampt/build/"

def Execute(name, N):
  times = []
  fname = "../data/experiments/"+name
  f = open(dir_name+fname)
  s = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
  re_pattern = re.compile('maxplanningtime>([-+]?\d*\.?\d+)')
  re_match = re_pattern.search(s)
  if re_match:
    max_time = float(re_match.group(1))
  else:
    print "Could not find maxplanningtime in file",fname
    print re_match
    sys.exit(0)

  for i in range(0,N):
    try:
      out = subprocess.check_output("./planner_standalone "+fname,cwd=dir_name, shell=True, stderr=open(os.devnull, 'wb'))
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
Execute("02D_disk.xml", 5)
Execute("02D_disk_bugtrap.xml", 5)
Execute("02D_disk_narrow.xml", 5)
Execute("02D_disk_decomposed.xml", 5)
Execute("03D_corner.xml", 5)
Execute("03D_misleading.xml", 5)
Execute("03D_nonsimple.xml", 5)
print '#'*80
