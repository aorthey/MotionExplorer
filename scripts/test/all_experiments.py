import subprocess
import os
import re
import numpy as np

fname = "/home/aorthey/git/orthoklampt/build/"
#out = subprocess.check_output(["cd", fname])
out = subprocess.check_output(["ls", "-la"],cwd=fname)

#print out
#out = subprocess.check_output(["makerunarg", "planner_standalone", "../data/experiments/03D_corner.xml"],cwd=fname, shell=True)
def Execute(name, N, max_time):
  times = []
  for i in range(0,N):
    try:
      out = subprocess.check_output("./planner_standalone ../data/experiments/"+name,cwd=fname, shell=True, stderr=open(os.devnull, 'wb'))
    except Exception, e:
      out = str(e.output)
    m = re.search(r'(planner time.*:).*(\d+\.\d+)',out)
    t = float(m.group(2))
    times.append(t)
    #print "time:",m.group(2)

  times = np.array(times)
  sN = len(times[times < max_time])
  print name,":",sN,"/",N," successful. Average time: ",np.mean(times), ", Worst time: ",np.max(times)


  
#Execute("02D_disk.xml", 5, 1.0)
Execute("03D_corner.xml", 5, 0.5)
Execute("03D_misleading.xml", 5, 0.5)
