
from xml.dom.minidom import parse
doc = parse("../build/benchmark_2018_09_27_16:55:30.xml")

name = doc.getElementsByTagName("name")[0]
print name

planners = doc.getElementsByTagName("planner")
for planner in planners:
  name = planner.getElementsByTagName("name")[0].firstChild.data
  print "name:", name
  runs = planner.getElementsByTagName("run")
  for run in runs:
    sid = run.getAttribute("number")
    time = planner.getElementsByTagName("time")[0].firstChild.data
    print "run ",sid," time ",time


