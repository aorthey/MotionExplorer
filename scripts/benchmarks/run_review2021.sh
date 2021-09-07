cd ../../build/
# make -j4 planner_standalone
pwd
for d in ../data/experiments/21-Review/*
do 
  if [[ -d $d ]]; then
    for f in $d/*
    do
      if [[ -f $f ]]; then
        echo $f
        ./planner_standalone $f
      fi
    done;
  fi
  # ./planner_standalone $f
done;
./PlotTableBenchmarkFolder
