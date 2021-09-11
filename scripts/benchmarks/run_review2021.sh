CUR_PATH=`pwd`
cd ../../build/
# make -j4 planner_standalone
# for d in ../data/experiments/21-Review/*
for d in ../data/experiments/21-Review/manifolds/
do 
  if [[ -d $d ]]; then
    for f in $d/*
    do
      if [[ -f $f ]]; then
        echo $f
        ./planner_standalone $f
      fi
    done;
  else
    if [[ -f $f ]]; then
      echo $f
      ./planner_standalone $f
    fi
  fi
done;
cd $CUR_PATH
pwd
./PlotTableReview2021.py
