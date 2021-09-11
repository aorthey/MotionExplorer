CUR_PATH=`pwd`
cd ../../build/
make -j4 planner_standalone
# make -j4 planner_standalone
# for d in ../data/experiments/21-Review/*
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
  else
    if [[ -f $f ]]; then
      echo $f
      ./planner_standalone $f
    fi
  fi
  sleep 2
done;
