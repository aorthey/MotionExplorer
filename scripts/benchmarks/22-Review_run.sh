cd ../../build/
make -j4 planner_standalone

OUTPUT='../data/benchmarks/22-Review/'
mkdir -p $OUTPUT

function ExecuteFile() {
  echo $1
  FOLDER=$OUTPUT$(basename $(dirname $1))
  ./planner_standalone $f
  mkdir -p $FOLDER
  mv $OUTPUT*.db $FOLDER 2>/dev/null
  mv $OUTPUT*.log $FOLDER 2>/dev/null
  echo "Created folder $FOLDER"
  OUTPUT_FOLDERS+=($FOLDER)
}

function RemoveFolders() {
  rm -rf $OUTPUT/*
}

##### Create Benchmark Files
OUTPUT_FOLDERS=()
RemoveFolders
for d in ../data/experiments/22-Review/Manifolds/*
do 
  if [[ -d $d ]]; then
    for f in $d/*
    do
      if [[ -f $f ]]; then
        ExecuteFile $f
      fi
    done;
  else
    if [[ -f $f ]]; then
      ExecuteFile $f
    fi
  fi
done;

##### Remove duplicates
OUTPUT_FOLDERS=($(echo "${OUTPUT_FOLDERS[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))

### Group all db files into one and plot the graphs
for FOLDER in "${OUTPUT_FOLDERS[@]}"; 
do 
  echo "$FOLDER" ; 
  LAST_FOLDER=$(basename $FOLDER);
  echo $LAST_FOLDER;
  BENCHMARK_FILE="$FOLDER/$LAST_FOLDER.db";
  ./../libs/ompl/scripts/ompl_benchmark_statistics.py -a $FOLDER/*.log -d $BENCHMARK_FILE
  ./../../ompl_benchmark_plotter/ompl_benchmark_plotter.py $BENCHMARK_FILE
  ITER=0
  for pdf in $FOLDER/*.pdf;
  do
    cp $pdf $OUTPUT$LAST_FOLDER$ITER.pdf
    ITER=$((ITER+1))
  done
done
