cd ../../build/
make -j4 planner_standalone

pwd
for f in ../data/experiments/RAL2021_FINAL/*;
do 
  echo $f
  ./planner_standalone $f
done;
./PlotTableBenchmarkFolder

# ./planner_standalone ../data/experiments/IJRR2020_FINAL/37D_shadowhand_pregrasp.xml
# ./planner_standalone ../data/experiments/IJRR2020_FINAL/30D_airport.xml
# ./planner_standalone ../data/experiments/IJRR2020_FINAL/24D_crossing_cars.xml
# ./planner_standalone ../data/experiments/IJRR2020_FINAL/21D_box_folding.xml

