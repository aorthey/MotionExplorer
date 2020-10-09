cd ../../build/
cd ../libs/ompl/build
cmake .. 
make -j `nproc` 
sudo make install
cd ../../../build/
make -j4 planner_standalone


pwd
for f in ../data/experiments/TRO2021_FINAL/*;
do 
  echo $f
  ./planner_standalone $f
done;
./PlotTableBenchmarkFolder
