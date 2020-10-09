PATH_PRIOR=`pwd`
cd ../../build/
cd ../libs/ompl/build
cmake .. 
make -j `nproc` 
sudo make install
cd ../../../build/
make -j4 planner_standalone

for f in ../data/experiments/ICRA2021_FINAL/*;
do 
  echo $latex f
  ./planner_standalone $f
done;
cd $PATH_PRIOR
./Plot2021ICRA.py
