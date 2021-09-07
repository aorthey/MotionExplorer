PATH_PRIOR=`pwd`
cd ../../build/
cd ../libs/ompl/build
cmake .. 
make -j `nproc` 
sudo make install
cd ../../../build/
make -j4 planner_standalone

./planner_standalone ../data/experiments/21-MultiModal/02D_kleinbottle.xml
# ./planner_standalone ../data/experiments/21-MultiModal/02D_mobius_strip.xml
# ./planner_standalone ../data/experiments/21-MultiModal/02D_torus.xml
# ./planner_standalone ../data/experiments/21-MultiModal/02D_sphere.xml
cd $PATH_PRIOR
./Plot2021RSS.py
