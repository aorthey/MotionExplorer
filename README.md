  sudo apt-get install g++-5

  mkdir build
  cd build
  cmake ..
  make -j10
  ./planner_hierarchy ../data/experiments/06D_doubleLshape.xml
