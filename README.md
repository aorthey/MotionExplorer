<h3>Dependencies</h3>

  Eigen 3.3
  Boost 1.55
  OMPL 1.4.0
  Klampt 0.6 (modified version github.com/aorthey/Klampt and
  github.com/aorthey/KrisLibrary)


  sudo apt-get install g++-5 libboost1.55-all-dev libeigen3-dev

<h3>Install</h3>

  mkdir build
  cd build
  cmake ..
  make -j10
  ./planner_hierarchy ../data/experiments/06D_doubleLshape.xml
