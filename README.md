<h3>Dependencies</h3>

<ul>
  <li> Eigen 3.3
  <li> Boost 1.55
  <li> OMPL 1.4.0
  <li> Klampt 0.6 (modified version github.com/aorthey/Klampt and
  github.com/aorthey/KrisLibrary)
</ul>


  sudo apt-get install g++-5 libboost1.55-all-dev libeigen3-dev

<h3>Install</h3>

      mkdir build
      cd build
      cmake ..
      make -j10
      ./planner_hierarchy ../data/experiments/06D_doubleLshape.xml
