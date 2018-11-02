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

<h3>Use</h3>

GUI uses hotkeys which are defined and can be modified in settings/gui.xml

The planners and settings are defined and can be modified in
settings/planner.xml

Current important keys:
      h : help

      [before planning] 
      t : switch to next planner
      w : plan until solution found or timelimit reached
      a : plan one step (needs to be supported by planner)
      b : draw bounding box of sampling domain

      [after planning] 
      u : start/stop moving robot along found path (if any)
      


