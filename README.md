<h2>Dependencies</h2>

<ul>
  <li> OMPL 1.4.0
  <li> Klampt 0.6 (modified version github.com/aorthey/Klampt and
  github.com/aorthey/KrisLibrary)
  <li> Eigen 3.3
  <li> Boost 1.55 or later
</ul>

<h3>Ubuntu 16.04</h3>

      sudo apt-get install g++-5 libboost1.55-all-dev libeigen3-dev libassimp-dev libflann-dev liburdfdom-tools libccd-dev

<h3>Ubuntu 18.04</h3>

      sudo apt-get install g++-5 libboost1.65-all-dev libeigen3-dev libassimp-dev libflann-dev liburdfdom-tools libccd-dev libqhull-dev

<h3>Install Klampt 0.6 (all Ubuntu versions)</h3>

      sudo apt-get install g++ cmake git libboost-system-dev libboost-thread-dev freeglut3 freeglut3-dev libglpk-dev python-dev python-opengl libxmu-dev libxi-dev libqt4-dev
      git clone git@github.com:aorthey/Klampt.git
      cd Klampt/Library
      make unpack-deps
      rm -rf KrisLibrary
      git clone git@github.com:aorthey/KrisLibrary.git
      make deps
      cd ..
      cmake .
      make



<h2>Install</h2>

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


      [General Control]
      Left Mouse Click: Move Camera around Focal Point
      Ctrl + Left Mouse Click: Move Focal Point
      Shift + Left Mouse Click: Zoom In/Out from Focal Point
      e: on/off show edges of objects
      f: on/off show faces of objects
      h : help

      [before planning] 
      t : switch to next planner
      w : plan until solution found or timelimit reached
      a : plan one step (needs to be supported by planner)
      b : draw bounding box of sampling domain

      [after planning] 
      u : start/stop moving robot along found path (if any)
      


