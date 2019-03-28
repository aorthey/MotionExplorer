<h1>Orthoklampt</h1>

An experimental motion planning framework, in which new planning algorithms can
quickly be developed and be benchmarked against other algorithms. As input we use
robots (in .urdf files), environments (in .tri files), and worlds (in .xml
files). 

The framework can fully be controlled by using hotkeys (in a vim-like fashion),
so that we can quickly test out new algorithm, either by letting them run until
a solution is found, or by letting them run for a single iteration.

The framework is build upon two open-source projects: First the open motion
planning library (OMPL), developed at Rice University, which provides implementations of many planning
algorithms. Second, the Klamp't library, developed at Duke University, which
provides a dynamical simulator, especially known for its realistic contact-point computations. 

More about those projects you can find here:

Open Motion Planning Library (OMPL): http://ompl.kavrakilab.org/

Kris' Locomotion and Manipulation Planning Toolbox (Klamp't): http://motion.pratt.duke.edu/klampt/

<h1>Install</h1>

<h3>Dependencies</h3>

<ul>
  <li> OMPL 1.4.0
  <li> Klampt 0.6 (modified version github.com/aorthey/Klampt and
  github.com/aorthey/KrisLibrary)
  <li> Eigen 3.3
  <li> Boost 1.55 or later
</ul>

For Ubuntu 16.04 and 18.04 (assumes that you have a github account and added your ssh key to github)

<ol>
  <li> Install Dependencies <b>Ubuntu 16.04 and 18.04</b>

      sudo apt-get install g++-5 cmake git libboost-system-dev libboost-thread-dev freeglut3 freeglut3-dev libglpk-dev python-dev python-opengl libxmu-dev libxi-dev libqt4-dev libeigen3-dev libassimp-dev libflann-dev liburdfdom-tools libccd-dev libqhull-dev
      
<b>On Ubuntu 16.04</b>

      sudo apt-get install libboost1.58-all-dev

<b>On Ubuntu 18.04</b>

      sudo apt-get install libboost1.65-all-dev
            
<li> Install OMPL (use install script on http://ompl.kavrakilab.org/download.html)
 <li> Install Klampt 0.6 (all Ubuntu versions)
  
      cd ~
      mkdir -p git
      cd git
      git clone git@github.com:aorthey/Klampt.git
      cd Klampt/Library
      make unpack-deps
      rm -rf KrisLibrary
      git clone git@github.com:aorthey/KrisLibrary.git
      make deps
      cd ..
      cmake .
      make

<li>Install Software

      cd ~/git
      git clone git@github.com:aorthey/orthoklampt.git
      mkdir build
      cd build
      cmake ..
      make -j10
      ./planner_gui ../data/experiments/15D_planar_manipulator.xml
</ol>


<h1>Use</h1>

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
      V: save viewpoint
      v: load viewpoint
      h: help

      [before planning] 
      t : switch to next planner
      w : plan until solution found or timelimit reached
      q : plan one step (needs to be supported by planner)
      b : draw bounding box of sampling domain

      [after planning] 
      u : start/stop moving robot along found path (if any)
      


