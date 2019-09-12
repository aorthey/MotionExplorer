<h1>MotionPlanningExplorerGUI</h1>

Visualize Local Minima using a local minima tree. 

As input we use
robots (in .urdf files), environments (in .tri files), and worlds (in .xml
files). 

Open Motion Planning Library (OMPL): http://ompl.kavrakilab.org/

Kris' Locomotion and Manipulation Planning Toolbox (Klamp't): http://motion.pratt.duke.edu/klampt/

<h1>Install</h1>

<h3>Dependencies</h3>

<ul>
  <li> OMPL 1.5.0 (modified version github.com/aorthey/ompl)
  <li> Klampt 0.8 (modified version github.com/aorthey/Klampt and
  github.com/aorthey/KrisLibrary)
  <li> Eigen 3.3
  <li> Lemon
  <li> Boost 1.55 or later
</ul>

<ol>
  <li> Install Script for Ubuntu 16.04 and 18.04

    ./install_script.sh
    cd build
    make planner_gui
</ol>

<h1>Use</h1>

Example run:

<ol>
  <li> 02D_manipulator
    
    cd /build
    ./planner_gui ../data/experiments_ICRA2020/02D_manipulator.xml
    
</ol>

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
      


