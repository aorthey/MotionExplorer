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
  <li> Lemon
  <li> Boost 1.55 or later
</ul>

<ol>
  <li> Install Script for Ubuntu 16.04 and 18.04

    ./install_script.sh
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
      


