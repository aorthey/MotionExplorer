<h1>MotionExplorer</h1>
<p align="middle">
  <img src="https://github.com/aorthey/MotionExplorer/blob/master/data/images/airplane.png" width="280" />
  <img src="https://github.com/aorthey/MotionExplorer/blob/master/data/images/PR2.png" width="280" />
  <img src="https://github.com/aorthey/MotionExplorer/blob/master/data/images/drone.png" width="280" />
</p>


Visualize Local Minima using a local minima tree. See <a href="https://arxiv.org/pdf/1909.05035.pdf">paper</a> for details.

As input we use
robots (in .urdf files) and objects/environments (in .tri or .stl files).

This framework combines two existing frameworks (OMPL [1] and Klamp't [2]) and adds local-minima visualization plus easier control through vim-like shortcuts.

[1] Open Motion Planning Library (OMPL): http://ompl.kavrakilab.org/

[2] Kris' Locomotion and Manipulation Planning Toolbox (Klamp't): http://motion.pratt.duke.edu/klampt/

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

Please make sure you have a github.com account AND ssh access from your workstation. See information here: https://help.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh
<ol>
  <li> Install Script for Ubuntu 16.04 and 18.04 (requires SSH access)
    
    git clone git@github.com:aorthey/MotionPlanningExplorerGUI.git
    cd MotionPlanningExplorerGUI
    ./install_script.sh
    
</ol>

<h1>Use</h1>

Examples

<ol>
  <li> 02D_manipulator
    
    ./planner_gui ../data/experiments/02D_manipulator.xml
    
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
      1 : show roadmap vertices (default: off)
      2 : show roadmap edges (default: off)
      3 : show paths (default: on)
      4 : display local-minima tree (default: on)
      5 : display planning information (default: on)
      
When compiling, we often need to compile OMPL and MotionExplorer plus do some debugging with GDB. To simplify this workflow, here are some shortcuts you can copy to your .bashrc (first build OMPL, then build MotionExplorer, then execute planner_gui with GDB)

    gdbrun (){
      gdb -q -ex 'set confirm off' -ex 'run' --args $@
    }
    makerunarg (){
      make -j5 $1 && gdbrun $@
    }
    alias cdm='cd ${HOME}/git/MotionPlanningExplorerGUI'
    alias cdmb='cdm && cd build'
    alias explorerGUI='cdmb && cd ../libs/ompl/build && cmake .. && sudo make -j4 install && cdmb && makerunarg planner_gui'
