<h1>MotionExplorer</h1>

This is an experimental repository, which I use as a research platform to investigate <a href="https://sites.google.com/view/multimodaloptimization">multimodal optimization</a> and <a href="https://sites.google.com/view/multilevelmotionplanning">multilevel abstractions</a> in robot motion planning. You can find relevant papers also on my website <a href="https://aorthey.de">aorthey.de</a>. 

If you want to use any of this code in your own project, it is recommended that you write me an email (see <a href="https://aorthey.de">website</a>) or open an Issue.

<img src="https://user-images.githubusercontent.com/1220541/120098502-95c45980-c136-11eb-9953-90aa0183c2ae.gif" width="200"  height="200"/><img src="https://user-images.githubusercontent.com/1220541/120098586-09666680-c137-11eb-9284-0d30e7342551.png" width="200" height="200"/><img src="https://user-images.githubusercontent.com/1220541/120098587-0b302a00-c137-11eb-95ec-8a517cde5114.png" width="200" height="200"/>

<img src="https://user-images.githubusercontent.com/1220541/120098608-31ee6080-c137-11eb-8e54-91a6ba1f6964.png" width="200"  height="200"/><img src="https://user-images.githubusercontent.com/1220541/120098620-3b77c880-c137-11eb-9041-a5f157717b08.png" width="200"  height="200"/>

<h1>Install</h1>

<h3>Dependencies</h3>

This framework combines two existing frameworks (OMPL [1] and Klamp't [2]) and adds easier control through vim-like shortcuts.

[1] Open Motion Planning Library (OMPL): http://ompl.kavrakilab.org/

[2] Kris' Locomotion and Manipulation Planning Toolbox (Klamp't): http://motion.pratt.duke.edu/klampt/

<ul>
  <li> OMPL 1.5.0 (modified version github.com/aorthey/ompl)
  <li> Klampt 0.8 (modified version github.com/aorthey/Klampt and
  github.com/aorthey/KrisLibrary)
  <li> Eigen 3.3
  <li> Boost 1.55 or later
</ul>

Please make sure you have a github.com account AND ssh access from your workstation. See information here: https://help.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh
<ol>
  <li> Install Script for Ubuntu 16.04 and 18.04 (requires SSH access)
    
    git clone git@github.com:aorthey/MotionExplorer.git
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
