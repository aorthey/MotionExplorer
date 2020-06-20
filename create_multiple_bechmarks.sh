#!/bin/bash

cd ~/MotionPlanningExplorerGUI/build

make -j$(nproc) planner_standalone

for filename in ../data/experiments/IJRR2020_FINAL/*.xml; do
  ./planner_standalone $filename
done