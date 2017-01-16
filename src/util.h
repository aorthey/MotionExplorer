#pragma once

namespace util {
  void SetSimulatedRobot( Robot *robot, WorldSimulation &sim, Config &q)
  {
    robot->UpdateConfig(q);
    int robotidx = 0;
    ODERobot *simrobot = sim.odesim.robot(robotidx);
    simrobot->SetConfig(q);
  }
} // namespace util

