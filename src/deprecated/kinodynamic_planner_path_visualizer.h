#pragma once

#include <Modeling/World.h>
#include <Simulation/WorldSimulation.h>
#include <KrisLibrary/planning/KinodynamicPath.h>

class KinodynamicPlannerPathVisualizer{
  public:
    KinodynamicPlannerPathVisualizer(RobotWorld *world, WorldSimulation *sim);
    std::vector<KinodynamicMilestonePath> GetPathLoops(Config& p_init);
    std::vector<KinodynamicMilestonePath> GetPathBouquet(Config& p_init);
  private:
    std::vector<KinodynamicMilestonePath> GetPaths( Config&, std::vector<double>, std::vector<double>, int, double);
    RobotWorld *_world;
    int _irobot;
    WorldSimulation *_sim;
};
