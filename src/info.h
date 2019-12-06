#pragma once

#include <stdio.h>
#include <Modeling/Paths.h>
#include <Modeling/Terrain.h>
#include <Modeling/MultiPath.h>
#include <KrisLibrary/planning/Path.h>
#include <KrisLibrary/planning/KinodynamicPath.h>
#include <Simulation/WorldSimulation.h>

class Info
{
  public:
    
    Info();
    void operator()(const MilestonePath &path);
    void operator()(const KinodynamicMilestonePath &path);
    void operator()(const MultiPath &path);
    void operator()(RobotWorld *world);
    void operator()(Robot *robot);
    void operator()(WorldSimulation *sim);
    void operator()(const Terrain *terrain);
};

