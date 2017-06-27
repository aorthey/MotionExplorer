#pragma once

#include <Planning/PlannerSettings.h>
#include <Planning/RobotCSpace.h>
#include <KrisLibrary/math/random.h>

class PlannerWorkspaceApproximation{
  private:
    Vector3 init;
    Vector3 goal;
    SingleRobotCSpace *inner;
    SingleRobotCSpace *outer;
    double radius;
  public:

    std::vector<Vector3> tree;
    double inner_radius;
    double outer_radius;

    PlannerWorkspaceApproximation(Vector3 &init_, Vector3 &goal_, SingleRobotCSpace *inner_, SingleRobotCSpace *outer_);
    void solve();
    bool IsFeasible(Vector3 &qq);
    bool isCollisionFree(SingleRobotCSpace *space, Config q);
    double GetRadiusFromRobot( Robot *r );

};

