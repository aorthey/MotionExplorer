#pragma once
#include <Interface/SimTestGUI.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <Modeling/World.h>
#include <View/ViewIK.h>

//Interface to Klampt IKSolver 
class IKSolver{
  protected:
    Real _tolerance;
    int _iters;
    int _verbose;
    vector<IKGoal> _problem;
    Config q_solution;
    Config q_initial;

    bool _isSolved;
    Robot *_robot;
    RobotWorld *_world;
  public:
    IKSolver(RobotWorld *world);
    virtual string GetRobotName() = 0;
    virtual vector<IKGoal> GetProblem() = 0;
    virtual bool solve();
    void visualize();
    ///Set IK solution to real robot
    void SetConfigSimulatedRobot(WorldSimulation &sim);
    IKGoal LinkToGoalRot( const char *linkName, double x, double y, double z, Matrix3 &rotation);
    IKGoal LinkToGoal( const char *linkName, double x, double y, double z);
};

