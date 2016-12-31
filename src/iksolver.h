#pragma once

//Interface to Klampt IKSolver 
class IKSolver{
  private:
    Real _tolerance;
    int _iters;
    int _verbose;
    vector<IKGoal> _problem;
    Config q_solution;
    Config q_initial;
    bool _isSolved;

  protected:
    Robot *_robot;
    RobotWorld *_world;
  public:
    //#########################################################################
    virtual string GetRobotName(){
      std::cout << "Forbidden" << std::endl;
      exit(0);
    }
    virtual vector<IKGoal> GetProblem(){
      std::cout << "Forbidden" << std::endl;
      exit(0);
    }
    //#########################################################################

    IKSolver(RobotWorld *world):
      _world(world)
    {
      this->_tolerance = 1e-3;
      this->_isSolved = false;
      this->_iters = 100;
      this->_verbose = 1;
    }

    bool solve(){
      this->_robot = _world->GetRobot(this->GetRobotName());
      std::cout << "solving" << std::endl;
      this->_problem = this->GetProblem();

      Config q_initial = this->_robot->q;

      _isSolved = SolveIK(*_robot,_problem,_tolerance,_iters,_verbose);

      Config q_solution = this->_robot->q;
      double d = (q_initial-q_solution).norm();
      std::cout << "DISTANCE IK" << d << std::endl;

      if(!_isSolved){
        std::cout << "No IK solution" << std::endl;
      }else{
        std::cout << "IK solution iters " << _iters << std::endl;
      }
      return _isSolved;
    }

    ///Set IK solution to real robot
    void SetConfigSimulatedRobot(WorldSimulation &sim)
    {
      if(!_isSolved){
        std::cout << "[ERROR] Robot cannot set to infeasible IK solution" << std::endl;
        return;
      }
      ODERobot *simrobot = sim.odesim.robot(0);
      simrobot->SetConfig(q_solution);
      std::cout << "[WARNING] Setting Simulated Robot to Config. This should be done exactly once!" << std::endl;
    }

};

