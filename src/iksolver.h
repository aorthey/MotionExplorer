#pragma once

//Interface to Klampt IKSolver 
class IKSolver{
  private:
    Real _tolerance;
    int _iters;
    int _verbose;
    vector<IKGoal> _problem;

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
      this->_iters = 100;
      this->_verbose = 1;
    }

    bool solve(){
      this->_robot = _world->GetRobot(this->GetRobotName());
      std::cout << "solving" << std::endl;
      this->_problem = this->GetProblem();
      bool res = SolveIK(*_robot,_problem,_tolerance,_iters,_verbose);
      if(!res){
        std::cout << "No IK solution" << std::endl;
      }else{
        std::cout << "IK solution iters " << _iters << std::endl;
      }
      return res;
    }
};

