#pragma once
#include <Contact/Grasp.h> //class Grasp
#include <Planning/ContactCSpace.h>
#include "iksolver.h"

class IKSolverGrasp: public IKSolver
{

  protected:
    int objectid;

  public:

    IKSolverGrasp(RobotWorld *world):
      IKSolver(world)
    {
    }

    virtual string GetRobotName() = 0;
    virtual vector<IKGoal> GetProblem() = 0;
    virtual bool solve(){
      this->_robot = _world->GetRobot(this->GetRobotName());
      //###########################################################################
      //Grasping OBJECT
      //
      //  ContactCSpace is a description of a specific contact submanifold, where
      //  certain points are in contact. Sampling this Cspace corresponds to
      //  sampling IK confs
      //
      //  Grasp worldGrasp: we need to define the IK constraints (i.e. which frames
      //  are in contact at which position. Also the object id is necessary to
      //  disable collision checking between links in contact and object)
      //
      //  Having done that, we can freely sample around the IK space to find a
      //  feasible configuration. This corresponds to cspace.Sample(Config&) and
      //  cspace.Isfeasible(Config)
      //
      //###########################################################################

      Grasp worldGrasp();
      int irobot = 0;
      WorldPlannerSettings settings;
      settings.InitializeDefault(*_world);
      //SmartPointer<SingleRobotCSpace> freeSpace = SingleRobotCSpace(*_world,irobot,&settings);
      SingleRobotCSpace freeSpace = SingleRobotCSpace(*_world,irobot,&settings);

      // The contact cspace needs to be initiziliazed with a free CSpace, and also
      // with the constraints of contacts + which dofs we like to fix. This defines
      // the contact submanifold on which we like to sample for IK solutions)
      ContactCSpace cspace(freeSpace);

      Config out;
      this->_problem = this->GetProblem();
      //(1) we need to add the constraints of IK here
      for(size_t i=0;i<this->_problem.size();i++) {
        cspace.AddContact(this->_problem[i]);
      }
      //(2) which dofs should be fixed
      //cspace.fixedDofs = worldGrasp.fixedDofs;
      //cspace.fixedValues = worldGrasp.fixedValues;
      //int objectID = world->RigidObjectID(iobject);

      //////(3) which robot links can be in contact with the graspable object?
      ////for(size_t i=0;i<worldGrasp.constraints.size();i++) {
      ////  int k=worldGrasp.constraints[i].link;
      ////  cspace.ignoreCollisions.push_back(pair<int,int>(objectid,world.RobotLinkID(irobot,k)));
      ////}
      ////for(size_t i=0;i<worldGrasp.fixedDofs.size();i++) {
      ////  int k=worldGrasp.fixedDofs[i];
      ////  cspace.ignoreCollisions.push_back(pair<int,int>(objectid,world.RobotLinkID(irobot,k)));
      ////}

      _isSolved = SolveIK(*_robot,_problem,_tolerance,_iters,_verbose);
      /////(4) draw #iters samples from the contact submanifold using IK solver
      ///std::cout << "sample" << std::endl;
      ///int iters = 1000;
      ///while(iters-- > 0 ){
      ///  cspace.Sample(out);
      ///  if(cspace.IsFeasible(out)){
      ///    std::cout << "found feasible grasp" << std::endl;
      ///    _isSolved=true;
      ///    break;
      ///  }
      ///}
      ///std::cout << "Sampling Contact Submanifold took " << iters << " iterations" << std::endl;
      ///_isSolved=false;
      return _isSolved;

    }

};

