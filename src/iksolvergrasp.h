#pragma once
#include <Contact/Grasp.h> //class Grasp
#include <Planning/ContactCSpace.h>
#include <KrisLibrary/geometry/AnyGeometry.h>
#include "iksolver.h"

class IKSolverGrasp: public IKSolver
{

  protected:
    int objectid;
    vector<Real> fixedDofValues;
    vector<int> fixedDofs;

  public:

    IKSolverGrasp(RobotWorld *world):
      IKSolver(world)
    {
    }

    virtual string GetRobotName() = 0;
    virtual vector<IKGoal> GetProblem() = 0;
    virtual vector<int> GetFixedDofs() = 0;
    virtual vector<Real> GetFixedDofValues() = 0;
    virtual void ComputeFixedDofs() = 0;

    ///*
    virtual bool solveIKconstraints(){
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
      SingleRobotCSpace freeSpace = SingleRobotCSpace(*_world,irobot,&settings);

      // The contact cspace needs to be initiziliazed with a free CSpace, and also
      // with the constraints of contacts + which dofs we like to fix. This defines
      // the contact submanifold on which we like to sample for IK solutions)
      ContactCSpace cspace(freeSpace);
      vector<IKGoal> constraints = this->GetProblem();

      //#######################################################################
      //(0) add margin
      //#######################################################################
      //Real margin = 0.01;
      //for(size_t i=0;i<_robot->geometry.size();i++)
      //{
      //  std::cout << i << "/" << _robot->geometry.size() << std::endl;
      //  //SmartPointer<CollisionGeometry> cc = _robot->geometry[i];
      //  //std::cout << _robot->geometry[i]->margin << std::endl;
      //  AnyCollisionGeometry3D *cc = _robot->geometry[i];
      //  std::cout << cc->margin << std::endl;
      //    //std::vector<SmartPointer<CollisionGeometry> > geometry;

      //}
      //#######################################################################
      //(1) add the constraints of IK here
      //#######################################################################
      for(size_t i=0;i<constraints.size();i++) {
        std::cout << "constraint " <<i << " link" << constraints[i].link << std::endl;
        cspace.AddContact(constraints[i]);
      }
      //getchar();

      //#######################################################################
      //(2) which dofs should be fixed
      //#######################################################################
      cspace.fixedDofs = this->GetFixedDofs();
      cspace.fixedValues = this->GetFixedDofValues();

      //#######################################################################
      //(3) ignore collisions between fixed dofs and object
      //and between contact links and object
      //#######################################################################
      int objectid = _world->RigidObjectID(0);
      for(size_t i=0;i<constraints.size();i++) {
        int k=constraints[i].link;
        cspace.ignoreCollisions.push_back(pair<int,int>(objectid,_world->RobotLinkID(irobot,k)));
      }
      for(size_t i=0;i<cspace.fixedDofs.size();i++) {
        int k=cspace.fixedDofs[i];
        cspace.ignoreCollisions.push_back(pair<int,int>(objectid,_world->RobotLinkID(irobot,k)));
      }

      //#######################################################################
      //(4) draw #iters samples from the contact submanifold using IK solver
      //#######################################################################
      Config out = this->_robot->q;
      _isSolved=false;
      int iters = 100;

      //cspace.SolveContact();
      //Config out = this->_robot->q;
      //if(cspace.IsFeasible(out)){
      //  return true;
      //}
      //else {
      //  printf("Infeasible:\n");
      //  cspace.PrintInfeasibleNames(out);
      //  getchar();
      //} 
      this->_robot->q = out;
      while(iters-- > 0 ){
        cspace.Sample(out);
        //cspace.SolveContact();
        std::cout << "DIST TO CMANIF" << cspace.ContactDistance() << std::endl;

        if(cspace.IsFeasible(out)){
          std::cout << "found feasible grasp" << std::endl;
          _isSolved=true;
          break;
        }else{
          cspace.PrintInfeasibleNames(out);
        }
      }
      std::cout << "Sampling Contact Submanifold took " << iters << " iterations" << std::endl;
      return _isSolved;

    }
    //*/

};

