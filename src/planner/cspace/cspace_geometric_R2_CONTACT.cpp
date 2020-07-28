#include "common.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "planner/cspace/contact/ConstraintIntersection_Transition.h"
#include "planner/cspace/contact/ContactConstraint.h"
#include "planner/cspace/contact/TransitionConstraint.h"
#include "planner/cspace/cspace_geometric_R2_CONTACT.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"


GeometricCSpaceOMPLRCONTACT::GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx):
    GeometricCSpaceOMPL(world_, robot_idx) 
{
}

void GeometricCSpaceOMPLRCONTACT::initSpace()
{
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(3 + Nompl));

    //Get Bounds on Joints
    std::vector<double> minimum, maximum;
    minimum = robot->qMin;
    maximum = robot->qMax;

    //SE2 Bounds
    ob::RealVectorBounds bounds(3 + Nompl);
    bounds.low.at(0) = minimum.at(0);
    bounds.low.at(1) = minimum.at(1);
    bounds.low.at(2) = minimum.at(3);

    bounds.high.at(0) = maximum.at(0);
    bounds.high.at(1) = maximum.at(1);
    bounds.high.at(2) = maximum.at(3);

    for(uint i = 0; i < Nompl;i++){
        uint idx = ompl_to_klampt.at(i);
        bounds.low.at(i + 3) = minimum.at(idx);
        bounds.high.at(i + 3) = maximum.at(idx);
    }
    std::cout << bounds.low << std::endl;
    std::cout << bounds.high << std::endl;
    bounds.check();
    static_pointer_cast<ob::RealVectorStateSpace>(Rn)->setBounds(bounds);

    //Constraint Pointer Vector
    std::cout << "Number of Links: " << robot->links.size() - 1 << std::endl;
    for(uint j = 0; j < input.contact_links.size(); j++)
    {
        ContactInformation cj = input.contact_links.at(j);

        int link = cj.robot_link_idx;
        if(cj.mode == "fixed"){
            std::cout << "Adding Fixed Contact Constraint:"
                      << " robot: " << cj.robot_name
                      << ", link: " << cj.robot_link << " (idx: " << cj.robot_link_idx << ")"
                      << " on mesh: " << cj.meshFrom << " (idx: " << cj.meshFromIdx << ")"
                      << std::endl;

            constraints.push_back(std::make_shared<ContactConstraint>(this, Rn->getDimension(), robot, world, link, cj.meshFrom));

        }else if(cj.mode == "transition"){
            std::cout << "Adding Transition Contact Constraint:"
                      << " robot: " << cj.robot_name
                      << ", link: " << cj.robot_link << " (idx: " << cj.robot_link_idx << ")"
                      << ", from mesh: " << cj.meshFrom << " (idx: " << cj.meshFromIdx << ")"
                      << " to mesh: " << cj.meshTo << " (idx: " << cj.meshToIdx << ")"
                      << std::endl;

            constraints.push_back(std::make_shared<TransitionConstraint>(this, Rn->getDimension(), robot, world, link, cj.meshFrom, cj.meshTo));

        }else{
            std::cout << "Could not identify contact mode" << std::endl;
            exit(0);
        }

    }

    //Constraint Intersection to join multiple constraints
    constraint_intersect = std::make_shared<ConstraintIntersectionTransition>(Rn->getDimension(), constraints);
    ob::StateSpacePtr RN_Constraint =  std::make_shared<ob::ProjectedStateSpaceTransition>(Rn, constraint_intersect);

    this->space = RN_Constraint;
}

ob::SpaceInformationPtr GeometricCSpaceOMPLRCONTACT::SpaceInformationPtr()
{
    if (!si) {
        si = std::make_shared<ob::ConstrainedSpaceInformation>(SpacePtr());
        validity_checker = StateValidityCheckerPtr(si);
        si->setStateValidityChecker(validity_checker);
    }
    return si;
}

void GeometricCSpaceOMPLRCONTACT::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    Eigen::VectorXd x(3+Nompl);

    x[0] = q[0];
    x[1] = q[1];
    x[2] = q[3];

    for(uint i = 0; i < Nklampt; i++)
    {
        int idx = klampt_to_ompl.at(i);
        if(idx<0) continue;
        else{
            x[3 + idx]=q(6+i);
        }
    }

    qompl->as<ob::ConstrainedStateSpace::StateType>()->copy(x);


    //TODO: set into right modus
    // for(uint i = 0; i < constraints.size(); i++)
    // {
    //   ob::ConstraintPtr cPi = constraints.at(i);

    //     // check if constraints in vector are transitionConstraints
    //     const TransitionConstraintPtr tCP = std::dynamic_pointer_cast<TransitionConstraint>(cPi);
    //     if (tCP != nullptr)
    //     {
    //       std::cout << "Constraint " << i << " modus " << tCP->getMode() << std::endl;
    //     }
    // }


    static_pointer_cast<ob::ConstrainedStateSpace>(this->space)->getConstraint()->project(qompl);
    SpaceInformationPtr()->enforceBounds(qompl);
}

Config GeometricCSpaceOMPLRCONTACT::OMPLStateToConfig(const ob::State *qompl)
{
    auto &&x = *qompl->as<ob::ConstrainedStateSpace::StateType>();

    Config q;
    q.resize(robot->q.size());
    q.setZero();

    q(0)=x[0];
    q(1)=x[1];
    q(3)=x[2];

    for(uint i = 0; i < Nompl; i++){
        uint idx = ompl_to_klampt.at(i);
        q(idx) = x[i+3];
    }
    return q;
}

Config GeometricCSpaceOMPLRCONTACT::EigenVectorToConfig(const Eigen::VectorXd &xd) const
{
    Config q;
    q.resize(robot->q.size());
    q.setZero();

    q(0) = xd[0];
    q(1) = xd[1];
    q(3) = xd[2];

    if(q(3) != q(3))
    {
      std::cout << std::string(80, '-') << std::endl;
      std::cout << xd << std::endl;
      std::cout << "NAN" << std::endl;
      exit(0);
    }
    while(q(3) > M_PI) q(3) -= 2*M_PI;
    while(q(3) < -M_PI) q(3) += 2*M_PI;

    for(uint i = 0; i < Nompl; i++){
        uint idx = ompl_to_klampt.at(i);
        q(idx) = xd[i+3];
    }
    return q;
}

//NOTE: add getXYZ to set XYZ coordinate of vertices
Vector3 GeometricCSpaceOMPLRCONTACT::getXYZ(const ob::State *s)
{
    Config q = OMPLStateToConfig(s);
    Vector3 v(q[0],q[1],0);
    return v;
}

void GeometricCSpaceOMPLRCONTACT::print(std::ostream& out) const{}
