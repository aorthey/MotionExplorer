#include "common.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "planner/cspace/contact/ContactConstraint_3D.h"

#include "planner/cspace/cspace_geometric_R3_CONTACT.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"


GeometricCSpaceOMPLRCONTACT_3D::GeometricCSpaceOMPLRCONTACT_3D(RobotWorld *world_, int robot_idx):
        GeometricCSpaceOMPL(world_, robot_idx) {}


void GeometricCSpaceOMPLRCONTACT_3D::initSpace()
{
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(6 + Nompl));

    //Get Bounds on Joints
    std::vector<double> minimum, maximum;
    minimum = robot->qMin;
    maximum = robot->qMax;

    //SE2 Bounds
    ob::RealVectorBounds bounds(6 + Nompl);

    for (int j = 0; j < 6; j++) {
        bounds.low.at(j) = minimum.at(j);
    }
    for (int k = 0; k < 6; k++) {
        bounds.high.at(k) = maximum.at(k);
    }

    for(uint i = 0; i < Nompl;i++){
        uint idx = ompl_to_klampt.at(i);
        bounds.low.at(i + 6) = minimum.at(idx);
        bounds.high.at(i + 6) = maximum.at(idx);
    }
    std::cout << bounds.low << std::endl;
    std::cout << bounds.high << std::endl;
    bounds.check();
    static_pointer_cast<ob::RealVectorStateSpace>(Rn)->setBounds(bounds);

    //Constrained State Space
    constraint = std::make_shared<ContactConstraint_3D>(this, Rn->getDimension(), robot, world);
    this->space = std::make_shared<ob::ProjectedStateSpace>(Rn, constraint);

}

ob::SpaceInformationPtr GeometricCSpaceOMPLRCONTACT_3D::SpaceInformationPtr()
{
    if (!si) {
        si = std::make_shared<ob::ConstrainedSpaceInformation>(SpacePtr());
        validity_checker = StateValidityCheckerPtr(si);
        si->setStateValidityChecker(validity_checker);
    }
    return si;
}

void GeometricCSpaceOMPLRCONTACT_3D::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    Eigen::VectorXd x(3+Nompl);

    for (int i = 0; i < 5; ++i) {
        x[i] = q[i];
    }
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
}

Config GeometricCSpaceOMPLRCONTACT_3D::OMPLStateToConfig(const ob::State *qompl)
{
    auto &&x = *qompl->as<ob::ConstrainedStateSpace::StateType>();

    Config q;
    q.resize(robot->q.size());
    q.setZero();

    for (int i = 0; i < 5; ++i) {
        q(i)=x[i];
    }
    q(3)=x[2];

    for(uint i = 0; i < Nompl; i++){
        uint idx = ompl_to_klampt.at(i);
        q(idx) = x[i+3];
    }
    return q;
}

Config GeometricCSpaceOMPLRCONTACT_3D::EigenVectorToConfig(const Eigen::VectorXd &xd) const
{
    Config q;
    q.resize(robot->q.size());
    q.setZero();

    for (int i = 0; i < 5; ++i) {
        q(i)=xd[i];
    }
    q(3) = xd[2];

    while(q(3)>M_PI) q(3) -= 2*M_PI;
    while(q(3)<-M_PI) q(3) += 2*M_PI;

    for(uint i = 0; i < Nompl; i++){
        uint idx = ompl_to_klampt.at(i);
        q(idx) = xd[i+3];
    }
    return q;
}

//NOTE: add getXYZ to set XYZ coordinate of vertices
Vector3 GeometricCSpaceOMPLRCONTACT_3D::getXYZ(const ob::State *s)
{
    Config q = OMPLStateToConfig(s);
    Vector3 v(q[0],q[1],q[2]);
    return v;
}

void GeometricCSpaceOMPLRCONTACT_3D::print(std::ostream& out) const{}
