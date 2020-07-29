#include "common.h"
#include "planner/cspace/cspace_geometric_contact_3d.h"
#include "planner/cspace/contact/ProjectedStateSpaceMultiMode.h"
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>


GeometricCSpaceContact3D::GeometricCSpaceContact3D(RobotWorld *world_, int robot_idx):
        GeometricCSpaceContact(world_, robot_idx) 
{
}


void GeometricCSpaceContact3D::initSpace()
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

    initConstraints(Rn);

    ob::StateSpacePtr RN_Constraint =  
      std::make_shared<ob::ProjectedStateSpaceMultiMode>(Rn, getConstraints());

    this->space = RN_Constraint;
}

void GeometricCSpaceContact3D::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    Eigen::VectorXd x(6+Nompl);

    for (int i = 0; i < 6; i++) {
        x[i] = q[i];
    }

    for(uint i = 0; i < Nklampt; i++)
    {
        int idx = klampt_to_ompl.at(i);
        if(idx<0) continue;
        else{
            x[6 + idx]=q(6+i);
        }
    }

    qompl->as<ob::ConstrainedStateSpace::StateType>()->copy(x);
    getConstraints()->project(qompl);
    SpaceInformationPtr()->enforceBounds(qompl);
}

Config GeometricCSpaceContact3D::OMPLStateToConfig(const ob::State *qompl)
{
    auto &&x = *qompl->as<ob::ConstrainedStateSpace::StateType>();

    Config q;
    q.resize(robot->q.size());
    q.setZero();

    for (int i = 0; i < 6; i++) {
        q(i)=x[i];
    }

    for(uint i = 0; i < Nompl; i++){
        uint idx = ompl_to_klampt.at(i);
        q(idx) = x[i+6];
    }
    return q;
}

Config GeometricCSpaceContact3D::EigenVectorToConfig(const Eigen::VectorXd &xd) const
{
    Config q;
    q.resize(robot->q.size());
    q.setZero();

    for (int i = 0; i < 6; i++) {
        q(i)=xd[i];
    }

    while(q(3)>M_PI) q(3) -= 2*M_PI;
    while(q(3)<-M_PI) q(3) += 2*M_PI;

    for(uint i = 0; i < Nompl; i++){
        uint idx = ompl_to_klampt.at(i);
        q(idx) = xd[i+6];
    }
    return q;
}

//NOTE: add getXYZ to set XYZ coordinate of vertices
Vector3 GeometricCSpaceContact3D::getXYZ(const ob::State *s)
{
    Config q = OMPLStateToConfig(s);
    Vector3 v(q[0],q[1],q[2]);
    return v;
}

void GeometricCSpaceContact3D::print(std::ostream& out) const{}
