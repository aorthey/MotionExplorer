#include "planner/cspace/cspace_geometric_R_CONTACT.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include "common.h"

GeometricCSpaceOMPLRCONTACT::GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
/*    //Adding triangle information to PlannerInput (to be used as constraint
    //manifolds)
    //filtering for those triangles that belong to feasible contact surfaces
    std::vector<Triangle3D> tris;
    std::vector<Triangle3D> tris_filtered;

    for(uint k = 0; k < world->terrains.size(); k++){
        Terrain* terrain_k = world->terrains[k];
        const Geometry::CollisionMesh mesh = terrain_k->geometry->TriangleMeshCollisionData();//different mesh
        for(uint j = 0; j < mesh.tris.size(); j++){
            Triangle3D tri;
            mesh.GetTriangle(j, tri);
            tris.push_back(tri);
        }
    }

    for(uint l = 0; l < tris.size(); l++){
        Vector3 normal = tris.at(l).normal();
        double epsilon = 1e-10;

        if(fabs((fabs(normal[2]) - 1.0))<epsilon){

            std::cout << "normal vector is in z-direction" << std::endl;

        }else{
            // get each triangle's corner coordinates and remove z coordinate
            Vector3 a = tris.at(l).a;
            Vector3 b = tris.at(l).b;
            Vector3 c = tris.at(l).c;

            std::vector<double> cornerA;
            cornerA.push_back(a[0]);
            cornerA.push_back(a[1]);

            std::vector<double> cornerB;
            cornerB.push_back(b[0]);
            cornerB.push_back(b[1]);

            std::vector<double> cornerC;
            cornerC.push_back(c[0]);
            cornerC.push_back(c[1]);

            std::cout << "Corner A1: " << cornerA[0] << std::endl;
            std::cout << "Corner A2: " << cornerA[1] << std::endl;
            //std::cout << "Corner B: " << cornerB << std::endl;
            //std::cout << "Corner C: " << cornerC << std::endl;

            // tris_filtered filled with surface triangles that are feasible for contact (normal in x or y direction)
            tris_filtered.push_back(tris.at(l));
        }

    }
    // ---------------------------currently doesnt get the right input -> output says 0 triangles-----------------------
    std::cout << "Environment has " << tris_filtered.size() << " triangles to make contact" << std::endl;

    surf_triangles = tris_filtered;

    robot = world->robots[robot_idx];*/

}


class ContactConstraint : public ob::Constraint
{
public:
    ContactConstraint() : ob::Constraint(4, 1) // (x,y,theta at 1st link,phi at 2nd)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        //out[0] = x.norm() - 1; // ----change to fit contact case-------
        out[0] = 0; // empty constraint, always minimized?
    }
};

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

    //Constrained State Space
    constraint = std::make_shared<ContactConstraint>();
    this->space = std::make_shared<ob::ProjectedStateSpace>(Rn, constraint);
}

ob::SpaceInformationPtr GeometricCSpaceOMPLRCONTACT::SpaceInformationPtr() {
    if (!si) { //si: stateInformationPtr from cspace.h
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
}

Config GeometricCSpaceOMPLRCONTACT::OMPLStateToConfig(const ob::State *qompl)
{
    auto &&x = *qompl->as<ob::ConstrainedStateSpace::StateType>();

    Config q;q.resize(robot->q.size());q.setZero();

    q(0)=x[0];
    q(1)=x[1];
    q(3)=x[2];

    for(uint i = 0; i < Nompl; i++){
        uint idx = ompl_to_klampt.at(i);
        q(idx) = x[i+3];
    }
    return q;
}

void GeometricCSpaceOMPLRCONTACT::print(std::ostream& out) const
{
}


Vector3 GeometricCSpaceOMPLRCONTACT::getXYZ(const ob::State *s)
{
    Config q = OMPLStateToConfig(s);
    robot->UpdateConfig(q);
    robot->UpdateGeometry();
    Vector3 qq;
    Vector3 zero; zero.setZero();
    int lastLink = robot->links.size()-1;
    //int firstLink = robot->links.0;

    //NOTE: the world position is zero exactly at the point where link is
    //attached using a joint to the whole linkage. Check where your last fixed
    //joint is positioned, before questioning the validity of this method
    robot->GetWorldPosition(zero, lastLink, qq);

    double x = qq[0];
    double y = qq[1];
    double z = qq[2];
    Vector3 v(x,y,z);
    return v;
}

//ob::SpaceInformationPtr CSpaceOMPL::SpaceInformationPtr() {
//    if (!si) { //si: stateInformationPtr from cspace.h
//        si = std::make_shared<ob::SpaceInformation>(SpacePtr());
//        // csi = std::make_shared<ob::ContactSpaceInformation>(SpacePtr());
//        validity_checker = StateValidityCheckerPtr(si);
//        si->setStateValidityChecker(validity_checker);
//    }
//    return si;
//}
