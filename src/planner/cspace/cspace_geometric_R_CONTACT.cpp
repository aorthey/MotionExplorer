#include "planner/cspace/cspace_geometric_R_CONTACT.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include "common.h"

GeometricCSpaceOMPLRCONTACT::GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx):
        GeometricCSpaceOMPL(world_, robot_idx)
{
    //Adding triangle information to PlannerInput (to be used as constraint
    //manifolds)
    //filtering for those triangles that belong to feasible contact surfaces
    std::vector<Triangle3D> tris;
    std::vector<Triangle3D> tris_filtered;

    std::vector<Vector2> cornerCoords;

    for(uint k = 0; k < world_->terrains.size(); k++){
        Terrain* terrain_k = world_->terrains[k];
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
            //do nothing
        }else{
            Vector2 aa = Vector2(tris.at(l).a[0], tris.at(l).a[1]);
            Vector2 bb = Vector2(tris.at(l).b[0], tris.at(l).b[1]);
            Vector2 cc = Vector2(tris.at(l).c[0], tris.at(l).c[1]);

            cornerCoords.push_back(aa);
            cornerCoords.push_back(bb);
            cornerCoords.push_back(cc);
            //std::cout << "Corner A: " << aa << std::endl;
            //std::cout << "Corner B: " << bb << std::endl;
            //std::cout << "Corner C: " << cc << std::endl;

            // tris_filtered filled with surface triangles that are feasible for contact (normal in x or y direction)
            tris_filtered.push_back(tris.at(l));
        }
    }
    std::cout << "Environment has " << tris_filtered.size() << " triangles to make contact!" << std::endl;
    std::cout << "Filtered corner coordinates: " << cornerCoords << std::endl;

    surf_triangles = tris_filtered;
    robot = world_->robots[robot_idx];

}

class ContactConstraint : public ob::Constraint
{
public:
    ContactConstraint() : ob::Constraint(4, 2) // (x,y,theta at 1st link,phi at 2nd)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        //out[0] = x.norm() - 1; // ----change to fit contact case-------
        out[0] = 0; // empty constraint
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
    // std::cout << bounds.low << std::endl;
    // std::cout << bounds.high << std::endl;
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