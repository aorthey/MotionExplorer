#include "planner/cspace/cspace_geometric_R_CONTACT.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <Library/KrisLibrary/math3d/geometry3d.cpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include "common.h"

GeometricCSpaceOMPLRCONTACT::GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx):
        GeometricCSpaceOMPL(world_, robot_idx)
{}

class ContactConstraint : public ob::Constraint
{
protected:
    std::vector<Triangle3D> trisFiltered;
    std::vector<Vector2> cornerCoord;

public:
    ContactConstraint(Robot *robot, RobotWorld *world)
    : ob::Constraint(4, 2) // (x,y,theta at 1st link,phi at 2nd)
    , robot_(robot)
    , world_(world)
    {


        //Adding triangle information to PlannerInput (to be used as constraint manifolds)
        std::vector<Triangle3D> tris;

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
            }
            else{
                // only x and y coordinates
                Vector2 a = Vector2(tris.at(l).a[0], tris.at(l).a[1]);
                Vector2 b = Vector2(tris.at(l).b[0], tris.at(l).b[1]);
                Vector2 c = Vector2(tris.at(l).c[0], tris.at(l).c[1]);

                cornerCoord.push_back(a);
                cornerCoord.push_back(b);
                cornerCoord.push_back(c);

                // trisFiltered filled with surface triangles that are feasible for contact (normal in x or y direction)
                trisFiltered.push_back(tris.at(l));
            }
        }
        std::cout << "Environment has " << trisFiltered.size() << " triangles to make contact!" << std::endl;

        // remove all duplicates of (2D) corner coordinates
        auto end = cornerCoord.end();
        for(auto it = cornerCoord.begin(); it != end; ++it){
            end = std::remove(it + 1, end, *it);
        }
        cornerCoord.erase(end, cornerCoord.end());
        std::cout << "Filtered corner coordinates: " << cornerCoord << std::endl;

        // robot = world_->robots[robot_idx];

    }

    Vector3 getContact(Eigen::VectorXd xd) const
    {
        Config  q;

        for (int r = 0; r < xd.size(); r++) {
            q(r) = xd[r]; // problem here with q, "segmentation fault"
        }

        robot_->UpdateConfig(q);
        robot_->UpdateGeometry();
        Vector3 qq;
        Vector3 zero; zero.setZero();
        //int lastLink = robot_->links.size() - 1;
        int firstLink = 0; // maybe .type to check type of last/first link, ball endeffector

        //NOTE: the world position is zero exactly at the point where link is
        //attached using a joint to the whole linkage. Check where your last fixed
        //joint is positioned, before questioning the validity of this method
        robot_->GetWorldPosition(zero, firstLink, qq);

        double x = qq[0];
        double y = qq[1];
        double z = qq[2];
        Vector3 v(x,y,z);

        return v;
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {

        Vector3 contact = getContact(x);
        std::cout << contact << std::endl;

        //not checked yet if this works
        GeometricPrimitive3D gP3D = GeometricPrimitive3D(contact);
        Real dist = gP3D.Distance(trisFiltered.at(0));

        std::cout << dist << std::endl;

        //out[0] = x.norm() - 1; // ----change to fit contact case-------
        out[0] = 0; // empty constraint
    }

private:
    Robot *robot_;
    RobotWorld *world_;
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
    constraint = std::make_shared<ContactConstraint>(robot, world);
    this->space = std::make_shared<ob::ProjectedStateSpace>(Rn, constraint);
}

ob::SpaceInformationPtr GeometricCSpaceOMPLRCONTACT::SpaceInformationPtr() {
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

void GeometricCSpaceOMPLRCONTACT::print(std::ostream& out) const
{
}


//Vector3 GeometricCSpaceOMPLRCONTACT::getXYZ(const Eigen::VectorXd xd)
//{
//    Config  q;
//    for (int r = 0; r < xd.size(); r++) {
//        q(r) = xd[r];
//    }
//
//    robot->UpdateConfig(q);
//    robot->UpdateGeometry();
//    Vector3 qq;
//    Vector3 zero; zero.setZero();
//    int lastLink = robot->links.size()-1;
//    //int firstLink = robot->links.0; check type? obs ne kugel ist
//
//    //NOTE: the world position is zero exactly at the point where link is
//    //attached using a joint to the whole linkage. Check where your last fixed
//    //joint is positioned, before questioning the validity of this method
//    robot->GetWorldPosition(zero, lastLink, qq);
//
//    double x = qq[0];
//    double y = qq[1];
//    double z = qq[2];
//    Vector3 v(x,y,z);
//    return v;
//}