#pragma once
#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

/*****************************************************
 * Contact Constraint
 * -> for a link that doesn't break contact with initial contact surface.
 *
 * @param cspace
 * @param robot
 * @param world
 * @param linkNumber
 * @param obstacleNumber
 * ***************************************************/

class GeometricCSpaceOMPLRCONTACT;

class ContactConstraint : public ob::Constraint
{
protected:
    std::vector<Triangle3D> trisFiltered;
    std::vector<Triangle3D> trisFiltered_negative;
    //std::vector<Vector2> cornerCoord;

public:
    ContactConstraint(GeometricCSpaceOMPLRCONTACT *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber, uint startMeshIdx);


    Vector3 getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const;
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;


    ob::ProjectionEvaluatorPtr getProjection(ob::StateSpacePtr space) const
    {

        class ContactProjection : public ob::ProjectionEvaluator
        {
        public:
            ContactProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
            {
            }

            unsigned int getDimension() const override
            {
                return 3;
            }

            void defaultCellSizes() override
            {
                cellSizes_.resize(2);
                cellSizes_[0] = 0.1;
                cellSizes_[1] = 0.1;
                cellSizes_[2] = 0.1;
            }

            void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
            {
                auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();
                projection(0) = x[0];
                projection(1) = x[1];
                projection(2) = 0;
                std::cout << "Project " << projection << std::endl;
            }
        };
        return std::make_shared<ContactProjection>(space);
    }


private:
    GeometricCSpaceOMPLRCONTACT *cspace_;
    Robot *robot_;
    RobotWorld *world_;
    uint linkNumber_;
    uint startMeshIdx_;
};
