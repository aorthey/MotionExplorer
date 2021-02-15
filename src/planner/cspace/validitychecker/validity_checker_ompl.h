#pragma once
#include "planner/cspace/cspace.h"
#include "neighborhood.h"
#include <ompl/base/StateValidityCheckerDifferentiable.h>


class InteractionPoint
{
  public:
    Robot *robot;

    uint link{0};
    uint collisionGeometriesId{0};

    double distance{0};

    //point on robot link closest to collision geometry
    Vector3 ptRobot;
    //point on collision geometry
    Vector3 ptEnvironment;

    //virtual force (directional vector) from geometry to robot
    Vector3 force;
    //force projected into state space of robot as wrench (on tangent space)
    Vector wrench;
};

class OMPLValidityChecker: public ob::StateValidityCheckerDifferentiable
{

  public:
    // typedef std::vector<std::pair<Vector3, Vector3> > InteractionPoints;
    typedef std::vector<InteractionPoint> InteractionPoints;

    OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_);
    ~OMPLValidityChecker();

    bool IsFeasible(const ob::State* state) const;

    virtual bool IsSufficientFeasible(const ob::State* state) const;
    virtual double SufficientDistance(const ob::State* state) const;

    bool isValid(const ob::State* state) const override;
    bool IsCollisionFree(SingleRobotCSpace *space, Config q) const;

    CSpaceOMPL* GetCSpaceOMPLPtr() const;
    void SetNeighborhood(double);

    virtual double clearance(const ob::State*) const override;

    virtual bool operator ==(const ob::StateValidityChecker &rhs) const override;

    virtual double cost(const ob::State *state) const override;

    double constrainedness(const ob::State *state) const;

    virtual Eigen::VectorXd costGradient(const ob::State *state) const override;

    Vector getVirtualForceRobotToMesh( Robot *robot,
        const ManagedGeometry::GeometryPtr geometry) const;

    virtual void DrawGL(GUIState& state);

    // double costFromInteractionPoints(InteractionPoint& ip1, InteractionPoint& ip2);

    InteractionPoints getInteractionPoints(
        Robot *robot,
        std::vector<ManagedGeometry::GeometryPtr> collisionGeometries 
        ) const;

  protected:
    double DistanceToRobot(const ob::State* state, SingleRobotCSpace *space) const;

    CSpaceOMPL *cspace{nullptr};
    SingleRobotCSpace *klampt_single_robot_cspace{nullptr};
    Neighborhood *neighborhood{nullptr};

    ompl::base::State *tmp;

};

class OMPLValidityCheckerNecessarySufficient: public OMPLValidityChecker
{
  public:
    OMPLValidityCheckerNecessarySufficient(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpaceKlampt *outer_);

    bool IsSufficientFeasible(const ob::State* state) const override;
    double SufficientDistance(const ob::State* state) const override;

  private:
    SingleRobotCSpace *klampt_single_robot_cspace_outer_approximation;
};
typedef std::shared_ptr<OMPLValidityChecker> OMPLValidityCheckerPtr;
typedef std::shared_ptr<OMPLValidityCheckerNecessarySufficient> OMPLValidityCheckerNecessarySufficientPtr;
