#pragma once

#include <ompl/base/Planner.h>
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

namespace ob = ompl::base;

//  Quotient Q1 contains a single Quotient Space Decomposition which is nested in a chain of
//  quotient spaces. This single decomposition can be visualized as
//
// [    ][ Q0 ]
// [ Q1 ][____]
// [    ][ X1 ]
// [    ][    ]
//
//  cspace X = X0 x X1 
//  Q0 = X0
//  Q1 = X0 x X1
//
//  whereby Q1 is the configuration space, X1 is a designated subspace of Q1, and Q0 =
//  Q1/X1 is the resulting quotient space. Here we assume that Q1 and Q0 have
//  been given, and we compute the inverse of the quotient map, i.e. X1 = Q1/Q0. 
//
//Cases we can handle:
// ---- non-compound:
// (1) Q1 Rn       , Q0 Rm       , 0<m<n   => X1 = R(n-m)
// ---- compound:
// (2) Q1 SE2      , Q0 R2                 => X1 = SO2
// (3) Q1 SE3      , Q0 R3                 => X1 = SO3
// (4) Q1 SE3xRn   , Q0 SE3                => X1 = Rn
// (5) Q1 SE3xRn   , Q0 SE3xRm   , 0<m<n   => X1 = R(n-m)
//
//TO be done (might be beneficial for rigid objects floating in space)
///// Q1 SE3        , Q0 R3xSO2xSO2         =>X1 = SO2
///// Q1 R3xSO2xSO2 , Q0 R3                 =>X1 = SO2xSO2
namespace ompl
{
  namespace geometric
  {
    class Quotient: public ob::Planner
    {
      public:
        const uint verbose{1};
        Quotient(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);
        ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override final; //final prevents subclasses to override

        virtual void Init() = 0;
        virtual void Grow(double t) = 0;
        virtual bool GetSolution(ob::PathPtr &solution) = 0;

        virtual bool SampleQuotient(ob::State *q_random);
        virtual bool Sample(ob::State *q_random);
        virtual bool HasSolution();
        virtual void clear() override;

        virtual double GetImportance() const;

        static void resetCounter();
        void MergeStates(const ob::State *qQ0, const ob::State *qX1, ob::State *qQ1) const;
        const ob::SpaceInformationPtr &GetX1() const;
        const ob::SpaceInformationPtr &GetQ1() const;
        const ob::SpaceInformationPtr &GetQ0() const;
        Quotient* GetParent() const;
        Quotient* GetChild() const;
        uint GetLevel() const;
        void SetLevel(uint);
        void SetChild(Quotient *child_);
        void SetParent(Quotient *parent_);

        friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::Quotient& qtnt);
        //friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::Quotient* qtnt);
        virtual void Print(std::ostream& out) const;

      protected:

        const ob::StateSpacePtr ComputeQuotientSpace(const ob::StateSpacePtr Q1, const ob::StateSpacePtr Q0);
        void ExtractX1Subspace( const ob::State* q, ob::State* qX1 ) const;
        void ExtractQ0Subspace( const ob::State* q, ob::State* qQ0 ) const;

        ob::SpaceInformationPtr Q1;
        ob::SpaceInformationPtr Q0;
        ob::SpaceInformationPtr X1;

        ob::StateSamplerPtr X1_sampler;
        ob::StateSamplerPtr Q1_sampler;
        ob::ValidStateSamplerPtr Q1_valid_sampler;

        ob::OptimizationObjectivePtr opt_;

        enum QuotientSpaceType{ UNKNOWN, ATOMIC_RN, RN_RM, SE2_R2, SE3_R3, SE3RN_R3, SE3RN_SE3, SE3RN_SE3RM };
        QuotientSpaceType type;
        uint Q1_dimension;
        uint Q0_dimension;
        uint X1_dimension;

        static uint counter;
        uint id;
        uint level{0};

        bool hasSolution{false};

        Quotient *parent{nullptr};
        Quotient *child{nullptr};

        //Outer robot available
        OMPLValidityCheckerPtr checker;
        bool checkOuterRobot{false};
        bool IsOuterRobotFeasible(ob::State *state);
        double DistanceOuterRobotToObstacle(ob::State *state);
        double DistanceInnerRobotToObstacle(ob::State *state);
        double DistanceRobotToObstacle(ob::State *state);
    };
  }
}


