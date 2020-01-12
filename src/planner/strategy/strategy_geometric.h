#pragma once
#include "planner/strategy/strategy.h"
// #include <omplapp/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

struct OMPLGeometricStratification{
  std::vector<ob::SpaceInformationPtr> si_vec;
  ob::ProblemDefinitionPtr pdef;
  OMPLGeometricStratification( std::vector<ob::SpaceInformationPtr> si_vec_, ob::ProblemDefinitionPtr pdef_):
    si_vec(si_vec_), pdef(pdef_)
  {
  }
};

typedef std::shared_ptr<OMPLGeometricStratification> OMPLGeometricStratificationPtr;

class StrategyGeometricMultiLevel: public Strategy{
  public:
    StrategyGeometricMultiLevel() = default;

    virtual void Plan( StrategyOutput &output) override;
    virtual void Step( StrategyOutput &output) override;
    virtual void Init( const StrategyInput &input) override;
    virtual void Clear() override;

    ob::PlannerPtr GetPlanner(std::string algorithm,
        OMPLGeometricStratificationPtr stratification);

    void RunBenchmark(const StrategyInput& input);
    OMPLGeometricStratificationPtr OMPLGeometricStratificationFromCSpaceStratification
    (const StrategyInput &input, std::vector<CSpaceOMPL*> cspace_levels );

    // template<class T_Algorithm>
    // ob::PlannerPtr GetSharedMultiChartPtr( 
    //     OMPLGeometricStratificationPtr stratification);
    // template<class T_Algorithm>
    // ob::PlannerPtr GetSharedMultiQuotientPtr( 
    //     OMPLGeometricStratificationPtr stratification);
    // template<class T_Algorithm, class T_Algorithm_Two>
    // ob::PlannerPtr GetSharedMultiQuotientPtr( 
    //     OMPLGeometricStratificationPtr stratification);
};

