#pragma once
#include "planner/strategy/strategy.h"
// #include <omplapp/config.h>

struct OMPLGeometricStratification{
  std::vector<ompl::base::SpaceInformationPtr> si_vec;
  ompl::base::ProblemDefinitionPtr pdef;
  OMPLGeometricStratification( std::vector<ompl::base::SpaceInformationPtr> si_vec_, ompl::base::ProblemDefinitionPtr pdef_):
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

    ompl::base::PlannerPtr GetPlanner(std::string algorithm,
        OMPLGeometricStratificationPtr stratification);

    void RunBenchmark(const StrategyInput& input);
    OMPLGeometricStratificationPtr OMPLGeometricStratificationFromCSpaceStratification
    (const StrategyInput &input, std::vector<CSpaceOMPL*> cspace_levels );

    // template<class T_Algorithm>
    // ompl::base::PlannerPtr GetSharedMultiChartPtr( 
    //     OMPLGeometricStratificationPtr stratification);
    // template<class T_Algorithm>
    // ompl::base::PlannerPtr GetSharedMultiQuotientPtr( 
    //     OMPLGeometricStratificationPtr stratification);
    // template<class T_Algorithm, class T_Algorithm_Two>
    // ompl::base::PlannerPtr GetSharedMultiQuotientPtr( 
    //     OMPLGeometricStratificationPtr stratification);
};

