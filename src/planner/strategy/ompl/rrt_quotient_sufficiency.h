#pragma once
#include "rrt_quotient.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class RRTQuotientSufficiency : public RRTQuotient
    {
    public:
        //RRTQuotient(const base::SpaceInformationPtr &si, PRMQuotient *previous_);
        RRTQuotientSufficiency(const base::SpaceInformationPtr &si, Quotient *previous_ = nullptr);
        ompl::PDF<Configuration*> GetConfigurationPDF();
        virtual bool SampleGraph(ob::State *q_random_graph) override;


    };
  }
}

