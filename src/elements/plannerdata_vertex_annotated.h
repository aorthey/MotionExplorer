#pragma once
#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>
namespace ob = ompl::base;

class PlannerDataVertexAnnotated: public ob::PlannerDataVertex
{
  public:
    PlannerDataVertexAnnotated(const ob::State *st, int tag=0, double d_ = 1):
      ob::PlannerDataVertex(st,tag), open_neighborhood_distance(d_)
    {
    }
    PlannerDataVertexAnnotated(const ob::State *st, double d_ = 1):
      ob::PlannerDataVertex(st), open_neighborhood_distance(d_)
    {
    }
    PlannerDataVertexAnnotated (const PlannerDataVertexAnnotated &rhs) : ob::PlannerDataVertex(rhs.state_, rhs.tag_)
    {
      open_neighborhood_distance = rhs.GetOpenNeighborhoodDistance();
    }
    double GetOpenNeighborhoodDistance() const
    {
      return open_neighborhood_distance;
    }
    void SetOpenNeighborhoodDistance(double d_)
    {
      open_neighborhood_distance = d_;
    }
    virtual PlannerDataVertex *clone() const override
    {
      return new PlannerDataVertexAnnotated(*this);
    }

    //virtual bool operator==(const PlannerDataVertex &rhs) const override
    //{
    //  return state_ == rhs.state_;
    //}


  protected:
    double open_neighborhood_distance;

    //template <class Archive>
    //void serialize(Archive & ar, const unsigned int version)
    //{
    //    ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
    //}
};

//BOOST_CLASS_EXPORT(PlannerDataVertexAnnotated);
