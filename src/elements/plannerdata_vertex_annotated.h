#pragma once
#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>
namespace ob = ompl::base;

class PlannerDataVertexAnnotated: public ob::PlannerDataVertex
{
  public:
    PlannerDataVertexAnnotated(const ob::State *st, int tag=0, double d_ = 1):
      ob::PlannerDataVertex(st,tag), open_neighborhood_distance(d_), level(-1), max_level(0)
    {
    }
    PlannerDataVertexAnnotated(const ob::State *st, double d_ = 1):
      ob::PlannerDataVertex(st), open_neighborhood_distance(d_), level(-1), max_level(0)
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
    void SetLevel(uint level_)
    {
      level = level_;
    }
    uint GetLevel()
    {
      return level;
    }
    void SetMaxLevel(uint level_)
    {
      max_level = level_;
    }
    uint GetMaxLevel()
    {
      return max_level;
    }
    virtual const ob::State *getState() const override
    {
      return state_;
    }
    void setState(ob::State *s)
    {
      state_ = s;
    }


    //virtual bool operator==(const PlannerDataVertex &rhs) const override
    //{
    //  return state_ == rhs.state_;
    //}


  protected:
    double open_neighborhood_distance;
    uint level;
    uint max_level;

    //template <class Archive>
    //void serialize(Archive & ar, const unsigned int version)
    //{
    //    ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
    //}
};

//BOOST_CLASS_EXPORT(PlannerDataVertexAnnotated);
