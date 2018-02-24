#pragma once
#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>
namespace ob = ompl::base;

class PlannerDataVertexAnnotated: public ob::PlannerDataVertex
{
  public:
    PlannerDataVertexAnnotated(const ob::State *st, int tag=0, double d_ = 0.0);
    PlannerDataVertexAnnotated (const PlannerDataVertexAnnotated &rhs);
    double GetOpenNeighborhoodDistance() const;
    void SetOpenNeighborhoodDistance(double d_);
    virtual PlannerDataVertex *clone() const override;
    void SetLevel(uint level_);
    uint GetLevel() const;
    void SetMaxLevel(uint level_);
    uint GetMaxLevel() const;
    void SetComponent(uint component_);
    uint GetComponent() const;

    virtual const ob::State *getState() const override;
    void setState(ob::State *s);


    //virtual bool operator==(const PlannerDataVertex &rhs) const override
    //{
    //  return state_ == rhs.state_;
    //}


  protected:
    double open_neighborhood_distance{0.0};
    uint level{0};
    uint max_level{1};
    uint component{1};

    //template <class Archive>
    //void serialize(Archive & ar, const unsigned int version)
    //{
    //    ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
    //}
};

//BOOST_CLASS_EXPORT(PlannerDataVertexAnnotated);
