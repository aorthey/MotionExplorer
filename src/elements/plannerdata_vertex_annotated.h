#pragma once
#include "planner/cspace/cover/open_set.h"
#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>
namespace ob = ompl::base;

class PlannerDataVertexAnnotated: public ob::PlannerDataVertex
{
  //If new elements are added, you need to update the clone/getstate functions!
  public:
    PlannerDataVertexAnnotated(const ob::State *st, int tag=0, double d_ = 0.0);
    PlannerDataVertexAnnotated (const PlannerDataVertexAnnotated &rhs);
    virtual PlannerDataVertex *clone() const override;

    void SetOpenSet( cover::OpenSet* );
    cover::OpenSet* GetOpenSet() const;

    void SetOpenNeighborhoodDistance(double d_);
    double GetOpenNeighborhoodDistance() const;
    void SetLevel(uint level_);
    uint GetLevel() const;
    void SetMaxLevel(uint level_);
    uint GetMaxLevel() const;
    void SetPathClass(uint level_);
    uint GetPathClass() const;
    void SetMaxPathClass(uint level_);
    uint GetMaxPathClass() const;
    void SetComponent(uint component_);
    uint GetComponent() const;
    void setState(ob::State *s);
    virtual const ob::State *getState() const override;

    virtual bool operator==(const PlannerDataVertex &rhs) const override
    {
      return state_ == rhs.getState();
    }

    friend std::ostream& operator<< (std::ostream&, const PlannerDataVertexAnnotated&);

    void DrawGL(GUIState&);

  protected:
    cover::OpenSet *openset{nullptr};

    double open_neighborhood_distance{0.0};
    uint level{0};
    uint max_level{1};

    uint path_class{0};
    uint max_path_class{0};

    uint component{99};

    enum ComponentType{START_COMPONENT, GOAL_COMPONENT, OUTLIER_COMPONENT, UNKNOWN};
    ComponentType component_t;

    //template <class Archive>
    //void serialize(Archive & ar, const unsigned int version)
    //{
    //    ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
    //}
};

//BOOST_CLASS_EXPORT(PlannerDataVertexAnnotated);
