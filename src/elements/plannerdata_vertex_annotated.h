#pragma once
#include "planner/cspace/cover/open_set.h"
#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>
namespace ob = ompl::base;

class PlannerDataVertexAnnotated: public ob::PlannerDataVertex
{
  //If new elements are added, you need to update the clone/getstate functions!
  public:
    enum class FeasibilityType{FEASIBLE, INFEASIBLE, SUFFICIENT_FEASIBLE};

    PlannerDataVertexAnnotated(const ob::State *st, int tag=0, double d_ = 0.0);
    PlannerDataVertexAnnotated (const PlannerDataVertexAnnotated &rhs);
    virtual PlannerDataVertex *clone() const override;

    void SetOpenSet( cover::OpenSet* );
    cover::OpenSet* GetOpenSet() const;

    void SetOpenNeighborhoodDistance(double d_);
    double GetOpenNeighborhoodDistance() const;

    void SetLevel(uint level_);
    uint GetLevel() const;

    void SetPath(std::vector<int> path_);
    std::vector<int> GetPath() const;

    void SetMaxLevel(uint level_);
    uint GetMaxLevel() const;

    void SetFeasibility(FeasibilityType feasibility_t_);
    FeasibilityType GetFeasibility() const;

    void SetComponent(uint component_);
    uint GetComponent() const;

    void SetPathClass(uint level_);
    uint GetPathClass() const;

    void SetInfeasible();
    bool IsInfeasible() const;

    void SetMaxPathClass(uint level_);
    uint GetMaxPathClass() const;

    void SetComplex(std::vector<std::vector<long unsigned int>> complex_);
    void AddComplex(std::vector<long unsigned int> simplex_);
    std::vector<std::vector<long unsigned int>> GetComplex() const;

    void setState(ob::State *s);
    virtual const ob::State *getState() const override;

    virtual bool operator==(const PlannerDataVertex &rhs) const override
    {
      const PlannerDataVertexAnnotated &v = static_cast<const PlannerDataVertexAnnotated&>(rhs);
      return (state_ == v.getState() && path == v.GetPath() && level == v.GetLevel());
    }

    friend std::ostream& operator<< (std::ostream&, const PlannerDataVertexAnnotated&);

    void DrawGL(GUIState&);

  protected:
    cover::OpenSet *openset{nullptr};

    bool infeasible{false};
    double open_neighborhood_distance{0.0};
    uint level{0};
    uint max_level{1};

    uint path_class{0};
    uint max_path_class{0};

    std::vector<int> path;

    std::vector<std::vector<long unsigned int>> simplicial_complex_local;

    uint component{99};

    enum ComponentType{START_COMPONENT, GOAL_COMPONENT, OUTLIER_COMPONENT, UNKNOWN};
    ComponentType component_t;
    FeasibilityType feasibility_t;

    //template <class Archive>
    //void serialize(Archive & ar, const unsigned int version)
    //{
    //    ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
    //}
};

//BOOST_CLASS_EXPORT(PlannerDataVertexAnnotated);
