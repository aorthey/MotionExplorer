#pragma once
#include "open_set.h"

namespace cover{

  //@brief: a cover of the cspace is a set of open sets. There are two marked
  //open sets, the ones containing the start and the goal configuration. Open
  //sets are allowed to overlap, but not to be nested inside other open sets.

  class Cover{
    public:
      Cover();

      bool IsInsideCover(ob::State *s);

      void Reduce();
      void AddOpenSet( OpenSet* set );
      void AddStartOpenSet( OpenSet* set );
      void AddGoalOpenSet( OpenSet* set );

      void Clear();

      std::vector<OpenSet*> GetCover() const;
      int GetStartSetIndex() const;
      int GetGoalSetIndex() const;

      friend std::ostream& operator<< (std::ostream& out, const Cover& cvr);
    protected:
      CSpace *cspace;
      int startSet{-1};
      int goalSet{-1};
      std::vector<OpenSet*> opensets;
  };
};
