#include "cover_convex_partition.h"
#include "open_set_convex.h"

using namespace cover;
CoverConvexPartition::CoverConvexPartition()
{
}
void CoverConvexPartition::AddOpenSet( OpenSet* set_ )
{
  cover::OpenSetConvex *set = dynamic_cast<cover::OpenSetConvex*>(set_);
  //do not add if subset of cover
  for(uint k = 0; k < opensets.size(); k++){
    if(set->IsSubsetOf(opensets.at(k))){
      rejected_openset++;
      return;
    }
  }

  //clip set so that it is not overlapping cover
  for(uint k = 0; k < opensets.size(); k++){
    set->RemoveIntersection(opensets.at(k));
  }
  opensets.push_back(set);
}
