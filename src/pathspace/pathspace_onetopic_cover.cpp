#include "pathspace_onetopic_cover.h"

PathSpaceOnetopicCover::PathSpaceOnetopicCover(RobotWorld *world_, PlannerInput& input_):
  PathSpace(world_, input_)
{

}

bool PathSpaceOnetopicCover::isAtomic(){
  return false;
}

std::vector<PathSpace*> PathSpaceOnetopicCover::Decompose(){

}


void PathSpaceOnetopicCover::DrawGL(const GUIState&){

}

