#include "pathspace_onetopic_cover.h"

PathSpaceOnetopicCover(RobotWorld *world_, PlannerInput& input_):
  PathSpace(world_, input_)
{

}

bool isAtomic(){
  return false;
}

std::vector<PathSpace*> Decompose(){

}


void DrawGL(const GUIState&){

}

