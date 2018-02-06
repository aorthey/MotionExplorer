#pragma once
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <Modeling/Robot.h>
#include "elements/swept_volume.h"

using namespace GLDraw;



// 
//Swept Volume: Volume of Robot along a path in CSpace
//Swath Volume: Volume of Robot along a graph in CSpace
//

class SwathVolume: public SweptVolume
{
  public:
    SwathVolume(Robot *robot);
    SwathVolume(Robot *robot, const std::vector<Config> &vertices);

    const std::vector<Config >& GetVertices();
};


