#pragma once
#include <stdio.h>

class ObjectPlacementManager{
  public:
    ObjectPlacementManager( RobotWorld *world ):
      _world(world)
    {
    }
    void spawn(){
      //spawn new object in world
      string objFile = "/home/aorthey/git/Klampt/data/robots/door.rob";
      //Robot nobj;
      //nobj.Load(objFile);
      std::cout << "adding objFile" << objFile << std::endl;
      std::cout << "IDs in world " << this->_world->NumIDs() << std::endl;
      //this->_world->LoadRobot(objFile);
      std::cout << "done adding objFile" << objFile << std::endl;
      std::cout << "IDs in world " << this->_world->NumIDs() << std::endl;
    }

    RobotWorld *_world;

};
