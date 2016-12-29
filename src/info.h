#pragma once
#include <stdio.h>

class Info
{
  public:
    Info(RobotWorld *world):
      _world(world)
    {
      std::cout << "Information Module initialized" << std::endl;
    }
    void print(){
      int ids = _world->NumIDs();

      std::cout << std::string(80, '-') << std::endl;

      std::cout << "RobotWorld Info" << std::endl;

      std::cout << std::string(80, '-') << std::endl;
      for(int itr = 0; itr <= ids; itr++){
        std::cout << "[" << itr << "] " << _world->GetName(itr) << std::endl;
      }
      std::cout << std::string(80, '-') << std::endl;

      std::vector<SmartPointer<Robot> > robots = _world->robots;
      std::vector<SmartPointer<Terrain> > terrains = _world->terrains;
      for (std::vector<SmartPointer<Robot> >::iterator it = robots.begin() ; it != robots.end(); ++it){
        std::cout << "Robot " << (*it)->name << std::endl;
      }
    }
  private:
    RobotWorld *_world;
};

