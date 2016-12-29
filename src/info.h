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

        vector<string> links = (*it)->linkNames;
        for(int i = 0; i< links.size(); i++){
          std::cout<< "Link[" << i << "] " << links[i] << std::endl;
        }

        vector<RobotJoint> joints = (*it)->joints;

        for(int i = 0; i< joints.size(); i++){
          std::cout<< "Joint[" << i << "] linkidx " << joints[i].linkIndex << " baseidx " << joints[i].baseIndex << " type " << joints[i].type << std::endl;
        }

        vector<RobotJointDriver> drivers = (*it)->drivers;
        vector<string> dnames = (*it)->driverNames;

        for(int i = 0; i< drivers.size(); i++){
          std::cout<< "Joint[" << i << "] " << dnames[i] << " qmin " << drivers[i].qmin << " qmax " << drivers[i].qmax << std::endl;
        }
      }
    }
  private:
    RobotWorld *_world;
};

