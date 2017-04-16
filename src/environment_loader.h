#pragma once
#include <tinyxml.h>
#include "util.h"
#include "gui.h"
#include "loader.h"
#include "planner/planner_input.h"

class EnvironmentLoader{
  private:
    std::string file_name;
    RobotWorld world;
    ForceFieldBackendPtr _backend;
    Info info;

    PlannerInput pin;

  public:
    RobotWorld& GetWorld(){
      return world;
    }
    RobotWorld* GetWorldPtr(){
      return &world;
    }
    PlannerInput GetPlannerInput(){
      return pin;
    }
    ForceFieldBackendPtr GetBackendPtr(){
      return _backend;
    }

    EnvironmentLoader(const char *xml_file){
      file_name = util::GetApplicationFolder()+xml_file;

      std::cout << "[EnvironmentLoader] loading from file " << file_name << std::endl;

      _backend = new ForceFieldBackend(&world);
      _backend->LoadAndInitSim(file_name.c_str());

      uint Nrobots = world.robots.size();
      if(Nrobots!=1){
        std::cout << "Current planner only supports 1 robot! selected " << Nrobots << " robots." << std::endl;
        for(int i = 0; i < Nrobots; i++){
          std::cout << world.robots[i]->name << std::endl;
        }
        std::cout << "Has the xml files been loaded multiple times?" << std::endl;
        exit(0);
      }

      info(&world);

      Robot *robot = world.robots[0];
      LoadPlannerSettings(file_name.c_str());

      for(int i = 0; i < 6; i++){
        robot->qMin[i] = pin.se3min[i];
        robot->qMax[i] = pin.se3max[i];
      }
      pin.qMin = robot->qMin;
      pin.qMax = robot->qMax;

    }

    bool LoadPlannerSettings(TiXmlElement *node)
    {
      CheckNodeName(node, "world");

      TiXmlElement* plannersettings = FindSubNode(node, "plannersettings");
      TiXmlElement* qinit = FindSubNode(plannersettings, "qinit");
      TiXmlElement* qgoal = FindSubNode(plannersettings, "qgoal");
      TiXmlElement* se3min = FindSubNode(plannersettings, "se3min");
      TiXmlElement* se3max = FindSubNode(plannersettings, "se3max");

      stringstream(qinit->Attribute("config") ) >> pin.q_init;
      stringstream(qgoal->Attribute("config") ) >> pin.q_goal;
      stringstream(se3min->Attribute("config"))  >> pin.se3min;
      stringstream(se3max->Attribute("config"))  >> pin.se3max;

      return true;
    }

    bool LoadPlannerSettings(const char* file)
    {
      TiXmlDocument doc(file);
      TiXmlElement *root = GetRootNodeFromDocument(doc);
      LoadPlannerSettings(root);

      return true;

    }

};
