#pragma once

#include <ctime>
#include <iostream>
#include <iomanip>
#include <Simulation/ODERobot.h>
#include <Simulation/WorldSimulation.h>
#include <boost/filesystem.hpp>

namespace util {
  std::string GetCurrentTimeString(const char* delimiter="_");
  std::string GetCurrentDateString();
  std::string GetCurrentDateTimeString();
  void PrintCurrentTime();

  void SetSimulatedRobot( Robot *robot, WorldSimulation &sim, Config &q);
  std::string GetApplicationFolder();
  std::string GetDataFolder();
  bool StartsWith(const std::string &str, const char* prefix);
  bool StartsWith(const std::string &str, const std::string &prefix);
  std::string RemoveStringBeginning(const std::string &s, const std::string &prefix);
}// namespace util

