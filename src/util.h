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

  void SetSimulatedRobot( Robot *robot, WorldSimulation &sim, const Config &q);
  void SetSimulatedRobot( Robot *robot, WorldSimulation &sim, const Config &q, const Config &dq);

  std::string GetApplicationFolder();
  std::string GetDataFolder();
  std::string GetFileBasename(const char *file);
  std::string GetFileBasename(const std::string& file);
  std::string GetFileExtension(const char *file);
  std::string GetFileExtension(const std::string& file);

  bool StartsWith(const std::string &str, const char* prefix);
  bool StartsWith(const std::string &str, const std::string &prefix);
  bool EndsWith(const std::string &str, const char* suffix);
  bool EndsWith(const std::string &str, const std::string &suffix);

  std::string RemoveStringBeginning(const std::string &s, const std::string &prefix);
}// namespace util

