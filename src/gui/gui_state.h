#pragma once
#include "file_io.h"
#include <string>
#include <vector>
#include <map>
#include <iostream>

struct GUIVariable
{
  enum Type{CHECKBOX, BUTTON, HOTKEY, PROPERTY, SLIDER};

  GUIVariable();
  GUIVariable(std::string);
  GUIVariable(std::string, std::string);

  int active;

  std::string key;
  std::string name;
  std::string descr;

  void deactivate();
  void activate();
  void toggle();

  operator bool() const;
  bool operator!() const;

  bool hasKey() const;

  Type type;

  friend std::ostream& operator<< (std::ostream&, const GUIVariable&);

  double value{0.0};
  double min{0.0};
  double max{0.0};
  double step{0.0};
};

class GUIState{

  public:
    GUIState();
    void add(const char* name);
    void add(const char* name, char* key);
    GUIVariable& operator()(const char* str);
    void toggle(const char* str);

    friend std::ostream& operator<< (std::ostream&, const GUIState&);

    bool Load(const char* file);
    bool Load(TiXmlElement *node);

    std::map<std::string, GUIVariable*> variables;

    GUIVariable *EMPTY_VARIABLE;
};

