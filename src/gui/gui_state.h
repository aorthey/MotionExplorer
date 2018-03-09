#pragma once
#include "file_io.h"
#include <string>
#include <vector>
#include <map>
#include <iostream>

struct GUIVariable
{
  enum Type{CHECKBOX, HOTKEY, BUTTON, PROPERTY, SLIDER};

  GUIVariable();
  GUIVariable(std::string);
  GUIVariable(std::string, std::string);

  void deactivate();
  void activate();
  void toggle();

  operator bool() const;
  bool operator!() const;

  bool hasKey() const;

  friend std::ostream& operator<< (std::ostream&, const GUIVariable&);

  Type type;
  int active;
  std::string name;
  std::string descr;
  std::string key;
  std::string mode;
  double value{0.0};
  double min{0.0};
  double max{0.0};
  double step{0.0};
};

class GUIState{

  typedef std::map<std::string, GUIVariable*> GUIVariableMap;

  public:
    GUIState();
    GUIVariable& operator()(const char* str);

    void GetTextVariable(TiXmlElement *node, GUIVariable *v);
    void AddVariableFromNode(TiXmlElement *node);
    void AddNodeType(TiXmlElement *node, const char* type);

    void toggle(const char* str);
    void toggle(const std::string &str);

    bool IsToggleable(const char* str);
    bool IsToggleable(const std::string &str);

    bool Load(const char* file);
    bool Load(TiXmlElement *node);

    void NextMode();
    void PreviousMode();
    const char* GetCurrentMode();

    GUIVariableMap variables;
    GUIVariableMap toggle_variables;

    std::vector<std::string> modes;
    uint mode;

    GUIVariable *EMPTY_VARIABLE;

    friend std::ostream& operator<< (std::ostream&, const GUIState&);

};

