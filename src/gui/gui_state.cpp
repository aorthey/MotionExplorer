#include "gui/gui_state.h"
#include <regex>


GUIVariable::GUIVariable(){
  name = "";
  descr = "";
  key = "NONE";
  mode = "default";
}

GUIVariable::GUIVariable(std::string name_, std::string descr_){
  name = name_;
  descr = descr_;
}
void GUIVariable::deactivate(){
  active = false;
}
void GUIVariable::activate(){
  active = true;
}

void GUIVariable::toggle(){
  if(active) active=false;
  else active=true;
}

GUIVariable::operator bool() const{
  return active;
}
bool GUIVariable::hasKey() const{
  if(key == "NONE") return false;
  else return true;
}
bool GUIVariable::operator!() const{
  return !active;
}

//******************************************************************************
GUIState::GUIState(){
  modes.push_back("default");
  mode = 0;
  if(!EMPTY_VARIABLE){
    EMPTY_VARIABLE = new GUIVariable();
  }
}
void GUIState::NextMode()
{
  if(mode < modes.size()-1) mode++;
  else mode = 0;
}
void GUIState::PreviousMode()
{
  if(mode > 0) mode--;
  else mode = modes.size()-1;
}
const char* GUIState::GetCurrentMode()
{
  return modes.at(mode).c_str();
}

bool GUIState::IsToggleable(const char* str)
{
  if(toggle_variables.find(str) != toggle_variables.end()){
    GUIVariable *v = toggle_variables[std::string(str)];
    if(v->mode == GetCurrentMode()){
      return true;
    }
  }
  return false;
}
bool GUIState::IsToggleable(const std::string &str)
{
  return IsToggleable(str.c_str());
}

GUIVariable& GUIState::operator()(const char* str){
  if ( variables.find(str) != variables.end() ) {
    GUIVariable *v = variables[std::string(str)];
    if(v->mode == GetCurrentMode()) {
      return *variables[std::string(str)];
    }
  }
  return *EMPTY_VARIABLE;

}

void GUIState::toggle(const char* str)
{
  (*this)(str).toggle();
}
void GUIState::toggle(const std::string &str)
{
  return toggle(str.c_str());
}

bool GUIState::Load(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return Load(root);
}

void GUIState::GetTextVariable(TiXmlElement *node, GUIVariable *v)
{
  v->name = GetSubNodeText<std::string>(node, "name");
  std::string default_descr = std::regex_replace(v->name, std::regex("_"), " ");
  v->descr = GetSubNodeTextDefault<std::string>(node, "descr", default_descr);
  v->key = GetSubNodeTextDefault<std::string>(node, "key", "NONE");
  v->mode = GetSubNodeTextDefault<std::string>(node, "mode", v->mode);
  v->active = GetSubNodeTextDefault<int>(node, "active", 0);
  v->value = GetSubNodeTextDefault<double>(node, "value", 0);
  v->min = GetSubNodeTextDefault<double>(node, "min", 0);
  v->max = GetSubNodeTextDefault<double>(node, "max", 0);
  v->step = GetSubNodeTextDefault<double>(node, "step", 0);

  for(uint k = 0; k < modes.size(); k++){
    if(modes.at(k) == v->mode)
      return;
  }
  modes.push_back(v->mode);
  std::cout << "new mode: " << v->mode << std::endl;
}

void GUIState::AddVariableFromNode(TiXmlElement *node)
{
  GUIVariable* v = new GUIVariable();

  std::string type = node->Value();
  if(type=="checkbox"){
    GetTextVariable(node, v);
    v->type = GUIVariable::Type::CHECKBOX;
    toggle_variables[v->name] = v;
  }else if(type=="hotkey"){
    GetTextVariable(node, v);
    v->type = GUIVariable::Type::HOTKEY;
    toggle_variables[v->name] = v;
  }else if(type=="button"){
    GetTextVariable(node, v);
    v->type = GUIVariable::Type::BUTTON;
    toggle_variables[v->name] = v;
  }else if(type=="property"){
    v->name = GetAttribute<std::string>(node, "name");
    v->key = GetAttributeDefault<std::string>(node, "key", "NONE");
    v->active = GetAttributeDefault(node, "enabled", 0);
    v->type = GUIVariable::Type::PROPERTY;
  }else if(type=="slider"){
    GetTextVariable(node, v);
    v->type = GUIVariable::Type::SLIDER;
  }else{
    std::cout << "Unknown type of variable: " << type << std::endl;
  }

  variables[v->name] = v;
}

void GUIState::AddNodeType(TiXmlElement *node, const char* type)
{
  TiXmlElement* node_state = FindSubNode(node, type);
  while(node_state!=NULL){
    AddVariableFromNode(node_state);
    node_state = FindNextSiblingNode(node_state, type);
  }
}

bool GUIState::Load(TiXmlElement *node)
{
  CheckNodeName(node, "gui");
  AddNodeType(node, "checkbox");
  AddNodeType(node, "hotkey");
  AddNodeType(node, "button");
  AddNodeType(node, "property");
  AddNodeType(node, "slider");
  return true;
}

std::ostream& operator<< (std::ostream& out, const GUIVariable& v)
{
  out << "  <state>" << std::endl;
  out << "    <name>" << v.name << "</name>" << std::endl;
  out << "    <descr>" << v.descr << "</descr>" << std::endl;
  out << "    <key>" << v.key << "</key>" << std::endl;
  out << "    <active>" << v.active << "</active>" << std::endl;
  out << "  </state>" << std::endl;
  return out;
}
std::ostream& operator<< (std::ostream& out, const GUIState& s)
{
  //get all keys and sort them lexicographically
  std::map< std::string, std::string > keymap;

  for (auto it = s.variables.begin(); it != s.variables.end(); ++it) {
    const GUIVariable* v = it->second;
    if(v->hasKey()){
      std::string s = v->key + ": " + v->descr + " (" + v->name + ")";
      keymap[v->key] = s;
    }
  }
  for (auto it = keymap.begin(); it != keymap.end(); ++it) {
    std::cout << it->second << std::endl;
  }
  return out;
}
