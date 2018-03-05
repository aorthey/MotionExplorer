#include "file_input.h"

using namespace std;

bool CheckNodeName(TiXmlElement *node, const char* name)
{
  if(0!=strcmp(node->Value(),name)) {
    std::cout << "Not a " << name <<  " file" << std::endl;
    return false;
  }
  return true;
}
TiXmlElement* GetRootNodeFromDocument(TiXmlDocument& doc)
{
  if(doc.LoadFile()){
    TiXmlElement *root = doc.RootElement();
    if(root){
      return root;
    }
  }else{
    std::cout << doc.ErrorDesc() << std::endl;
  }
  return NULL;
}

TiXmlElement* FindSubNode(TiXmlElement* node, const char *name){
  if(!node) return NULL;
  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) 
  {
    if(0==strcmp(e->Value(),name)) return e; 
    e = e->NextSiblingElement();
  }
  return NULL;
}

TiXmlElement* FindFirstSubNode(TiXmlElement* node, const char *name){
  return FindSubNode(node, name);
}

TiXmlElement* FindNextSiblingNode(TiXmlElement* node){
  return FindNextSiblingNode(node, node->Value());
}
TiXmlElement* FindNextSiblingNode(TiXmlElement* node, const char *name){
  while(node != NULL) 
  {
    node = node->NextSiblingElement();
    if(node!=NULL){
      if(0==strcmp(node->Value(),name)) return node; 
    }
  }
  return NULL;
}

bool ExistStreamAttribute(TiXmlElement* node, const char *name){
  if(!node) return false;
  const char *na = node->Attribute(name);
  if(na){
    return true;
  }else{
    return false;
  }
}

stringstream GetStreamAttributeConfig(TiXmlElement* node, const char *name){

  if(!node) return stringstream ("NONE");
  const char *na = node->Attribute(name);
  if(na){
    //safety check for config
//  int n;
//  in >> n;
//  if(!in) return in;
//  if(n != v.n)
//    v.resize(n);
//  for(int i=0; i<v.n; i++)
//    in >> v[i];
//  return in;
//}

    return stringstream (na);
  }else{
    return stringstream ("NONE");
  }
}

stringstream GetStreamText(TiXmlElement* node){

  if(!node) return stringstream ("NONE");
  const char *na = node->GetText();
  if(na){
    return stringstream (na);
  }else{
    return stringstream ("NONE");
  }
}
template<typename T> 
stringstream GetStreamTextDefault(TiXmlElement* node, T default_value){
  std::stringstream ss;
  ss = GetStreamText(node);
  if(ss.str() != "NONE"){
    return ss;
  }else{
    std::stringstream def;
    def << default_value;
    return def;
  }
}

stringstream GetStreamAttribute(TiXmlElement* node, const char *attribute)
{
  if(!node) return stringstream ("NONE");
  const char *na = node->Attribute(attribute);
  if(na){
    return stringstream (na);
  }else{
    return stringstream ("NONE");
  }
}
template<typename T> 
stringstream GetStreamAttributeDefault(TiXmlElement *node, const char *name, T default_value){
  if(ExistStreamAttribute(node, name)){
    return GetStreamAttribute(node, name);
  }
  stringstream ss;
  ss << default_value;
  return ss;
}


template<typename T> 
inline std::vector<T> GetNodeVector(TiXmlElement* node)
{
  std::stringstream ss = GetStreamText(node);
  uint _size;
  ss >> _size;

  std::vector<T> _val;
  _val.resize(_size);
  for(uint k = 0; k < _size; k++){
    ss >> _val.at(k);
  }
  return _val;
}

template<typename T> 
T GetSubNodeText(TiXmlElement* node, const char *name)
{
  TiXmlElement* subnode = FindSubNode(node, name);
  if(!subnode){
    std::cout << "[ERROR] Subnode: " << name << " has not been found." << std::endl;
    exit(0);
  }
  T _val;
  GetStreamText(subnode) >> _val;
  return _val;
}
template<typename T> 
T GetSubNodeTextDefault(TiXmlElement* node, const char *name, T default_value)
{
  TiXmlElement* subnode = FindSubNode(node, name);
  if(!subnode){
    std::cout << "[WARNING] Subnode: " << name << " has not been found. Setting to " << default_value << std::endl;
  }
  T _val;
  GetStreamTextDefault<T>(subnode, default_value) >> _val;
  return _val;
}
template<typename T> 
inline T GetSubNodeAttribute(TiXmlElement* node, const char *name, const char *attribute)
{
  TiXmlElement* subnode = FindSubNode(node, name);
  if(!subnode){
    std::cout << "[ERROR] Subnode: " << name << " has not been found." << std::endl;
    exit(0);
  }

  T _val;
  GetStreamAttribute(subnode, attribute) >> _val;
  return _val;
}
template<typename T> 
inline T GetSubNodeAttributeDefault(TiXmlElement* node, const char *name, const char *attribute, T default_value)
{
  TiXmlElement* subnode = FindSubNode(node, name);
  if(!subnode){
    std::cout << "[WARNING] Subnode: " << name << " has not been found. Setting to " << default_value << std::endl;
  }
  T _val;
  GetStreamAttributeDefault(subnode, attribute, default_value) >> _val;
  return _val;
}
template<typename T> 
inline T GetAttribute(TiXmlElement* node, const char *attribute)
{
  T _val;
  GetStreamAttribute(node, attribute) >> _val;
  return _val;
}

template<typename T> 
inline T GetAttributeDefault(TiXmlElement* node, const char *attribute, T default_value)
{
  T _val;
  GetStreamAttributeDefault(node, attribute, default_value) >> _val;
  return _val;
}

