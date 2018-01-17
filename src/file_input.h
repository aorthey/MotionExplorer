#pragma once
#include <tinyxml.h>
#include <iostream>
#include <sstream>

bool CheckNodeName(TiXmlElement *node, const char* name);
TiXmlElement* GetRootNodeFromDocument(TiXmlDocument& doc);
TiXmlElement* FindSubNode(TiXmlElement* node, const char *name);
TiXmlElement* FindFirstSubNode(TiXmlElement* node, const char *name);
TiXmlElement* FindNextSiblingNode(TiXmlElement* node, const char *name);
bool ExistStreamAttribute(TiXmlElement* node, const char *name);
std::stringstream GetStreamAttribute(TiXmlElement* node, const char *name);
std::stringstream GetStreamAttributeConfig(TiXmlElement* node, const char *name);
std::stringstream GetStreamText(TiXmlElement* node);

template<typename T> 
std::stringstream GetStreamTextDefault(TiXmlElement* node, T default_value);

template<typename T> 
std::stringstream GetStreamAttributeDefault(TiXmlElement *node, const char *name, T default_value);

#include "file_input.ipp"
