#pragma once
#include <tinyxml.h>
#include <iostream>
#include <vector>
#include <sstream>

inline bool CheckNodeName(TiXmlElement *node, const char* name);
inline TiXmlElement* GetRootNodeFromDocument(TiXmlDocument& doc);
inline TiXmlElement* FindSubNode(TiXmlElement* node, const char *name);
inline TiXmlElement* FindFirstSubNode(TiXmlElement* node, const char *name);
inline TiXmlElement* FindNextSiblingNode(TiXmlElement* node, const char *name);
inline TiXmlElement* FindNextSiblingNode(TiXmlElement* node);
inline bool ExistStreamAttribute(TiXmlElement* node, const char *name);
inline bool ExistSubnodeAttribute(TiXmlElement* node, const char *name, const char *attribute);
inline std::stringstream GetStreamAttributeConfig(TiXmlElement* node, const char *name);
inline int CountNumberOfSubNodes(TiXmlElement* parent, const char *name);


template<typename T> 
inline std::vector<T> GetNodeVector(TiXmlElement* node);

inline std::stringstream GetStreamText(TiXmlElement* node);
template<typename T> 
inline std::stringstream GetStreamTextDefault(TiXmlElement* node, T default_value);
template<typename T> 
inline T GetSubNodeText(TiXmlElement* node, const char *name);
template<typename T> 
inline T GetSubNodeTextDefault(TiXmlElement* node, const char *name, T default_value);

inline std::stringstream GetStreamAttribute(TiXmlElement* node, const char *attribute);
template<typename T> 
inline std::stringstream GetStreamAttributeDefault(TiXmlElement *node, const char *name, T default_value);

template<typename T> 
inline T GetAttribute(TiXmlElement* node, const char *attribute);
template<typename T> 
inline T GetAttributeDefault(TiXmlElement* node, const char *attribute, T default_value);
template<typename T> 
inline T GetSubNodeAttribute(TiXmlElement* node, const char *name, const char *attribute);
template<typename T> 
inline T GetSubNodeAttributeDefault(TiXmlElement* node, const char *name, const char *attribute, T default_value);


#include "file_input.ipp"
