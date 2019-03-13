#pragma once

#include <tinyxml.h>
#include <vector>

//INSERTION OF ELEMENTS HAS TO BE DONE FIRST BY INSERTING IN THE DEEPEST
//ELEMENT, THEN GOING UPWARDS. 
//
// Example:
// TiXmlElement root;
// TiXmlElement subnode;
// TiXmlElement subsubnode;
// 
// WORKS:
// AddSubNode(subnode, "name", "sub");
// node->insertendchild(subnode);
//

inline TiXmlElement* CreateRootNodeInDocument(TiXmlDocument& doc, const char *name = "default");

inline void AddComment(TiXmlElement& node, const char *str = "");

template <typename T>
inline TiXmlElement CreateSubNode(const char *name, T _val);
template <typename T>
inline void AddSubNode(TiXmlElement& node, const char *name, T _val);
template <typename T>
inline void AddSubNodeBeginning(TiXmlElement& node, const char *name, T _val);

template <typename T>
inline void AddSubNodeVector(TiXmlElement& node, const char *name, std::vector<T> _val);

template <typename T>
inline TiXmlElement* ReturnSubNodeVector(TiXmlElement& node, const char *name, std::vector<T> _val);
#include "file_output.ipp"
