#pragma once

#include <tinyxml.h>
#include <vector>

inline TiXmlElement* CreateRootNodeInDocument(TiXmlDocument& doc, const char *name = "default");

inline void AddComment(TiXmlElement& node, const char *str = "");

template <typename T>
inline void AddSubNode(TiXmlElement& node, const char *name, T _val);

template <typename T>
inline void AddSubNodeVector(TiXmlElement& node, const char *name, std::vector<T> _val);

#include "file_output.ipp"
