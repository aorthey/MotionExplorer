#pragma once
#include <vector>
#include <map>
#include <ostream>

template<typename T>
T* ptr(T& obj){ return &obj; }

template<typename T>
T* ptr(T* obj) { return obj; }

template<typename T> 
std::ostream& operator<< (std::ostream& out, const std::vector<T> &v){
  for(uint k = 0; k < v.size(); k++){
    out << *ptr(v.at(k));
    out << (k<v.size()-1?" ":"");
  }
  return out;
}

template<typename T, typename TT> 
std::ostream& operator<< (std::ostream& out, const std::map<T,TT> &t_map){
  //out << "map[" << v.size() << "]"<< " ";
  out << "{" << std::endl;
  for(auto p : t_map){
    out << "  " << p.first << ":" << p.second << std::endl;
  }
  out << "}";
  return out;
}
typedef unsigned int uint;
