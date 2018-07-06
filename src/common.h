#pragma once
#include <vector>
#include <map>
#include <ostream>

template<typename T> 
std::ostream& operator<< (std::ostream& out, const std::vector<T> &v){
  out << "[" << v.size() << "]"<< " ";
  for(uint k = 0; k < v.size(); k++){
    out << v.at(k) << " ";
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
