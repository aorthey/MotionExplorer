#include <vector>
#include <ostream>

template<typename T> 
std::ostream& operator<< (std::ostream& out, const std::vector<T> &v){
  out << v.size() << "  ";
  for(uint k = 0; k < v.size(); k++){
    out << v.at(k) << " ";
  }
  out << std::endl;
  return out;
}
