#include <vector>
#include <iostream>
#include <boost/type_traits/is_pointer.hpp>

template<typename T>
T * ptr(T & obj) { return &obj; } //turn reference into pointer!

template<typename T>
T * ptr(T * obj) { return obj; } //obj is already pointer, return it!

template<typename T> 
std::ostream& operator<< (std::ostream& out, const std::vector<T> &v){
  //out << "[" << v.size() << "]"<< " ";
  const bool T_is_ptr = boost::is_pointer<T>::value;
  std::cout << (T_is_ptr?"IsPointer":"NotAPointer") << std::endl;
  for(uint k = 0; k < v.size(); k++){
    out << *ptr(v.at(k));
    //out << v.at(k);
    out << (k<v.size()-1?" ":"");
  }
  return out;
}

struct MyDouble{
  double d;
  friend std::ostream& operator<< (std::ostream& out, const MyDouble &md)
  {
    out << "[" << md.d << "]";
    return out;
  }
};

int main(int argc,const char** argv)
{
  std::vector<double> A;
  double dstep = 0.01;
  for(double d = 0; d < 0.1; d+=dstep){
    A.push_back(d);
  }

  std::vector<MyDouble*> B;
  for(double d = 0; d < 0.1; d+=dstep){
    MyDouble *md = new MyDouble();
    md->d = d;
    B.push_back(md);
  }
  std::cout << A << std::endl;
  std::cout << B << std::endl;


  return 0;
}
