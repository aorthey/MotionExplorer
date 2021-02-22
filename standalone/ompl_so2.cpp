#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>


namespace ob = ompl::base;

namespace ompl{
  namespace base{
    class SO2StateSpaceAwesomeInterpolate : public SO2StateSpace
    {
      public:
      virtual void interpolate(const State *from, const State *to, double t, State *state) const override
      {
        const double &pi = boost::math::constants::pi<double>();
        const double &v_from = from->as<StateType>()->value;
        const double &v_to = to->as<StateType>()->value;
        double &v_result = state->as<StateType>()->value;

        double diff = v_to - v_from;
        if(fabs(diff) <= pi){
          v_result = v_from + diff * t;
        }else{
          if (diff > 0.0) diff = +2.0*pi - diff;
          else diff = -2.0*pi - diff;
          v_result = v_from - diff * t;
        }
        while(v_result < -pi) v_result += 2.0*pi;
        while(v_result > +pi) v_result -= 2.0*pi;
      }

    };

  }
}
namespace ompl{
  namespace base{
    class SO2StateSpaceCmplxInterpolate : public SO2StateSpace
    {

      public:
      virtual void interpolate(const State *from, const State *to, double t, State *state) const override
      {
        const double pi = boost::math::constants::pi<double>();
        const double &v_from = from->as<StateType>()->value;
        const double &v_to = to->as<StateType>()->value;
        double &v_result = state->as<StateType>()->value;

        double diff = v_to - v_from;
        const std::complex<double> i(0, 1);
        std::complex<double> d_cmplx;

        if(fabs(diff) <= pi){
          d_cmplx = std::exp(i*(v_from+t*(diff)));
        }else{
          if (diff > 0.0) diff = +2.0*pi - diff;
          else diff = -2.0*pi - diff;
          d_cmplx = std::exp(i*(v_from-t*(diff)));
        }
        v_result = std::log(d_cmplx).imag();
      }

    };

  }
}

int main(int argc,const char** argv)
{
  auto space(std::make_shared<ob::SO2StateSpaceAwesomeInterpolate>());
  auto spaceCmplx(std::make_shared<ob::SO2StateSpaceCmplxInterpolate>());

  ob::State* s1 = space->allocState();
  ob::State* s2 = space->allocState();
  ob::State* s3_new = space->allocState();
  ob::State* s3_orig = space->allocState();

  double &d1 = s1->as<ob::SO2StateSpace::StateType>()->value;
  double &d2 = s2->as<ob::SO2StateSpace::StateType>()->value;
  double &d3_new = s3_new->as<ob::SO2StateSpace::StateType>()->value;
  double &d3_orig = s3_orig->as<ob::SO2StateSpace::StateType>()->value;


  uint Msamples = 10000;
  double epsilon = 1e-10;
  uint numberTests = 4;
  ompl::RNG rng;

  //############################################################################
  //Test 1: compare values on [0,1]
  //############################################################################
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Test 1/" << numberTests << ": Compare output on [0,1] interpolation step." << std::endl;
  ob::StateSamplerPtr sampler = space->allocDefaultStateSampler();

  for(uint k = 0; k < Msamples; k++){
    sampler->sampleUniform(s1);
    sampler->sampleUniform(s2);
    double interp_step = rng.uniform01();
    space->interpolate(s1, s2, interp_step, s3_new);
    space->SO2StateSpace::interpolate(s1, s2, interp_step, s3_orig);

    if(fabs(d3_new - d3_orig) > epsilon)
    {
      std::cout << "Error on interpolation from " << d1 << " to " << d2 << " (interpolation step=" << interp_step << ")" << std::endl;
      std::cout << "s3 (new)  : " << d3_new << std::endl;
      std::cout << "s3 (orig) : " << d3_orig << std::endl;
      std::cout << "distance: " << fabs(d3_new - d3_orig) << std::endl;
      exit(0);
    }
  }
  std::cout << "OK" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  //############################################################################
  //Test 2: compare time to execute
  //############################################################################
  Msamples = 1e6;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Test 2/" << numberTests << ": Compare Execution Times." << std::endl;
  typedef ompl::time::point TimePoint;
  typedef ompl::time::duration TimeDuration;

  TimePoint t1 = ompl::time::now();
  for(uint k = 0; k < Msamples; k++){
    sampler->sampleUniform(s1);
    sampler->sampleUniform(s2);
    double interp_step = rng.uniform01();
    space->interpolate(s1, s2, interp_step, s3_new);
  }
  TimePoint t2 = ompl::time::now();
  for(uint k = 0; k < Msamples; k++){
    sampler->sampleUniform(s1);
    sampler->sampleUniform(s2);
    double interp_step = rng.uniform01();
    space->SO2StateSpace::interpolate(s1, s2, interp_step, s3_orig);
  }
  TimePoint t3 = ompl::time::now();
  TimeDuration t_new = t2-t1;
  TimeDuration t_orig = t3-t2;
  std::cout << "New      time for execution: " << ompl::time::seconds(t_new) << " s" << std::endl;
  std::cout << "Original time for execution: " << ompl::time::seconds(t_orig)<< " s" <<  std::endl;
  std::cout << "OK" << std::endl;
  std::cout << std::string(80, '-') << std::endl;


  //############################################################################
  //Test 3: compare on larger Interpolation Steps
  //############################################################################
  Msamples = 1e6;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Test 3/" << numberTests << ": Check that interpolation steps outside [0,1] are correctly executed." << std::endl;
  for(uint k = 0; k < Msamples; k++){
    sampler->sampleUniform(s1);
    sampler->sampleUniform(s2);
    double interp_step = 1.0+rng.uniformReal(-3,2);
    space->interpolate(s1, s2, interp_step, s3_new);

    spaceCmplx->interpolate(s1, s2, interp_step, s3_orig);
    double d_result = d3_orig;

    double diff_out = fabs(d3_new - d_result);
    if(fabs(diff_out)>1e-10){
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "ERROR" << std::endl;
      std::cout << "d1=" << d1 << ",d2="<< d2 << ",diff=" << fabs(d2-d1) << ",d3=" << d3_new << " =/= " << d_result << " (interpol: " << interp_step << ")" << std::endl;
      std::cout << "diff_out: " << diff_out << std::endl;
      exit(0);
    }
  }
  std::cout << "OK" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  ////############################################################################
  ////Test 4: compare on crazy interpolation values
  ////############################################################################
  //Msamples = 1e2;
  //std::cout << std::string(80, '-') << std::endl;
  //std::cout << "Test 4/" << numberTests << ": Check crazy interpolation steps outside [0,1]." << std::endl;

  //double nan  = std::numeric_limits<double>::quiet_NaN();
  //double inf = std::numeric_limits<double>::infinity();

  //std::vector<double> crazyValues{1e80, nan, inf};
  //for(uint i = 0; i < crazyValues.size(); i++){
  //  double interp_step = crazyValues.at(i);
  //  for(uint k = 0; k < Msamples; k++){
  //    sampler->sampleUniform(s1);
  //    sampler->sampleUniform(s2);
  //    space->interpolate(s1, s2, interp_step, s3_new);
  //    spaceCmplx->interpolate(s1, s2, interp_step, s3_orig);
  //    double d_result = d3_orig;

  //    double diff_out = fabs(d3_new - d_result);
  //    if(fabs(diff_out)>1e-10){
  //      std::cout << std::string(80, '-') << std::endl;
  //      std::cout << "ERROR" << std::endl;
  //      std::cout << "d1=" << d1 << ",d2="<< d2 << ",diff=" << fabs(d2-d1) << ",d3=" << d3_new << " =/= " << d_result << " (interpol: " << interp_step << ")" << std::endl;
  //      std::cout << "diff_out: " << diff_out << std::endl;
  //      exit(0);
  //    }
  //  }
  //  std::cout << "OK for " << crazyValues.at(i) << std::endl;
  //}
  //std::cout << "OK" << std::endl;
  //std::cout << std::string(80, '-') << std::endl;

  space->freeState(s1);
  space->freeState(s2);
  space->freeState(s3_orig);
  space->freeState(s3_new);
  return 0;
}
