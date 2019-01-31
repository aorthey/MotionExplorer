#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>


namespace ob = ompl::base;
static const double MAX_QUATERNION_NORM_ERROR = 1e-9;

namespace ompl
{
    namespace base
    {
        static inline double arcLength(const State *state1, const State *state2)
        {
            const auto *qs1 = static_cast<const SO3StateSpace::StateType *>(state1);
            const auto *qs2 = static_cast<const SO3StateSpace::StateType *>(state2);
            double dq = fabs(qs1->x * qs2->x + qs1->y * qs2->y + qs1->z * qs2->z + qs1->w * qs2->w);
            if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
                return 0.0;
            return acos(dq);
        }

        static inline void quaternionProduct(SO3StateSpace::StateType &q, const SO3StateSpace::StateType &q0,
                                     const SO3StateSpace::StateType &q1)
        {
            q.x = q0.w * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y;
            q.y = q0.w * q1.y + q0.y * q1.w + q0.z * q1.x - q0.x * q1.z;
            q.z = q0.w * q1.z + q0.z * q1.w + q0.x * q1.y - q0.y * q1.x;
            q.w = q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z;
        }
        static inline void QuaternionMultiplication(SO3StateSpace::StateType &q, const SO3StateSpace::StateType &q0,
                                     const SO3StateSpace::StateType &q1)
        {
          //Overall function: q = q1 * inverse(q0) * q1;
            //Step (1): q = inverse(q0) * q1;
            q.x = q0.w * q1.x + (-1)*q0.x * q1.w + (-1)*q0.y * q1.z - (-1)*q0.z * q1.y;
            q.y = q0.w * q1.y + (-1)*q0.y * q1.w + (-1)*q0.z * q1.x - (-1)*q0.x * q1.z;
            q.z = q0.w * q1.z + (-1)*q0.z * q1.w + (-1)*q0.x * q1.y - (-1)*q0.y * q1.x;
            q.w = q0.w * q1.w - (-1)*q0.x * q1.x - (-1)*q0.y * q1.y - (-1)*q0.z * q1.z;

            //Step (2): q = q1 * q
            quaternionProduct(q, q1, q);
            
        }

    }  // namespace base
}  // namespace ompl


namespace ompl{
  namespace base{
    class SO3StateSpaceAwesomeInterpolate : public SO3StateSpace
    {
      public:
      virtual void interpolate(const State *from, const State *to, double t, State *state) const override
      {
        // assert(fabs(norm(static_cast<const StateType *>(from)) - 1.0) < MAX_QUATERNION_NORM_ERROR);
        // assert(fabs(norm(static_cast<const StateType *>(to)) - 1.0) < MAX_QUATERNION_NORM_ERROR);


        //quaternionProduct(q, q0, q1);



// quat slerp_generic(quat q0, quat q1, float t)
// {
//     // If t is too large, divide it by two recursively
//     if (t > 1.0)
//     {
//         quat tmp = slerp_generic(q0, q1, t / 2);
//         return tmp * inverse(q0) * tmp;
//     }

//     // It’s easier to handle negative t this way
//     if (t < 0.0)
//         return slerp_generic(q1, q0, 1.0 - t);

//     return slerp(q0, q1, t);
// }
        //double theta = arcLength(from, to);
        if (t > 1.0)
        {
          SO3StateSpaceAwesomeInterpolate::interpolate(from, to, t/2.0, state);

          const SO3StateSpace::StateType *q0 = from->as<ob::SO3StateSpace::StateType>();
          //const SO3StateSpace::StateType *q1 = to->as<ob::SO3StateSpace::StateType>();
          SO3StateSpace::StateType *q = state->as<ob::SO3StateSpace::StateType>();
          //return state * inverse(from) * state;
          return QuaternionMultiplication(*q, *q0, *q);
        }
        if(t < 0.0){
          SO3StateSpaceAwesomeInterpolate::interpolate(from, to, 1.0 - t, state);
        }

        SO3StateSpace::interpolate(from, to, t, state);

        //if (theta > std::numeric_limits<double>::epsilon())
        //{
        //    const auto *qs1 = static_cast<const StateType *>(from);
        //    const auto *qs2 = static_cast<const StateType *>(to);
        //    auto *qr = static_cast<StateType *>(state);
        //    double dot = qs1->x * qs2->x + qs1->y * qs2->y + qs1->z * qs2->z + qs1->w * qs2->w;

        //    // If the dot product is negative, slerp won't take
        //    // the shorter path. Note that v1 and -v1 are equivalent when
        //    // the negation is applied to all four components. Fix by
        //    // reversing one quaternion.
        //    int sign = 1;
        //    if (dot < 0) {
        //        //v1 = -v1;
        //        sign = -1;
        //        dot = -dot;
        //    }

        //    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
        //    double theta = acos(dot);        // theta_0 = angle between input vectors
        //    //double theta = theta_0*t;          // theta = angle between v0 and result
        //    double sin_theta = sin(theta*t);     // compute this value only once
        //    double sin_theta_0 = sin(theta); // compute this value only once

        //    double s0 = cos(t*theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
        //    double s1 = sin_theta / sin_theta_0;

        //    s0 *= sign;

        //    //return (s0 * v0) + (s1 * v1);
        //    qr->x = (qs1->x * s0 + qs2->x * s1);
        //    qr->y = (qs1->y * s0 + qs2->y * s1);
        //    qr->z = (qs1->z * s0 + qs2->z * s1);
        //    qr->w = (qs1->w * s0 + qs2->w * s1);



        //}
        //else
        //{
        //  if (state != from) copyState(state, from);
        //}

      }

    };

  }
}

int main(int argc,const char** argv)
{
  //############################################################################
  //INIT
  //############################################################################
  auto space(std::make_shared<ob::SO3StateSpaceAwesomeInterpolate>());

  ob::State* s1 = space->allocState();
  ob::State* s2 = space->allocState();
  ob::State* s3_new = space->allocState();
  ob::State* s3_orig = space->allocState();

  uint Msamples = 10000;
  uint numberTests = 2;
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
    space->SO3StateSpace::interpolate(s1, s2, interp_step, s3_orig);

    if(!space->equalStates(s3_new, s3_orig))
    {
      //NOTE: s3_new == s3_orig compares the pointers!
      std::cout << "On sample " << k << "/" << Msamples << std::endl;
      std::cout << "Error on interpolation (interpolation step=" << interp_step << ")" << std::endl;
      space->printState(s3_new, std::cout);
      space->printState(s3_orig, std::cout);
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
    space->SO3StateSpace::interpolate(s1, s2, interp_step, s3_orig);
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
  Msamples = 1e2;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Test 3/" << numberTests << ": Check that interpolation steps outside [0,1] are correctly executed." << std::endl;
  for(uint k = 0; k < Msamples; k++){
    sampler->sampleUniform(s1);
    sampler->sampleUniformNear(s2, s1, 0.5);
    double interp_step = 1.0+rng.uniformReal(0.5,1);

    space->interpolate(s1, s2, interp_step, s3_new);
    space->SO3StateSpace::interpolate(s1, s2, interp_step, s3_orig);

    double d12 = space->distance(s1, s2);
    double d23 = space->distance(s1, s3_orig);

    //if(!space->equalStates(s3_new, s3_orig))
    if( fabs(d23 - interp_step*d12) > 1e-10)
    {
      //NOTE: s3_new == s3_orig compares the pointers!
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "distance s1-s2     : " << space->distance(s1,s2) << std::endl;
      std::cout << "distance s2-s3_new : " << space->distance(s2,s3_new) << std::endl;
      std::cout << "distance s2-s3_orig: " << space->distance(s2,s3_orig) << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "distance s1-s3_new : " << space->distance(s1,s3_new) << std::endl;
      std::cout << "distance s1-s3_orig: " << space->distance(s1,s3_orig) << std::endl;
      std::cout << "Error on interpolation (interpolation step=" << interp_step << ")" << std::endl;
      std::cout << "On sample " << k << "/" << Msamples << std::endl;
      space->printState(s3_new, std::cout);
      space->printState(s3_orig, std::cout);
      exit(0);
    }
  }
  std::cout << "OK" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  //############################################################################
  //END
  //############################################################################
  space->freeState(s1);
  space->freeState(s2);
  space->freeState(s3_orig);
  space->freeState(s3_new);
  return 0;
}
