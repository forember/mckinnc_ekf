#pragma once

#include <cmath>

/*! \file
 *  \brief Interchange structs and a debug output macro.
 *
 *  Length units are meters. Time units are seconds.
 *  Angle units are radians, (-&pi;, &pi], unless otherwise specified;
 */

//! Output stuff to stderr along with the filename, function, and line number.
/*!
 *  Usage: `traceStream("it's " << just << " ostream " << syntax);`
 *  Output: `filename.cpp:callingFunction:66: it's 3 ostream [3](0.25 0.5 0.75)`
 *  A newline is automatically appended.
 *
 *  I know, ew, a macro, but only the preprocessor knows line numbers.
 *  Doesn't work on Windows, but with the state of ROS on Windows not under WSL
 *  (this macro does work under WSL), this macro is probably the least of your
 *  concerns.
 *  Sorry if you're POSIX but not UNIX, just remove the check and warning.
 *  Also email me (maestro@tachibanatech.com). I'm curious what you're running.
 */
#if defined(NDEBUG) || !defined(__unix__)
#if defined(NDEBUG) && !defined(__unix__)
#pragma GCC warning "Unsupported OS: Making traceStream a no-op."
#endif
#define traceStream(streamOps) ((void)0)
#else
#include <iostream>
#include <string>
#define traceStream(streamOps) { \
  std::string path(__FILE__); \
  std::cerr << path.substr(path.find_last_of('/') + 1) << ':' << __func__ \
    << ':' << __LINE__ << ": " << streamOps << "\n"; \
}
#endif

namespace mckinnc_ekf {

//! Reduce an angle to be within (-pi, pi].
/*!
 *  Note: this function is very small, so it is statically compiled into every
 *  compilation unit that includes this header, directly or indirectly.
 *  The compiler's optimizer will remove it if you don't use it.
 */
static double reduceAngle(double angle)
{   
  return atan2(sin(angle), cos(angle));
}

//! Control input to EKF.
struct Control {
  double linear;
  double angular;
  //! Time since the most recently processed control input.
  double timeDelta;
};

//! Landmark for EKF localization.
struct Landmark {
  double x;
  double y;
  //! For a corner landmark, inner angle of the corner, (0, 2&pi;).
  double signature;
};

//! Raw observation of a landmark.
struct Observation {
  double range;
  double angle;
  //! \see Landmark::signature
  double signature;
};

//! Robot position in map coordinates.
struct Pose {
  double x;
  double y;
  double heading;
};

} // namespace
