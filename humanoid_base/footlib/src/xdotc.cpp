//
// File: xdotc.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 24-Aug-2023 18:26:44
//

// Include Files
#include "xdotc.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const double x[4]
//                const double y[4]
// Return Type  : double
//
namespace coder {
namespace internal {
namespace blas {
double xdotc(const double x[4], const double y[4])
{
  return x[0] * y[2] + x[1] * y[3];
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xdotc.cpp
//
// [EOF]
//
