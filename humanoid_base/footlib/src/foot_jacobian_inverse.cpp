//
// File: foot_jacobian_inverse.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 24-Aug-2023 18:26:44
//

// Include Files
#include "foot_jacobian_inverse.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include <cmath>
#include <cstring>
#include <math.h>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : double roll
//                double pitch
//                double invJ[4]
// Return Type  : void
//
void foot_jacobian_inverse(double roll, double pitch, double invJ[4])
{
  double J[4];
  double J_tmp;
  double a_tmp;
  double a_tmp_tmp;
  double absx;
  double b_J_tmp;
  double b_a_tmp;
  double b_a_tmp_tmp;
  double b_et1_tmp_tmp;
  double b_unnamed_idx_1_tmp;
  double b_unnamed_idx_2_tmp;
  double b_unnamed_idx_3_tmp;
  double c_J_tmp;
  double c_a_tmp;
  double c_a_tmp_tmp;
  double c_et1_tmp_tmp;
  double c_unnamed_idx_1_tmp;
  double d_J_tmp;
  double d_a_tmp;
  double d_a_tmp_tmp;
  double e_J_tmp;
  double e_a_tmp;
  double e_a_tmp_tmp;
  double et1_tmp_tmp;
  double f_a_tmp;
  double f_a_tmp_tmp;
  double g_a_tmp;
  double g_a_tmp_tmp;
  double h_a_tmp;
  double h_a_tmp_tmp;
  double i_a_tmp;
  double j_a_tmp;
  double k_a_tmp;
  double l_a_tmp;
  double m_a_tmp;
  double n_a_tmp;
  double o_a_tmp;
  double p_a_tmp;
  double q_a_tmp;
  double unnamed_idx_0_tmp;
  double unnamed_idx_1_tmp;
  double unnamed_idx_2_tmp;
  double unnamed_idx_3_tmp;
  int exponent;
  bool p;
  // FOOT_JACOBIAN
  //     J = FOOT_JACOBIAN(ROLL,PITCH)
  //     This function was generated by the Symbolic Math Toolbox version 9.1.
  //     24-Aug-2023 16:58:59
  et1_tmp_tmp = std::cos(roll);
  absx = std::sin(roll);
  b_et1_tmp_tmp = std::cos(pitch);
  c_et1_tmp_tmp = std::sin(pitch);
  a_tmp_tmp = et1_tmp_tmp * 16.5;
  b_a_tmp_tmp = absx * 16.0;
  a_tmp = a_tmp_tmp + b_a_tmp_tmp;
  c_a_tmp_tmp = c_et1_tmp_tmp * a_tmp;
  d_a_tmp_tmp = b_et1_tmp_tmp * 47.2;
  b_a_tmp = d_a_tmp_tmp + c_a_tmp_tmp;
  e_a_tmp_tmp = b_et1_tmp_tmp * a_tmp;
  f_a_tmp_tmp = e_a_tmp_tmp * 100.0;
  g_a_tmp_tmp = c_et1_tmp_tmp * -4720.0;
  c_a_tmp = (g_a_tmp_tmp + f_a_tmp_tmp) + 23200.0;
  d_a_tmp = c_et1_tmp_tmp * -47.2;
  e_a_tmp = (d_a_tmp + e_a_tmp_tmp) + 232.0;
  f_a_tmp = absx * 16.5;
  g_a_tmp = (et1_tmp_tmp * -16.0 + f_a_tmp) + 16.0;
  h_a_tmp_tmp = 0.0 - c_et1_tmp_tmp * 4720.0;
  h_a_tmp = ((h_a_tmp_tmp + f_a_tmp_tmp) + c_a_tmp_tmp * 0.0) + 23200.0;
  i_a_tmp = b_et1_tmp_tmp * 4720.0;
  j_a_tmp = i_a_tmp - c_a_tmp_tmp * -100.0;
  k_a_tmp = a_tmp_tmp - b_a_tmp_tmp;
  a_tmp_tmp = c_et1_tmp_tmp * k_a_tmp;
  l_a_tmp = d_a_tmp_tmp + a_tmp_tmp;
  b_a_tmp_tmp = b_et1_tmp_tmp * k_a_tmp;
  d_a_tmp_tmp = b_a_tmp_tmp * 100.0;
  m_a_tmp = (g_a_tmp_tmp + d_a_tmp_tmp) + 29000.0;
  d_a_tmp = (d_a_tmp + b_a_tmp_tmp) + 290.0;
  absx = et1_tmp_tmp * 16.0;
  f_a_tmp_tmp = absx + f_a_tmp;
  n_a_tmp = ((h_a_tmp_tmp + d_a_tmp_tmp) + a_tmp_tmp * 0.0) + 29000.0;
  o_a_tmp = i_a_tmp - a_tmp_tmp * -100.0;
  d_a_tmp_tmp = b_a_tmp * b_a_tmp;
  p_a_tmp = ((d_a_tmp_tmp + e_a_tmp * e_a_tmp) + g_a_tmp * g_a_tmp) - 51324.0;
  g_a_tmp_tmp = l_a_tmp * l_a_tmp;
  q_a_tmp = ((g_a_tmp_tmp + d_a_tmp * d_a_tmp) +
             (f_a_tmp_tmp - 16.0) * (f_a_tmp_tmp - 16.0)) -
            81600.0;
  et1_tmp_tmp = c_et1_tmp_tmp * f_a_tmp_tmp;
  unnamed_idx_0_tmp = b_et1_tmp_tmp * f_a_tmp_tmp;
  unnamed_idx_1_tmp = absx - f_a_tmp;
  b_unnamed_idx_1_tmp = c_et1_tmp_tmp * unnamed_idx_1_tmp;
  c_unnamed_idx_1_tmp = b_et1_tmp_tmp * unnamed_idx_1_tmp;
  absx = c_et1_tmp_tmp * 47.2;
  unnamed_idx_2_tmp = l_a_tmp * (absx - b_a_tmp_tmp) * 2.0;
  b_unnamed_idx_2_tmp = a_tmp_tmp * 100.0;
  unnamed_idx_3_tmp = b_a_tmp * (absx - e_a_tmp_tmp) * 2.0;
  b_unnamed_idx_3_tmp = c_a_tmp_tmp * 100.0;
  J_tmp = n_a_tmp * n_a_tmp;
  g_a_tmp_tmp += m_a_tmp * m_a_tmp / 10000.0;
  b_J_tmp = 1.0 / std::sqrt(g_a_tmp_tmp);
  c_J_tmp = 1.0 / rt_powd_snf(g_a_tmp_tmp, 1.5);
  d_J_tmp = q_a_tmp * -0.005;
  e_J_tmp = o_a_tmp * o_a_tmp + J_tmp;
  J[0] = ((0.0 - et1_tmp_tmp * 100.0) / n_a_tmp +
          (unnamed_idx_0_tmp * 100.0 - et1_tmp_tmp * 0.0) * o_a_tmp / J_tmp) /
             e_J_tmp * J_tmp +
         1.0 / std::sqrt(q_a_tmp * q_a_tmp * -0.0001 / g_a_tmp_tmp + 1.0) *
             (b_J_tmp * (((k_a_tmp * (f_a_tmp_tmp - 16.0) * -2.0 +
                           unnamed_idx_0_tmp * d_a_tmp * 2.0) +
                          c_et1_tmp_tmp * l_a_tmp * f_a_tmp_tmp * 2.0) /
                         100.0) +
              c_J_tmp *
                  (unnamed_idx_0_tmp * m_a_tmp / 50.0 +
                   std::sin(pitch) *
                       (std::cos(pitch) * 47.2 +
                        std::sin(pitch) *
                            (std::cos(roll) * 16.5 - std::sin(roll) * 16.0)) *
                       (std::cos(roll) * 16.0 + std::sin(roll) * 16.5) * 2.0) *
                  d_J_tmp);
  invJ[0] = 0.0;
  c_a_tmp_tmp = h_a_tmp * h_a_tmp;
  absx = d_a_tmp_tmp + c_a_tmp * c_a_tmp / 10000.0;
  et1_tmp_tmp = 1.0 / std::sqrt(absx);
  f_a_tmp = 1.0 / rt_powd_snf(absx, 1.5);
  b_et1_tmp_tmp = p_a_tmp * -0.005;
  a_tmp_tmp = j_a_tmp * j_a_tmp + c_a_tmp_tmp;
  J[1] =
      (b_unnamed_idx_1_tmp * -100.0 / h_a_tmp +
       (c_unnamed_idx_1_tmp * 100.0 + b_unnamed_idx_1_tmp * 0.0) * j_a_tmp /
           c_a_tmp_tmp) /
          a_tmp_tmp * -c_a_tmp_tmp -
      1.0 / std::sqrt(p_a_tmp * p_a_tmp * -0.0001 / absx + 1.0) *
          (et1_tmp_tmp *
               ((a_tmp * g_a_tmp * 2.0 + c_unnamed_idx_1_tmp * e_a_tmp * 2.0) +
                c_et1_tmp_tmp * b_a_tmp * unnamed_idx_1_tmp * 2.0) *
               0.01 +
           f_a_tmp *
               (c_unnamed_idx_1_tmp * c_a_tmp / 50.0 +
                std::sin(pitch) *
                    (std::cos(pitch) * 47.2 +
                     std::sin(pitch) *
                         (std::cos(roll) * 16.5 + std::sin(roll) * 16.0)) *
                    (std::cos(roll) * 16.0 - std::sin(roll) * 16.5) * 2.0) *
               b_et1_tmp_tmp);
  invJ[1] = 0.0;
  J[2] = 1.0 / std::sqrt(q_a_tmp * q_a_tmp * -0.0001 / g_a_tmp_tmp + 1.0) *
             (b_J_tmp * (unnamed_idx_2_tmp + l_a_tmp * d_a_tmp * 2.0) / 100.0 +
              c_J_tmp *
                  (unnamed_idx_2_tmp +
                   (i_a_tmp + b_unnamed_idx_2_tmp) * m_a_tmp / 5000.0) *
                  d_J_tmp) +
         1.0 / e_J_tmp *
             ((h_a_tmp_tmp - b_a_tmp_tmp * -100.0) / n_a_tmp +
              o_a_tmp * ((i_a_tmp - b_a_tmp_tmp * 0.0) + b_unnamed_idx_2_tmp) *
                  (1.0 / J_tmp)) *
             J_tmp;
  invJ[2] = 0.0;
  J[3] = (et1_tmp_tmp * (unnamed_idx_3_tmp + b_a_tmp * e_a_tmp * 2.0) / 100.0 +
          f_a_tmp *
              (unnamed_idx_3_tmp +
               (i_a_tmp + b_unnamed_idx_3_tmp) * c_a_tmp / 5000.0) *
              b_et1_tmp_tmp) *
             (1.0 / std::sqrt(p_a_tmp * p_a_tmp * -0.0001 / absx + 1.0)) +
         ((h_a_tmp_tmp - e_a_tmp_tmp * -100.0) / h_a_tmp +
          j_a_tmp * ((i_a_tmp - e_a_tmp_tmp * 0.0) + b_unnamed_idx_3_tmp) /
              c_a_tmp_tmp) /
             a_tmp_tmp * c_a_tmp_tmp;
  invJ[3] = 0.0;
  p = true;
  if (std::isinf(J[0]) || std::isnan(J[0]) ||
      (std::isinf(J[1]) || std::isnan(J[1]))) {
    p = false;
  }
  if ((!p) || (std::isinf(J[2]) || std::isnan(J[2]))) {
    p = false;
  }
  if ((!p) || (std::isinf(J[3]) || std::isnan(J[3]))) {
    p = false;
  }
  if (!p) {
    invJ[0] = rtNaN;
    invJ[1] = rtNaN;
    invJ[2] = rtNaN;
    invJ[3] = rtNaN;
  } else {
    double U[4];
    double V[4];
    double s[2];
    int r;
    coder::internal::svd(J, U, s, V);
    absx = std::abs(s[0]);
    if ((!std::isinf(absx)) && (!std::isnan(absx))) {
      if (absx <= 2.2250738585072014E-308) {
        absx = 4.94065645841247E-324;
      } else {
        frexp(absx, &exponent);
        absx = std::ldexp(1.0, exponent - 53);
      }
    } else {
      absx = rtNaN;
    }
    absx *= 2.0;
    r = -1;
    exponent = 0;
    while ((exponent < 2) && (s[exponent] > absx)) {
      r++;
      exponent++;
    }
    if (r + 1 > 0) {
      int br;
      int i;
      int vcol;
      vcol = 1;
      for (br = 0; br <= r; br++) {
        absx = 1.0 / s[br];
        i = vcol + 1;
        for (exponent = vcol; exponent <= i; exponent++) {
          V[exponent - 1] *= absx;
        }
        vcol += 2;
      }
      for (exponent = 0; exponent <= 2; exponent += 2) {
        i = exponent + 1;
        vcol = exponent + 2;
        if (i <= vcol) {
          std::memset(&invJ[i + -1], 0, ((vcol - i) + 1) * sizeof(double));
        }
      }
      br = 0;
      for (exponent = 0; exponent <= 2; exponent += 2) {
        int ar;
        ar = -1;
        br++;
        i = br + (r << 1);
        for (int ib{br}; ib <= i; ib += 2) {
          int i1;
          vcol = exponent + 1;
          i1 = exponent + 2;
          for (int ic{vcol}; ic <= i1; ic++) {
            invJ[ic - 1] += U[ib - 1] * V[(ar + ic) - exponent];
          }
          ar += 2;
        }
      }
    }
  }
}

//
// File trailer for foot_jacobian_inverse.cpp
//
// [EOF]
//