//
// File: svd.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 24-Aug-2023 18:26:44
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include "xdotc.h"
#include "xrotg.h"
#include "xswap.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double A[4]
//                double U[4]
//                double s[2]
//                double V[4]
// Return Type  : void
//
namespace coder {
namespace internal {
void svd(const double A[4], double U[4], double s[2], double V[4])
{
  double Vf[4];
  double b_A[4];
  double b_s[2];
  double e[2];
  double absxk;
  double c;
  double f;
  double nrm;
  double scale;
  double snorm;
  double sqds;
  double t;
  int iter;
  int kase;
  int m;
  int q;
  int qs;
  b_A[0] = A[0];
  b_A[1] = A[1];
  b_A[2] = A[2];
  b_A[3] = A[3];
  scale = 3.3121686421112381E-170;
  absxk = std::abs(A[0]);
  if (absxk > 3.3121686421112381E-170) {
    nrm = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    nrm = t * t;
  }
  absxk = std::abs(A[1]);
  if (absxk > scale) {
    t = scale / absxk;
    nrm = nrm * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    nrm += t * t;
  }
  nrm = scale * std::sqrt(nrm);
  if (nrm > 0.0) {
    if (A[0] < 0.0) {
      b_s[0] = -nrm;
    } else {
      b_s[0] = nrm;
    }
    if (std::abs(b_s[0]) >= 1.0020841800044864E-292) {
      absxk = 1.0 / b_s[0];
      b_A[0] = absxk * A[0];
      b_A[1] = absxk * A[1];
    } else {
      b_A[0] = A[0] / b_s[0];
      b_A[1] = A[1] / b_s[0];
    }
    b_A[0]++;
    b_s[0] = -b_s[0];
    absxk = -(blas::xdotc(b_A, b_A) / b_A[0]);
    if (!(absxk == 0.0)) {
      b_A[2] = A[2] + absxk * b_A[0];
      b_A[3] = A[3] + absxk * b_A[1];
    }
  } else {
    b_s[0] = 0.0;
  }
  m = 2;
  b_s[1] = b_A[3];
  e[0] = b_A[2];
  e[1] = 0.0;
  U[0] = b_A[0];
  U[2] = 0.0;
  U[1] = b_A[1];
  U[3] = 1.0;
  if (b_s[0] != 0.0) {
    absxk = -(blas::xdotc(U, U) / b_A[0]);
    if (!(absxk == 0.0)) {
      U[2] = absxk * b_A[0];
      U[3] = absxk * b_A[1] + 1.0;
    }
    U[1] = -b_A[1];
    U[0] = -b_A[0] + 1.0;
  } else {
    U[1] = 0.0;
    U[0] = 1.0;
  }
  Vf[2] = 0.0;
  Vf[3] = 1.0;
  Vf[1] = 0.0;
  Vf[0] = 1.0;
  for (q = 0; q < 2; q++) {
    nrm = b_s[q];
    if (nrm != 0.0) {
      t = std::abs(nrm);
      absxk = nrm / t;
      b_s[q] = t;
      if (q + 1 < 2) {
        e[0] /= absxk;
      }
      kase = q << 1;
      qs = kase + 2;
      for (int k{kase + 1}; k <= qs; k++) {
        U[k - 1] *= absxk;
      }
    }
    if ((q + 1 < 2) && (e[0] != 0.0)) {
      t = std::abs(e[0]);
      absxk = t / e[0];
      e[0] = t;
      b_s[1] *= absxk;
      Vf[2] *= absxk;
      Vf[3] *= absxk;
    }
  }
  iter = 0;
  snorm = std::fmax(std::fmax(0.0, std::fmax(std::abs(b_s[0]), std::abs(e[0]))),
                    std::fmax(std::abs(b_s[1]), 0.0));
  while ((m > 0) && (iter < 75)) {
    int ii_tmp_tmp;
    bool exitg1;
    ii_tmp_tmp = m - 1;
    q = m - 1;
    exitg1 = false;
    while (!(exitg1 || (q == 0))) {
      absxk = std::abs(e[0]);
      if ((absxk <=
           2.2204460492503131E-16 * (std::abs(b_s[0]) + std::abs(b_s[1]))) ||
          (absxk <= 1.0020841800044864E-292) ||
          ((iter > 20) && (absxk <= 2.2204460492503131E-16 * snorm))) {
        e[0] = 0.0;
        exitg1 = true;
      } else {
        q = 0;
      }
    }
    if (q == m - 1) {
      kase = 4;
    } else {
      qs = m;
      kase = m;
      exitg1 = false;
      while ((!exitg1) && (kase >= q)) {
        qs = kase;
        if (kase == q) {
          exitg1 = true;
        } else {
          absxk = 0.0;
          if (kase < m) {
            absxk = std::abs(e[0]);
          }
          if (kase > q + 1) {
            absxk += std::abs(e[0]);
          }
          t = std::abs(b_s[kase - 1]);
          if ((t <= 2.2204460492503131E-16 * absxk) ||
              (t <= 1.0020841800044864E-292)) {
            b_s[kase - 1] = 0.0;
            exitg1 = true;
          } else {
            kase--;
          }
        }
      }
      if (qs == q) {
        kase = 3;
      } else if (qs == m) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }
    switch (kase) {
    case 1: {
      f = e[0];
      e[0] = 0.0;
      for (int k{ii_tmp_tmp}; k >= q + 1; k--) {
        double temp;
        blas::xrotg(&b_s[0], &f, &c, &absxk);
        kase = (m - 1) << 1;
        temp = c * Vf[0] + absxk * Vf[kase];
        Vf[kase] = c * Vf[kase] - absxk * Vf[0];
        Vf[0] = temp;
        t = Vf[kase + 1];
        temp = c * Vf[1] + absxk * t;
        Vf[kase + 1] = c * t - absxk * Vf[1];
        Vf[1] = temp;
      }
    } break;
    case 2: {
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (int k{q + 1}; k <= m; k++) {
        double temp;
        blas::xrotg(&b_s[k - 1], &f, &sqds, &c);
        absxk = e[k - 1];
        f = -c * absxk;
        e[k - 1] = absxk * sqds;
        qs = (k - 1) << 1;
        kase = (q - 1) << 1;
        temp = sqds * U[qs] + c * U[kase];
        U[kase] = sqds * U[kase] - c * U[qs];
        U[qs] = temp;
        t = U[kase + 1];
        absxk = U[qs + 1];
        U[kase + 1] = sqds * t - c * absxk;
        U[qs + 1] = sqds * absxk + c * t;
      }
    } break;
    case 3: {
      double temp;
      absxk = b_s[m - 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(absxk), std::abs(b_s[0])),
                              std::abs(e[0])),
                    std::abs(b_s[q])),
          std::abs(e[q]));
      temp = absxk / scale;
      absxk = b_s[0] / scale;
      t = e[0] / scale;
      sqds = b_s[q] / scale;
      nrm = ((absxk + temp) * (absxk - temp) + t * t) / 2.0;
      c = temp * t;
      c *= c;
      if ((nrm != 0.0) || (c != 0.0)) {
        absxk = std::sqrt(nrm * nrm + c);
        if (nrm < 0.0) {
          absxk = -absxk;
        }
        absxk = c / (nrm + absxk);
      } else {
        absxk = 0.0;
      }
      f = (sqds + temp) * (sqds - temp) + absxk;
      absxk = sqds * (e[q] / scale);
      for (int k{q + 1}; k < 2; k++) {
        blas::xrotg(&f, &absxk, &sqds, &c);
        f = sqds * b_s[0] + c * e[0];
        nrm = sqds * e[0] - c * b_s[0];
        e[0] = nrm;
        absxk = c * b_s[1];
        b_s[1] *= sqds;
        temp = sqds * Vf[0] + c * Vf[2];
        Vf[2] = sqds * Vf[2] - c * Vf[0];
        Vf[0] = temp;
        temp = sqds * Vf[1] + c * Vf[3];
        Vf[3] = sqds * Vf[3] - c * Vf[1];
        Vf[1] = temp;
        b_s[0] = f;
        blas::xrotg(&b_s[0], &absxk, &sqds, &c);
        f = sqds * nrm + c * b_s[1];
        b_s[1] = -c * nrm + sqds * b_s[1];
        absxk = c * e[1];
        e[1] *= sqds;
        temp = sqds * U[0] + c * U[2];
        U[2] = sqds * U[2] - c * U[0];
        U[0] = temp;
        temp = sqds * U[1] + c * U[3];
        U[3] = sqds * U[3] - c * U[1];
        U[1] = temp;
      }
      e[0] = f;
      iter++;
    } break;
    default:
      if (b_s[q] < 0.0) {
        b_s[q] = -b_s[q];
        kase = q << 1;
        qs = kase + 2;
        for (int k{kase + 1}; k <= qs; k++) {
          Vf[k - 1] = -Vf[k - 1];
        }
      }
      while ((q + 1 < 2) && (b_s[0] < b_s[1])) {
        t = b_s[0];
        b_s[0] = b_s[1];
        b_s[1] = t;
        blas::xswap(Vf);
        blas::xswap(U);
        q = 1;
      }
      iter = 0;
      m--;
      break;
    }
  }
  s[0] = b_s[0];
  V[0] = Vf[0];
  V[1] = Vf[1];
  s[1] = b_s[1];
  V[2] = Vf[2];
  V[3] = Vf[3];
}

} // namespace internal
} // namespace coder

//
// File trailer for svd.cpp
//
// [EOF]
//
