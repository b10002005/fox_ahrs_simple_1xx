/*
 * File: calibrazione_acc_mag0.c
 *
 * Code generated for Simulink model 'calibrazione_acc_mag0'.
 *
 * Model version                  : 1.7
 * Simulink Coder version         : 8.0 (R2011a) 09-Mar-2011
 * TLC version                    : 8.0 (Feb  3 2011)
 * C/C++ source code generated on : Sun Nov 06 21:47:51 2011
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "calibrazione_acc_mag0.h"
#include "calibrazione_acc_mag0_private.h"

/* Block signals (auto storage) */
BlockIO_calibrazione_acc_mag0 calibrazione_acc_mag0_B;

/* Block states (auto storage) */
D_Work_calibrazione_acc_mag0 calibrazione_acc_mag0_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_calibrazione_acc calibrazione_acc_mag0_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_calibrazione_ac calibrazione_acc_mag0_Y;

/* Real-time model */
RT_MODEL_calibrazione_acc_mag0 calibrazione_acc_mag0_M_;
RT_MODEL_calibrazione_acc_mag0 *const calibrazione_acc_mag0_M =
  &calibrazione_acc_mag0_M_;

/* Forward declaration for local functions */
static void calibrazione_acc_mag0_invNxN(const real_T x[16], real_T y[16]);
static void calibrazione_acc_mag0_eml_xscal(int32_T n, real_T a, real_T x[100],
  int32_T ix0);
static void calibrazione_acc_mag0_eml_xswap(real_T x[100], int32_T ix0, int32_T
  iy0);
static void calibrazione_acc_mag0_eml_xrotg(real_T *a, real_T *b, real_T *c,
  real_T *s);
static void calibrazione_acc_mag0_eml_xrot(real_T x[100], int32_T ix0, int32_T
  iy0, real_T c, real_T s);
static real_T calibrazione_acc_mag0_max(const real_T varargin_1[5]);
static real_T calibrazione_acc_mag0_eml_xdotc(int32_T n, const real_T x[100],
  int32_T ix0, const real_T y[100], int32_T iy0);
static void calibrazione_acc_mag0_eml_xaxpy(int32_T n, real_T a, int32_T ix0,
  real_T y[100], int32_T iy0);
static real_T calibrazione_acc_mag0_eml_xnrm2(int32_T n, const real_T x[100],
  int32_T ix0);
static real_T calibrazione_acc_ma_eml_xnrm2_d(int32_T n, const real_T x[10],
  int32_T ix0);
static void calibrazione_acc_ma_eml_xscal_d(int32_T n, real_T a, real_T x[10],
  int32_T ix0);
static void calibrazione_acc_m_eml_xaxpy_dj(int32_T n, real_T a, const real_T x
  [10], int32_T ix0, real_T y[100], int32_T iy0);
static void calibrazione_acc_ma_eml_xaxpy_d(int32_T n, real_T a, const real_T x
  [100], int32_T ix0, real_T y[10], int32_T iy0);
static void calibrazione_acc_mag_eml_xgesvd(const real_T A[100], real_T U[100],
  real_T S[10], real_T V[100]);
static void calibrazione_acc_mag0_svd(const real_T A[100], real_T U[100], real_T
  S[100], real_T V[100]);
static void calibrazione_acc_mag0_inv(const real_T x[9], real_T y[9]);
static real_T calibrazione_acc_m_eml_xnrm2_dj(int32_T n, const real_T x[9],
  int32_T ix0);
static void calibrazione_acc_m_eml_xscal_dj(int32_T n, real_T a, real_T x[9],
  int32_T ix0);
static real_T calibrazione_acc__eml_xnrm2_dja(int32_T n, const real_T x[3],
  int32_T ix0);
static void calibrazione_acc__eml_xscal_dja(int32_T n, real_T a, real_T x[3],
  int32_T ix0);
static void calibrazione_acc_ma_eml_xswap_d(real_T x[9], int32_T ix0, int32_T
  iy0);
static void calibrazione_acc_mag_eml_xrot_d(real_T x[9], int32_T ix0, int32_T
  iy0, real_T c, real_T s);
static real_T calibrazione_acc_ma_eml_xdotc_d(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0);
static void calibrazione_acc__eml_xaxpy_dja(int32_T n, real_T a, int32_T ix0,
  real_T y[9], int32_T iy0);
static void calibrazione_ac_eml_xaxpy_djafj(int32_T n, real_T a, const real_T x
  [3], int32_T ix0, real_T y[9], int32_T iy0);
static void calibrazione_acc_eml_xaxpy_djaf(int32_T n, real_T a, const real_T x
  [9], int32_T ix0, real_T y[3], int32_T iy0);
static void calibrazione_acc_m_eml_xgesvd_d(const real_T A[9], real_T U[9],
  real_T S[3], real_T V[9]);
static void calibrazione_acc_mag0_svd_d(const real_T A[9], real_T U[9], real_T
  S[9], real_T V[9]);
static void calibrazione_acc_mag0_sqrt(real_T x[9]);
void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi, uint32_T
                  *ptrOutBitsLo)
{
  uint32_T absIn;
  uint32_T absIn_0;
  uint32_T in0Lo;
  uint32_T in0Hi;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn = (uint32_T)(in0 < 0 ? -in0 : in0);
  absIn_0 = (uint32_T)(in1 < 0 ? -in1 : in1);
  in0Hi = absIn >> 16;
  in0Lo = absIn & 65535U;
  in1Hi = absIn_0 >> 16;
  absIn = absIn_0 & 65535U;
  productHiLo = in0Hi * absIn;
  productLoHi = in0Lo * in1Hi;
  absIn *= in0Lo;
  absIn_0 = 0;
  in0Lo = (productLoHi << 16) + absIn;
  if (in0Lo < absIn) {
    absIn_0 = (uint32_T)1;
  }

  absIn = in0Lo;
  in0Lo += productHiLo << 16;
  if (in0Lo < absIn) {
    absIn_0++;
  }

  absIn = ((in0Hi * in1Hi + absIn_0) + (productLoHi >> 16)) + (productHiLo >> 16);
  if (!((in0 == 0) || (in1 == 0) || ((in0 > 0) == (in1 > 0)))) {
    absIn = ~absIn;
    in0Lo = ~in0Lo;
    in0Lo++;
    if (in0Lo == 0) {
      absIn++;
    }
  }

  *ptrOutBitsHi = absIn;
  *ptrOutBitsLo = in0Lo;
}

int32_T mul_s32_s32_s32_sat(int32_T a, int32_T b)
{
  int32_T result;
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  if (((int32_T)u32_chi > 0) || ((u32_chi == 0) && (u32_clo >= 2147483648U))) {
    result = MAX_int32_T;
  } else if (((int32_T)u32_chi < -1) || (((int32_T)u32_chi == -1) && (u32_clo <
               2147483648U))) {
    result = MIN_int32_T;
  } else {
    result = (int32_T)u32_clo;
  }

  return result;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = (rtNaN);
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function Block: '<S1>/acc_calibration' */
static void calibrazione_acc_mag0_invNxN(const real_T x[16], real_T y[16])
{
  int8_T p[4];
  real_T A[16];
  int8_T ipiv[4];
  int32_T mmj;
  int32_T jj;
  int32_T jp1j;
  int32_T b_j;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T b_ix;
  int32_T jy;
  int32_T ijA;
  int32_T pipk;
  int32_T c_i;
  for (b_j = 0; b_j < 16; b_j++) {
    y[b_j] = 0.0;
    A[b_j] = x[b_j];
  }

  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (b_j = 0; b_j < 3; b_j++) {
    mmj = -b_j;
    jj = b_j * 5;
    jp1j = jj + 1;
    jy = 0;
    ix = jj;
    smax = fabs(A[jj]);
    for (pipk = 2; pipk <= mmj + 4; pipk++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        jy = pipk - 1;
        smax = s;
      }
    }

    if (A[jj + jy] != 0.0) {
      if (jy != 0) {
        ipiv[b_j] = (int8_T)((b_j + jy) + 1);
        pipk = b_j + jy;
        smax = A[b_j];
        A[b_j] = A[pipk];
        A[pipk] = smax;
        b_ix = b_j + 4;
        pipk += 4;
        smax = A[b_ix];
        A[b_ix] = A[pipk];
        A[pipk] = smax;
        b_ix += 4;
        pipk += 4;
        smax = A[b_ix];
        A[b_ix] = A[pipk];
        A[pipk] = smax;
        b_ix += 4;
        pipk += 4;
        smax = A[b_ix];
        A[b_ix] = A[pipk];
        A[pipk] = smax;
      }

      ix = (jp1j + mmj) + 3;
      for (pipk = jp1j; pipk + 1 <= ix; pipk++) {
        A[pipk] /= A[jj];
      }
    }

    pipk = 3 - b_j;
    jy = jj + 4;
    for (c_i = 1; c_i <= pipk; c_i++) {
      if (A[jy] != 0.0) {
        smax = A[jy] * -1.0;
        ix = jp1j;
        b_ix = (mmj + jj) + 8;
        for (ijA = 5 + jj; ijA + 1 <= b_ix; ijA++) {
          A[ijA] += A[ix] * smax;
          ix++;
        }
      }

      jy += 4;
      jj += 4;
    }
  }

  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;
  if (ipiv[0] > 1) {
    pipk = p[ipiv[0] - 1];
    p[ipiv[0] - 1] = 1;
    p[0] = (int8_T)pipk;
  }

  if (ipiv[1] > 2) {
    pipk = p[ipiv[1] - 1];
    p[ipiv[1] - 1] = p[1];
    p[1] = (int8_T)pipk;
  }

  if (ipiv[2] > 3) {
    pipk = p[ipiv[2] - 1];
    p[ipiv[2] - 1] = p[2];
    p[2] = (int8_T)pipk;
  }

  y[(p[0] - 1) << 2] = 1.0;
  for (pipk = 0; pipk + 1 < 5; pipk++) {
    if (y[((p[0] - 1) << 2) + pipk] != 0.0) {
      for (jy = pipk + 1; jy + 1 < 5; jy++) {
        y[jy + ((p[0] - 1) << 2)] = y[((p[0] - 1) << 2) + jy] - y[((p[0] - 1) <<
          2) + pipk] * A[(pipk << 2) + jy];
      }
    }
  }

  y[1 + ((p[1] - 1) << 2)] = 1.0;
  for (pipk = 1; pipk + 1 < 5; pipk++) {
    if (y[((p[1] - 1) << 2) + pipk] != 0.0) {
      for (jy = pipk + 1; jy + 1 < 5; jy++) {
        y[jy + ((p[1] - 1) << 2)] = y[((p[1] - 1) << 2) + jy] - y[((p[1] - 1) <<
          2) + pipk] * A[(pipk << 2) + jy];
      }
    }
  }

  y[2 + ((p[2] - 1) << 2)] = 1.0;
  for (pipk = 2; pipk + 1 < 5; pipk++) {
    if (y[((p[2] - 1) << 2) + pipk] != 0.0) {
      for (jy = pipk + 1; jy + 1 < 5; jy++) {
        y[jy + ((p[2] - 1) << 2)] = y[((p[2] - 1) << 2) + jy] - y[((p[2] - 1) <<
          2) + pipk] * A[(pipk << 2) + jy];
      }
    }
  }

  y[3 + ((p[3] - 1) << 2)] = 1.0;
  for (pipk = 3; pipk + 1 < 5; pipk++) {
    if (y[((p[3] - 1) << 2) + pipk] != 0.0) {
      for (jy = pipk + 1; jy + 1 < 5; jy++) {
        y[jy + ((p[3] - 1) << 2)] = y[((p[3] - 1) << 2) + jy] - y[((p[3] - 1) <<
          2) + pipk] * A[(pipk << 2) + jy];
      }
    }
  }

  for (ix = 0; ix < 4; ix++) {
    pipk = ix << 2;
    if (y[3 + pipk] != 0.0) {
      y[3 + pipk] /= A[15];
      for (c_i = 0; c_i + 1 < 4; c_i++) {
        y[c_i + pipk] -= y[3 + pipk] * A[c_i + 12];
      }
    }

    if (y[2 + pipk] != 0.0) {
      y[2 + pipk] /= A[10];
      for (c_i = 0; c_i + 1 < 3; c_i++) {
        y[c_i + pipk] -= y[2 + pipk] * A[c_i + 8];
      }
    }

    if (y[1 + pipk] != 0.0) {
      y[1 + pipk] /= A[5];
      for (c_i = 0; c_i + 1 < 2; c_i++) {
        y[c_i + pipk] -= y[1 + pipk] * A[c_i + 4];
      }
    }

    if (y[pipk] != 0.0) {
      y[pipk] /= A[0];
      for (c_i = 0; c_i + 1 < 1; c_i++) {
        y[c_i + pipk] -= y[pipk] * A[c_i];
      }
    }
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    y = atan2(u0 > 0.0 ? 1.0 : -1.0, u1 > 0.0 ? 1.0 : -1.0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_eml_xscal(int32_T n, real_T a, real_T x[100],
  int32_T ix0)
{
  int32_T k;
  int32_T loop_ub;
  loop_ub = (ix0 + n) - 1;
  for (k = ix0; k <= loop_ub; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_eml_xswap(real_T x[100], int32_T ix0, int32_T
  iy0)
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  int32_T k;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 10; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_eml_xrotg(real_T *a, real_T *b, real_T *c,
  real_T *s)
{
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T r;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    bds = 0.0;
    ads = 1.0;
    r = 0.0;
    scale = 0.0;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    r = sqrt(ads * ads + bds * bds) * scale;
    if (roe < 0.0) {
      r = -r;
    }

    ads = *a / r;
    bds = *b / r;
    if (absa > absb) {
      scale = bds;
    } else if (ads != 0.0) {
      scale = 1.0 / ads;
    } else {
      scale = 1.0;
    }
  }

  *a = r;
  *b = scale;
  *c = ads;
  *s = bds;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_eml_xrot(real_T x[100], int32_T ix0, int32_T
  iy0, real_T c, real_T s)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  real_T y;
  real_T b_y;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 10; k++) {
    y = c * x[ix];
    b_y = s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = y + b_y;
    iy++;
    ix++;
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static real_T calibrazione_acc_mag0_max(const real_T varargin_1[5])
{
  real_T mtmp;
  int32_T itmp;
  int32_T ix;
  boolean_T searchingForNonNaN;
  int32_T k;
  boolean_T guard;
  boolean_T exitg;
  mtmp = varargin_1[0];
  itmp = 1;
  ix = 0;
  guard = FALSE;
  if (rtIsNaN(varargin_1[0])) {
    searchingForNonNaN = TRUE;
    k = 2;
    exitg = FALSE;
    while (((uint32_T)exitg == 0U) && (k < 6)) {
      ix++;
      if (!rtIsNaN(varargin_1[ix])) {
        mtmp = varargin_1[ix];
        itmp = k;
        searchingForNonNaN = FALSE;
        exitg = TRUE;
      } else {
        k++;
      }
    }

    if (!searchingForNonNaN) {
      guard = TRUE;
    }
  } else {
    guard = TRUE;
  }

  if (guard) {
    while (itmp + 1 < 6) {
      if (varargin_1[itmp] > mtmp) {
        mtmp = varargin_1[itmp];
      }

      itmp++;
    }
  }

  return mtmp;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static real_T calibrazione_acc_mag0_eml_xdotc(int32_T n, const real_T x[100],
  int32_T ix0, const real_T y[100], int32_T iy0)
{
  real_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0;
  if (!(n < 1)) {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_eml_xaxpy(int32_T n, real_T a, int32_T ix0,
  real_T y[100], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k <= n - 1; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static real_T calibrazione_acc_mag0_eml_xnrm2(int32_T n, const real_T x[100],
  int32_T ix0)
{
  real_T y;
  real_T scale;
  boolean_T firstNonZero;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 0.0;
      firstNonZero = TRUE;
      kend = (ix0 + n) - 1;
      for (k = ix0 - 1; k + 1 <= kend; k++) {
        if (x[k] != 0.0) {
          absxk = fabs(x[k]);
          if (firstNonZero) {
            scale = absxk;
            y = 1.0;
            firstNonZero = FALSE;
          } else if (scale < absxk) {
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            y += t * t;
          }
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static real_T calibrazione_acc_ma_eml_xnrm2_d(int32_T n, const real_T x[10],
  int32_T ix0)
{
  real_T y;
  real_T scale;
  boolean_T firstNonZero;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 0.0;
      firstNonZero = TRUE;
      kend = (ix0 + n) - 1;
      for (k = ix0 - 1; k + 1 <= kend; k++) {
        if (x[k] != 0.0) {
          absxk = fabs(x[k]);
          if (firstNonZero) {
            scale = absxk;
            y = 1.0;
            firstNonZero = FALSE;
          } else if (scale < absxk) {
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            y += t * t;
          }
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_ma_eml_xscal_d(int32_T n, real_T a, real_T x[10],
  int32_T ix0)
{
  int32_T k;
  int32_T loop_ub;
  loop_ub = (ix0 + n) - 1;
  for (k = ix0; k <= loop_ub; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_m_eml_xaxpy_dj(int32_T n, real_T a, const real_T x
  [10], int32_T ix0, real_T y[100], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k <= n - 1; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_ma_eml_xaxpy_d(int32_T n, real_T a, const real_T x
  [100], int32_T ix0, real_T y[10], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k <= n - 1; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag_eml_xgesvd(const real_T A[100], real_T U[100],
  real_T S[10], real_T V[100])
{
  real_T b_A[100];
  real_T s[10];
  real_T e[10];
  real_T work[10];
  int32_T q;
  int32_T qp;
  int32_T iter;
  real_T snorm;
  real_T ztest;
  int32_T kase;
  int32_T qs;
  real_T ztest_0;
  real_T f;
  int32_T km1;
  real_T scale;
  real_T sm;
  real_T sqds;
  real_T b;
  boolean_T exitg;
  int32_T i;
  real_T tmp[5];
  memcpy((void *)&b_A[0], (void *)&A[0], 100U * sizeof(real_T));
  for (i = 0; i < 10; i++) {
    s[i] = 0.0;
    e[i] = 0.0;
    work[i] = 0.0;
  }

  for (i = 0; i < 100; i++) {
    U[i] = 0.0;
    V[i] = 0.0;
  }

  for (i = 0; i < 9; i++) {
    qp = i + 2;
    iter = (10 * i + i) + 1;
    km1 = 9 - i;
    qs = km1 + 1;
    ztest = calibrazione_acc_mag0_eml_xnrm2(qs, b_A, iter);
    if (ztest == 0.0) {
      s[i] = 0.0;
    } else {
      if (b_A[iter - 1] < 0.0) {
        ztest = -ztest;
      }

      s[i] = ztest;
      calibrazione_acc_mag0_eml_xscal(qs, 1.0 / s[i], b_A, iter);
      b_A[iter - 1]++;
      s[i] = -s[i];
    }

    for (kase = qp; kase < 11; kase++) {
      q = ((kase - 1) * 10 + i) + 1;
      if (s[i] != 0.0) {
        calibrazione_acc_mag0_eml_xaxpy(qs, -(calibrazione_acc_mag0_eml_xdotc(qs,
          b_A, iter, b_A, q) / b_A[i + 10 * i]), iter, b_A, q);
      }

      e[kase - 1] = b_A[q - 1];
    }

    for (kase = i; kase + 1 < 11; kase++) {
      U[kase + 10 * i] = b_A[10 * i + kase];
    }

    if (i + 1 <= 8) {
      iter = 9 - i;
      ztest = calibrazione_acc_ma_eml_xnrm2_d(iter, e, qp);
      if (ztest == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[qp - 1] < 0.0) {
          ztest = -ztest;
        }

        e[i] = ztest;
        calibrazione_acc_ma_eml_xscal_d(iter, 1.0 / e[i], e, qp);
        e[qp - 1]++;
      }

      e[i] = -e[i];
      if (e[i] != 0.0) {
        for (kase = qp; kase < 11; kase++) {
          work[kase - 1] = 0.0;
        }

        for (kase = qp; kase < 11; kase++) {
          calibrazione_acc_ma_eml_xaxpy_d(km1, e[kase - 1], b_A, qp + 10 * (kase
            - 1), work, qp);
        }

        for (kase = qp; kase < 11; kase++) {
          calibrazione_acc_m_eml_xaxpy_dj(km1, -e[kase - 1] / e[qp - 1], work,
            qp, b_A, qp + 10 * (kase - 1));
        }
      }

      for (kase = qp; kase < 11; kase++) {
        V[(kase + 10 * i) - 1] = e[kase - 1];
      }
    }
  }

  i = 10;
  s[9] = b_A[99];
  e[8] = b_A[98];
  e[9] = 0.0;
  memset((void *)&U[90], 0, 10U * sizeof(real_T));
  U[99] = 1.0;
  for (km1 = 8; km1 >= 0; km1 += -1) {
    qs = 10 - km1;
    iter = 10 * km1 + km1;
    if (s[km1] != 0.0) {
      for (kase = km1 + 1; kase + 1 < 11; kase++) {
        q = (10 * kase + km1) + 1;
        calibrazione_acc_mag0_eml_xaxpy(qs, -(calibrazione_acc_mag0_eml_xdotc(qs,
          U, iter + 1, U, q) / U[iter]), iter + 1, U, q);
      }

      for (kase = km1; kase + 1 < 11; kase++) {
        U[kase + 10 * km1] = -U[10 * km1 + kase];
      }

      U[iter]++;
      for (kase = 1; kase <= km1; kase++) {
        U[(kase + 10 * km1) - 1] = 0.0;
      }
    } else {
      memset((void *)&U[10 * km1], 0, 10U * sizeof(real_T));
      U[iter] = 1.0;
    }
  }

  for (q = 9; q >= 0; q += -1) {
    if ((q + 1 <= 8) && (e[q] != 0.0)) {
      qp = q + 2;
      iter = 9 - q;
      kase = 10 * q + qp;
      for (km1 = qp; km1 < 11; km1++) {
        qs = (km1 - 1) * 10 + qp;
        calibrazione_acc_mag0_eml_xaxpy(iter, -(calibrazione_acc_mag0_eml_xdotc
          (iter, V, kase, V, qs) / V[kase - 1]), kase, V, qs);
      }
    }

    memset((void *)&V[10 * q], 0, 10U * sizeof(real_T));
    V[q + 10 * q] = 1.0;
  }

  for (kase = 0; kase < 10; kase++) {
    f = e[kase];
    if (s[kase] != 0.0) {
      ztest_0 = fabs(s[kase]);
      ztest = s[kase] / ztest_0;
      s[kase] = ztest_0;
      if (kase + 1 < 10) {
        f = e[kase] / ztest;
      }

      calibrazione_acc_mag0_eml_xscal(10, ztest, U, 1 + 10 * kase);
    }

    if ((kase + 1 < 10) && (f != 0.0)) {
      ztest_0 = fabs(f);
      ztest = ztest_0 / f;
      f = ztest_0;
      s[kase + 1] *= ztest;
      calibrazione_acc_mag0_eml_xscal(10, ztest, V, 1 + 10 * (kase + 1));
    }

    e[kase] = f;
  }

  iter = 0;
  snorm = 0.0;
  for (kase = 0; kase < 10; kase++) {
    f = fabs(s[kase]);
    b = fabs(e[kase]);
    f = (f >= b) || rtIsNaN(b) ? f : b;
    snorm = (snorm >= f) || rtIsNaN(f) ? snorm : f;
  }

  while ((i > 0) && (!(iter >= 75))) {
    km1 = i - 1;
    do {
      kase = 0;
      q = km1;
      if (km1 == 0) {
        kase = 1;
      } else {
        ztest = fabs(e[km1 - 1]);
        if ((ztest <= (fabs(s[km1 - 1]) + fabs(s[km1])) * 2.2204460492503131E-16)
            || (ztest <= 1.0020841800044864E-292) || ((iter > 20) && (ztest <=
              2.2204460492503131E-16 * snorm))) {
          e[km1 - 1] = 0.0;
          kase = 1;
        } else {
          km1--;
        }
      }
    } while ((uint32_T)kase == 0U);

    if (i - 1 == km1) {
      kase = 4;
    } else {
      qs = i;
      kase = i;
      exitg = FALSE;
      while (((uint32_T)exitg == 0U) && (kase >= km1)) {
        qs = kase;
        if (kase == km1) {
          exitg = TRUE;
        } else {
          ztest = 0.0;
          if (kase < i) {
            ztest = fabs(e[kase - 1]);
          }

          if (kase > km1 + 1) {
            ztest += fabs(e[kase - 2]);
          }

          ztest_0 = fabs(s[kase - 1]);
          if ((ztest_0 <= 2.2204460492503131E-16 * ztest) || (ztest_0 <=
               1.0020841800044864E-292)) {
            s[kase - 1] = 0.0;
            exitg = TRUE;
          } else {
            kase--;
          }
        }
      }

      if (qs == km1) {
        kase = 3;
      } else if (qs == i) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[i - 2];
      e[i - 2] = 0.0;
      for (kase = i - 2; kase + 1 >= q + 1; kase--) {
        ztest = s[kase];
        calibrazione_acc_mag0_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[kase] = ztest;
        if (kase + 1 > q + 1) {
          km1 = kase - 1;
          f = -b * e[km1];
          e[km1] *= ztest_0;
        }

        calibrazione_acc_mag0_eml_xrot(V, 1 + 10 * kase, 1 + 10 * (i - 1),
          ztest_0, b);
      }
      break;

     case 2:
      kase = q - 1;
      f = e[kase];
      e[kase] = 0.0;
      while (q + 1 <= i) {
        ztest = s[q];
        calibrazione_acc_mag0_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[q] = ztest;
        f = -b * e[q];
        e[q] *= ztest_0;
        calibrazione_acc_mag0_eml_xrot(U, 1 + 10 * q, 1 + 10 * kase, ztest_0, b);
        q++;
      }
      break;

     case 3:
      kase = i - 2;
      tmp[0] = fabs(s[i - 1]);
      tmp[1] = fabs(s[kase]);
      tmp[2] = fabs(e[kase]);
      tmp[3] = fabs(s[q]);
      tmp[4] = fabs(e[q]);
      scale = calibrazione_acc_mag0_max(tmp);
      sm = s[i - 1] / scale;
      ztest = s[kase] / scale;
      ztest_0 = e[kase] / scale;
      sqds = s[q] / scale;
      b = ((ztest + sm) * (ztest - sm) + ztest_0 * ztest_0) / 2.0;
      ztest = sm * ztest_0;
      ztest *= ztest;
      ztest_0 = 0.0;
      if ((b != 0.0) || (ztest != 0.0)) {
        ztest_0 = sqrt(b * b + ztest);
        if (b < 0.0) {
          ztest_0 = -ztest_0;
        }

        ztest_0 = ztest / (b + ztest_0);
      }

      f = (sqds + sm) * (sqds - sm) + ztest_0;
      ztest = e[q] / scale * sqds;
      for (km1 = q; km1 + 1 <= kase + 1; km1++) {
        qs = km1 + 1;
        calibrazione_acc_mag0_eml_xrotg(&f, &ztest, &sm, &scale);
        if (km1 + 1 > q + 1) {
          e[km1 - 1] = f;
        }

        ztest = sm * s[km1];
        ztest_0 = scale * e[km1];
        e[km1] = sm * e[km1] - scale * s[km1];
        b = s[qs];
        s[qs] *= sm;
        calibrazione_acc_mag0_eml_xrot(V, 1 + 10 * km1, 1 + 10 * (km1 + 1), sm,
          scale);
        ztest_0 += ztest;
        ztest = scale * b;
        calibrazione_acc_mag0_eml_xrotg(&ztest_0, &ztest, &b, &sm);
        s[km1] = ztest_0;
        f = b * e[km1] + sm * s[qs];
        s[qs] = -sm * e[km1] + b * s[qs];
        ztest = sm * e[qs];
        e[qs] *= b;
        calibrazione_acc_mag0_eml_xrot(U, 1 + 10 * km1, 1 + 10 * (km1 + 1), b,
          sm);
      }

      e[i - 2] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
        calibrazione_acc_mag0_eml_xscal(10, -1.0, V, 1 + 10 * q);
      }

      qp = q + 1;
      while ((q + 1 < 10) && (s[q] < s[qp])) {
        ztest_0 = s[q];
        s[q] = s[qp];
        s[qp] = ztest_0;
        calibrazione_acc_mag0_eml_xswap(V, 1 + 10 * q, 1 + 10 * (q + 1));
        calibrazione_acc_mag0_eml_xswap(U, 1 + 10 * q, 1 + 10 * (q + 1));
        q = qp;
        qp++;
      }

      iter = 0;
      i--;
      break;
    }
  }

  memcpy((void *)&S[0], (void *)&s[0], 10U * sizeof(real_T));
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_svd(const real_T A[100], real_T U[100], real_T
  S[100], real_T V[100])
{
  real_T s[10];
  int32_T k;
  calibrazione_acc_mag_eml_xgesvd(A, U, s, V);
  memset((void *)&S[0], 0, 100U * sizeof(real_T));
  for (k = 0; k < 10; k++) {
    S[k + 10 * k] = s[k];
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_inv(const real_T x[9], real_T y[9])
{
  real_T b_x[9];
  int32_T p;
  int32_T p_0;
  int32_T p_1;
  real_T absx;
  real_T absx_0;
  real_T absx_1;
  int32_T itmp;
  memcpy((void *)&b_x[0], (void *)&x[0], 9U * sizeof(real_T));
  p = 0;
  p_0 = 3;
  p_1 = 6;
  absx = fabs(x[0]);
  absx_0 = fabs(x[1]);
  absx_1 = fabs(x[2]);
  if ((absx_0 > absx) && (absx_0 > absx_1)) {
    p = 3;
    p_0 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    absx = b_x[3];
    b_x[3] = b_x[4];
    b_x[4] = absx;
    absx = b_x[6];
    b_x[6] = b_x[7];
    b_x[7] = absx;
  } else {
    if (absx_1 > absx) {
      p = 6;
      p_1 = 0;
      b_x[0] = x[2];
      b_x[2] = x[0];
      absx = b_x[3];
      b_x[3] = b_x[5];
      b_x[5] = absx;
      absx = b_x[6];
      b_x[6] = b_x[8];
      b_x[8] = absx;
    }
  }

  b_x[1] /= b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= b_x[1] * b_x[3];
  b_x[5] -= b_x[2] * b_x[3];
  b_x[7] -= b_x[1] * b_x[6];
  b_x[8] -= b_x[2] * b_x[6];
  if (fabs(b_x[5]) > fabs(b_x[4])) {
    itmp = p_0;
    p_0 = p_1;
    p_1 = itmp;
    absx = b_x[1];
    b_x[1] = b_x[2];
    b_x[2] = absx;
    absx = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx;
    absx = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx;
  }

  b_x[5] /= b_x[4];
  b_x[8] -= b_x[5] * b_x[7];
  absx = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx_0 = -(b_x[7] * absx + b_x[1]) / b_x[4];
  y[p] = ((1.0 - b_x[3] * absx_0) - b_x[6] * absx) / b_x[0];
  y[p + 1] = absx_0;
  y[p + 2] = absx;
  absx = -b_x[5] / b_x[8];
  absx_0 = (1.0 - b_x[7] * absx) / b_x[4];
  y[p_0] = -(b_x[3] * absx_0 + b_x[6] * absx) / b_x[0];
  y[p_0 + 1] = absx_0;
  y[p_0 + 2] = absx;
  absx = 1.0 / b_x[8];
  absx_0 = -b_x[7] * absx / b_x[4];
  y[p_1] = -(b_x[3] * absx_0 + b_x[6] * absx) / b_x[0];
  y[p_1 + 1] = absx_0;
  y[p_1 + 2] = absx;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static real_T calibrazione_acc_m_eml_xnrm2_dj(int32_T n, const real_T x[9],
  int32_T ix0)
{
  real_T y;
  real_T scale;
  boolean_T firstNonZero;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 0.0;
      firstNonZero = TRUE;
      kend = (ix0 + n) - 1;
      for (k = ix0 - 1; k + 1 <= kend; k++) {
        if (x[k] != 0.0) {
          absxk = fabs(x[k]);
          if (firstNonZero) {
            scale = absxk;
            y = 1.0;
            firstNonZero = FALSE;
          } else if (scale < absxk) {
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            y += t * t;
          }
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_m_eml_xscal_dj(int32_T n, real_T a, real_T x[9],
  int32_T ix0)
{
  int32_T k;
  int32_T loop_ub;
  loop_ub = (ix0 + n) - 1;
  for (k = ix0; k <= loop_ub; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static real_T calibrazione_acc__eml_xnrm2_dja(int32_T n, const real_T x[3],
  int32_T ix0)
{
  real_T y;
  real_T scale;
  boolean_T firstNonZero;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 0.0;
      firstNonZero = TRUE;
      kend = (ix0 + n) - 1;
      for (k = ix0 - 1; k + 1 <= kend; k++) {
        if (x[k] != 0.0) {
          absxk = fabs(x[k]);
          if (firstNonZero) {
            scale = absxk;
            y = 1.0;
            firstNonZero = FALSE;
          } else if (scale < absxk) {
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            y += t * t;
          }
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc__eml_xscal_dja(int32_T n, real_T a, real_T x[3],
  int32_T ix0)
{
  int32_T k;
  int32_T loop_ub;
  loop_ub = (ix0 + n) - 1;
  for (k = ix0; k <= loop_ub; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_ma_eml_xswap_d(real_T x[9], int32_T ix0, int32_T
  iy0)
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag_eml_xrot_d(real_T x[9], int32_T ix0, int32_T
  iy0, real_T c, real_T s)
{
  int32_T ix;
  int32_T iy;
  real_T y;
  real_T b_y;
  ix = ix0 - 1;
  iy = iy0 - 1;
  y = c * x[ix];
  b_y = s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = y + b_y;
  iy++;
  ix++;
  y = c * x[ix];
  b_y = s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = y + b_y;
  iy++;
  ix++;
  y = c * x[ix];
  b_y = s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = y + b_y;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static real_T calibrazione_acc_ma_eml_xdotc_d(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0)
{
  real_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0;
  if (!(n < 1)) {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc__eml_xaxpy_dja(int32_T n, real_T a, int32_T ix0,
  real_T y[9], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k <= n - 1; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_ac_eml_xaxpy_djafj(int32_T n, real_T a, const real_T x
  [3], int32_T ix0, real_T y[9], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k <= n - 1; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_eml_xaxpy_djaf(int32_T n, real_T a, const real_T x
  [9], int32_T ix0, real_T y[3], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k <= n - 1; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_m_eml_xgesvd_d(const real_T A[9], real_T U[9],
  real_T S[3], real_T V[9])
{
  real_T b_A[9];
  real_T s[3];
  real_T e[3];
  real_T work[3];
  int32_T q;
  int32_T qp;
  int32_T m;
  int32_T iter;
  real_T snorm;
  real_T ztest;
  int32_T kase;
  int32_T qs;
  real_T ztest_0;
  real_T f;
  int32_T mm;
  real_T scale;
  real_T sm;
  real_T sqds;
  real_T b;
  boolean_T exitg;
  real_T tmp[5];
  s[0] = 0.0;
  s[1] = 0.0;
  s[2] = 0.0;
  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  work[0] = 0.0;
  work[1] = 0.0;
  work[2] = 0.0;
  for (qp = 0; qp < 9; qp++) {
    b_A[qp] = A[qp];
    U[qp] = 0.0;
    V[qp] = 0.0;
  }

  ztest = calibrazione_acc_m_eml_xnrm2_dj(3, b_A, 1);
  if (ztest == 0.0) {
    s[0] = 0.0;
  } else {
    if (b_A[0] < 0.0) {
      ztest = -ztest;
    }

    s[0] = ztest;
    calibrazione_acc_m_eml_xscal_dj(3, 1.0 / s[0], b_A, 1);
    b_A[0]++;
    s[0] = -s[0];
  }

  for (qp = 2; qp < 4; qp++) {
    qs = (qp - 1) * 3 + 1;
    if (s[0] != 0.0) {
      calibrazione_acc__eml_xaxpy_dja(3, -(calibrazione_acc_ma_eml_xdotc_d(3,
        b_A, 1, b_A, qs) / b_A[0]), 1, b_A, qs);
    }

    e[qp - 1] = b_A[qs - 1];
  }

  for (kase = 0; kase + 1 < 4; kase++) {
    U[kase] = b_A[kase];
  }

  ztest = calibrazione_acc__eml_xnrm2_dja(2, e, 2);
  if (ztest == 0.0) {
    e[0] = 0.0;
  } else {
    if (e[1] < 0.0) {
      ztest = -ztest;
    }

    e[0] = ztest;
    calibrazione_acc__eml_xscal_dja(2, 1.0 / e[0], e, 2);
    e[1]++;
  }

  e[0] = -e[0];
  if (e[0] != 0.0) {
    for (kase = 2; kase < 4; kase++) {
      work[kase - 1] = 0.0;
    }

    for (kase = 2; kase < 4; kase++) {
      calibrazione_acc_eml_xaxpy_djaf(2, e[kase - 1], b_A, 2 + 3 * (kase - 1),
        work, 2);
    }

    for (kase = 2; kase < 4; kase++) {
      calibrazione_ac_eml_xaxpy_djafj(2, -e[kase - 1] / e[1], work, 2, b_A, 2 +
        3 * (kase - 1));
    }
  }

  for (kase = 2; kase < 4; kase++) {
    V[kase - 1] = e[kase - 1];
  }

  ztest = calibrazione_acc_m_eml_xnrm2_dj(2, b_A, 5);
  if (ztest == 0.0) {
    s[1] = 0.0;
  } else {
    if (b_A[4] < 0.0) {
      ztest = -ztest;
    }

    s[1] = ztest;
    calibrazione_acc_m_eml_xscal_dj(2, 1.0 / s[1], b_A, 5);
    b_A[4]++;
    s[1] = -s[1];
  }

  qp = 3;
  while (qp < 4) {
    if (s[1] != 0.0) {
      calibrazione_acc__eml_xaxpy_dja(2, -(calibrazione_acc_ma_eml_xdotc_d(2,
        b_A, 5, b_A, 8) / b_A[4]), 5, b_A, 8);
    }

    e[2] = b_A[7];
    qp = 4;
  }

  for (kase = 1; kase + 1 < 4; kase++) {
    U[kase + 3] = b_A[kase + 3];
  }

  m = 3;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 0.0;
  U[8] = 1.0;
  for (mm = 1; mm >= 0; mm += -1) {
    kase = 3 - mm;
    q = 3 * mm + mm;
    if (s[mm] != 0.0) {
      for (qp = mm + 1; qp + 1 < 4; qp++) {
        qs = (3 * qp + mm) + 1;
        calibrazione_acc__eml_xaxpy_dja(kase, -(calibrazione_acc_ma_eml_xdotc_d
          (kase, U, q + 1, U, qs) / U[q]), q + 1, U, qs);
      }

      for (kase = mm; kase + 1 < 4; kase++) {
        U[kase + 3 * mm] = -U[3 * mm + kase];
      }

      U[q]++;
      kase = 1;
      while (kase <= mm) {
        U[3] = 0.0;
        kase = 2;
      }
    } else {
      U[3 * mm] = 0.0;
      U[1 + 3 * mm] = 0.0;
      U[2 + 3 * mm] = 0.0;
      U[q] = 1.0;
    }
  }

  for (qp = 2; qp >= 0; qp += -1) {
    if ((qp + 1 <= 1) && (e[0] != 0.0)) {
      for (kase = 2; kase < 4; kase++) {
        mm = (kase - 1) * 3 + 2;
        calibrazione_acc__eml_xaxpy_dja(2, -(calibrazione_acc_ma_eml_xdotc_d(2,
          V, 2, V, mm) / V[1]), 2, V, mm);
      }
    }

    V[3 * qp] = 0.0;
    V[1 + 3 * qp] = 0.0;
    V[2 + 3 * qp] = 0.0;
    V[qp + 3 * qp] = 1.0;
  }

  f = e[0];
  if (s[0] != 0.0) {
    ztest_0 = fabs(s[0]);
    ztest = s[0] / ztest_0;
    s[0] = ztest_0;
    f = e[0] / ztest;
    calibrazione_acc_m_eml_xscal_dj(3, ztest, U, 1);
  }

  if (f != 0.0) {
    ztest_0 = fabs(f);
    ztest = ztest_0 / f;
    f = ztest_0;
    s[1] *= ztest;
    calibrazione_acc_m_eml_xscal_dj(3, ztest, V, 4);
  }

  e[0] = f;
  f = e[1];
  if (s[1] != 0.0) {
    ztest_0 = fabs(s[1]);
    ztest = s[1] / ztest_0;
    s[1] = ztest_0;
    f = e[1] / ztest;
    calibrazione_acc_m_eml_xscal_dj(3, ztest, U, 4);
  }

  if (f != 0.0) {
    ztest_0 = fabs(f);
    ztest = ztest_0 / f;
    f = ztest_0;
    s[2] *= ztest;
    calibrazione_acc_m_eml_xscal_dj(3, ztest, V, 7);
  }

  e[1] = f;
  if (s[2] != 0.0) {
    ztest_0 = fabs(s[2]);
    ztest = s[2] / ztest_0;
    s[2] = ztest_0;
    calibrazione_acc_m_eml_xscal_dj(3, ztest, U, 7);
  }

  e[2] = 0.0;
  iter = 0;
  f = (s[0] >= e[0]) || rtIsNaN(e[0]) ? s[0] : e[0];
  snorm = (0.0 >= f) || rtIsNaN(f) ? 0.0 : f;
  f = (s[1] >= e[1]) || rtIsNaN(e[1]) ? s[1] : e[1];
  snorm = (snorm >= f) || rtIsNaN(f) ? snorm : f;
  snorm = (snorm >= s[2]) || rtIsNaN(s[2]) ? snorm : s[2];
  while ((m > 0) && (!(iter >= 75))) {
    qp = m - 1;
    do {
      kase = 0;
      q = qp;
      if (qp == 0) {
        kase = 1;
      } else {
        ztest = fabs(e[qp - 1]);
        if ((ztest <= (fabs(s[qp - 1]) + fabs(s[qp])) * 2.2204460492503131E-16) ||
            (ztest <= 1.0020841800044864E-292) || ((iter > 20) && (ztest <=
              2.2204460492503131E-16 * snorm))) {
          e[qp - 1] = 0.0;
          kase = 1;
        } else {
          qp--;
        }
      }
    } while ((uint32_T)kase == 0U);

    if (m - 1 == qp) {
      kase = 4;
    } else {
      qs = m;
      mm = m;
      exitg = FALSE;
      while (((uint32_T)exitg == 0U) && (mm >= qp)) {
        qs = mm;
        if (mm == qp) {
          exitg = TRUE;
        } else {
          ztest = 0.0;
          if (mm < m) {
            ztest = fabs(e[mm - 1]);
          }

          if (mm > qp + 1) {
            ztest += fabs(e[mm - 2]);
          }

          ztest_0 = fabs(s[mm - 1]);
          if ((ztest_0 <= 2.2204460492503131E-16 * ztest) || (ztest_0 <=
               1.0020841800044864E-292)) {
            s[mm - 1] = 0.0;
            exitg = TRUE;
          } else {
            mm--;
          }
        }
      }

      if (qs == qp) {
        kase = 3;
      } else if (qs == m) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m - 2];
      e[m - 2] = 0.0;
      for (kase = m - 2; kase + 1 >= q + 1; kase--) {
        ztest = s[kase];
        calibrazione_acc_mag0_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[kase] = ztest;
        if (kase + 1 > q + 1) {
          f = -b * e[0];
          e[0] *= ztest_0;
        }

        calibrazione_acc_mag_eml_xrot_d(V, 1 + 3 * kase, 1 + 3 * (m - 1),
          ztest_0, b);
      }
      break;

     case 2:
      kase = q - 1;
      f = e[kase];
      e[kase] = 0.0;
      while (q + 1 <= m) {
        ztest = s[q];
        calibrazione_acc_mag0_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[q] = ztest;
        f = -b * e[q];
        e[q] *= ztest_0;
        calibrazione_acc_mag_eml_xrot_d(U, 1 + 3 * q, 1 + 3 * kase, ztest_0, b);
        q++;
      }
      break;

     case 3:
      mm = m - 2;
      tmp[0] = fabs(s[m - 1]);
      tmp[1] = fabs(s[mm]);
      tmp[2] = fabs(e[mm]);
      tmp[3] = fabs(s[q]);
      tmp[4] = fabs(e[q]);
      scale = calibrazione_acc_mag0_max(tmp);
      sm = s[m - 1] / scale;
      ztest = s[mm] / scale;
      ztest_0 = e[mm] / scale;
      sqds = s[q] / scale;
      b = ((ztest + sm) * (ztest - sm) + ztest_0 * ztest_0) / 2.0;
      ztest = sm * ztest_0;
      ztest *= ztest;
      ztest_0 = 0.0;
      if ((b != 0.0) || (ztest != 0.0)) {
        ztest_0 = sqrt(b * b + ztest);
        if (b < 0.0) {
          ztest_0 = -ztest_0;
        }

        ztest_0 = ztest / (b + ztest_0);
      }

      f = (sqds + sm) * (sqds - sm) + ztest_0;
      ztest = e[q] / scale * sqds;
      for (qp = q; qp + 1 <= mm + 1; qp++) {
        kase = qp + 1;
        calibrazione_acc_mag0_eml_xrotg(&f, &ztest, &sm, &scale);
        if (qp + 1 > q + 1) {
          e[0] = f;
        }

        ztest = sm * s[qp];
        ztest_0 = scale * e[qp];
        e[qp] = sm * e[qp] - scale * s[qp];
        b = s[kase];
        s[kase] *= sm;
        calibrazione_acc_mag_eml_xrot_d(V, 1 + 3 * qp, 1 + 3 * (qp + 1), sm,
          scale);
        ztest_0 += ztest;
        ztest = scale * b;
        calibrazione_acc_mag0_eml_xrotg(&ztest_0, &ztest, &b, &sm);
        s[qp] = ztest_0;
        f = b * e[qp] + sm * s[kase];
        s[kase] = -sm * e[qp] + b * s[kase];
        ztest = sm * e[kase];
        e[kase] *= b;
        calibrazione_acc_mag_eml_xrot_d(U, 1 + 3 * qp, 1 + 3 * (qp + 1), b, sm);
      }

      e[m - 2] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
        calibrazione_acc_m_eml_xscal_dj(3, -1.0, V, 1 + 3 * q);
      }

      qp = q + 1;
      while ((q + 1 < 3) && (s[q] < s[qp])) {
        ztest_0 = s[q];
        s[q] = s[qp];
        s[qp] = ztest_0;
        calibrazione_acc_ma_eml_xswap_d(V, 1 + 3 * q, 1 + 3 * (q + 1));
        calibrazione_acc_ma_eml_xswap_d(U, 1 + 3 * q, 1 + 3 * (q + 1));
        q = qp;
        qp++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  S[0] = s[0];
  S[1] = s[1];
  S[2] = s[2];
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_svd_d(const real_T A[9], real_T U[9], real_T
  S[9], real_T V[9])
{
  real_T s[3];
  calibrazione_acc_m_eml_xgesvd_d(A, U, s, V);
  memset((void *)&S[0], 0, 9U * sizeof(real_T));
  S[0] = s[0];
  S[4] = s[1];
  S[8] = s[2];
}

/* Function for MATLAB Function Block: '<S1>/mag_calibration' */
static void calibrazione_acc_mag0_sqrt(real_T x[9])
{
  int32_T k;
  for (k = 0; k < 9; k++) {
    x[k] = sqrt(x[k]);
  }
}

/* Model step function */
void calibrazione_acc_mag0_step(void)
{
  real_T W[24];
  real_T X[12];
  real_T H[400];
  real_T gsi[10];
  real_T K[100];
  real_T P[100];
  real_T L[100];
  real_T U[9];
  real_T A[9];
  real_T V[9];
  real_T rtb_UnitConversion;
  real_T rtb_UnitConversion_d;
  real_T rtb_gphih;
  real_T rtb_Sum;
  boolean_T rtb_RelationalOperator;
  real_T rtb_u80deg;
  real_T rtb_u0deg;
  real_T rtb_Gain;
  boolean_T rtb_LogicalOperator;
  int32_T s15_iter;
  real_T rtb_b[3];
  real_T rtb_Sum_i;
  int32_T s64_iter;
  real_T rtb_b2;
  real_T rtb_a2;
  real_T rtb_q;
  real_T rtb_Sum1_b;
  real_T rtb_Sum4;
  int32_T s23_iter;
  real_T rtb_TmpSignalConversionAtspm1In[13];
  int32_T rtb_Sum2_g;
  real_T rtb_G[9];
  real_T rtb_tc_old[169];
  real_T Assignment_d[169];
  real_T Assignment[169];
  int32_T i;
  real_T W_0[16];
  real_T tmp[16];
  real_T K_0[9];
  real_T H_0[100];
  real_T tmp_0[24];
  real_T tmp_1[18];
  real_T tmp_2[13];
  real_T rtb_sincos_o1_p_idx;
  real_T rtb_sincos_o1_p_idx_0;
  real_T rtb_sincos_o1_idx;
  real_T rtb_sincos_o2_idx;
  real_T rtb_Sum1_e_idx;
  real_T rtb_Sum1_e_idx_0;
  real_T rtb_Sum1_e_idx_1;
  real_T rtb_Sum1_e_idx_2;
  real_T UnitDelay2_DSTATE_idx;
  int32_T qY;
  int32_T qY_0;
  int32_T qY_1;
  int32_T qY_2;
  int32_T qY_3;
  int32_T qY_4;

  /* Trigonometry: '<S2>/sin(phi)' incorporates:
   *  Inport: '<Root>/position'
   */
  rtb_UnitConversion = sin(calibrazione_acc_mag0_U.position[0]);

  /* Math: '<S2>/sin(phi)^2' */
  rtb_UnitConversion *= rtb_UnitConversion;

  /* Sum: '<S2>/Sum1' incorporates:
   *  Constant: '<S2>/First Eccentricity'
   *  Constant: '<S2>/One'
   *  Math: '<S2>/eps^2'
   *  Product: '<S2>/Product'
   */
  rtb_UnitConversion_d = calibrazione_acc_mag0_P.One_Value -
    calibrazione_acc_mag0_P.FirstEccentricity_Value *
    calibrazione_acc_mag0_P.FirstEccentricity_Value * rtb_UnitConversion;

  /* Math: '<S2>/Math Function1' incorporates:
   *  Constant: '<S2>/1//2'
   */
  if ((rtb_UnitConversion_d < 0.0) && (calibrazione_acc_mag0_P.u_Value > floor
       (calibrazione_acc_mag0_P.u_Value))) {
    rtb_UnitConversion_d = -rt_powd_snf(-rtb_UnitConversion_d,
      calibrazione_acc_mag0_P.u_Value);
  } else {
    rtb_UnitConversion_d = rt_powd_snf(rtb_UnitConversion_d,
      calibrazione_acc_mag0_P.u_Value);
  }

  /* End of Math: '<S2>/Math Function1' */

  /* Product: '<S2>/Product4' incorporates:
   *  Constant: '<S2>/Gravity at Equator'
   *  Constant: '<S2>/Gravity formula const'
   *  Constant: '<S2>/One'
   *  Product: '<S2>/Product3'
   *  Sum: '<S2>/Sum2'
   */
  rtb_UnitConversion_d = (calibrazione_acc_mag0_P.Gravityformulaconst_Value *
    rtb_UnitConversion + calibrazione_acc_mag0_P.One_Value) *
    calibrazione_acc_mag0_P.GravityatEquator_Value / rtb_UnitConversion_d;

  /* Fcn: '<S2>/g(phi,h)' incorporates:
   *  Inport: '<Root>/position'
   */
  rtb_gphih = (rtb_UnitConversion_d - (3.0877e-006 - 4.4e-009 *
    rtb_UnitConversion) * calibrazione_acc_mag0_U.position[2]) + 7.2e-014 *
    calibrazione_acc_mag0_U.position[2] * calibrazione_acc_mag0_U.position[2];

  /* MATLAB Function Block: '<S1>/acc_calibration' incorporates:
   *  Inport: '<Root>/w1'
   *  Inport: '<Root>/w2'
   *  Inport: '<Root>/w3'
   *  Inport: '<Root>/w4'
   *  Inport: '<Root>/w5'
   *  Inport: '<Root>/w6'
   */

  /* MATLAB Function 'calibrazione_acc_mag/acc_calibration': '<S4>:1' */
  /* '<S4>:1:3' */
  /* '<S4>:1:10' */
  W[0] = calibrazione_acc_mag0_U.w1[0];
  W[6] = calibrazione_acc_mag0_U.w1[1];
  W[12] = calibrazione_acc_mag0_U.w1[2];
  W[18] = calibrazione_acc_mag0_U.w1[3];
  W[1] = calibrazione_acc_mag0_U.w2[0];
  W[7] = calibrazione_acc_mag0_U.w2[1];
  W[13] = calibrazione_acc_mag0_U.w2[2];
  W[19] = calibrazione_acc_mag0_U.w2[3];
  W[2] = calibrazione_acc_mag0_U.w3[0];
  W[8] = calibrazione_acc_mag0_U.w3[1];
  W[14] = calibrazione_acc_mag0_U.w3[2];
  W[20] = calibrazione_acc_mag0_U.w3[3];
  W[3] = calibrazione_acc_mag0_U.w4[0];
  W[9] = calibrazione_acc_mag0_U.w4[1];
  W[15] = calibrazione_acc_mag0_U.w4[2];
  W[21] = calibrazione_acc_mag0_U.w4[3];
  W[4] = calibrazione_acc_mag0_U.w5[0];
  W[10] = calibrazione_acc_mag0_U.w5[1];
  W[16] = calibrazione_acc_mag0_U.w5[2];
  W[22] = calibrazione_acc_mag0_U.w5[3];
  W[5] = calibrazione_acc_mag0_U.w6[0];
  W[11] = calibrazione_acc_mag0_U.w6[1];
  W[17] = calibrazione_acc_mag0_U.w6[2];
  W[23] = calibrazione_acc_mag0_U.w6[3];

  /* '<S4>:1:11' */
  for (qY_3 = 0; qY_3 < 4; qY_3++) {
    for (i = 0; i < 4; i++) {
      W_0[qY_3 + (i << 2)] = 0.0;
      for (rtb_Sum2_g = 0; rtb_Sum2_g < 6; rtb_Sum2_g++) {
        W_0[qY_3 + (i << 2)] = W[6 * qY_3 + rtb_Sum2_g] * W[6 * i + rtb_Sum2_g]
          + W_0[(i << 2) + qY_3];
      }
    }
  }

  calibrazione_acc_mag0_invNxN(W_0, tmp);
  for (qY_3 = 0; qY_3 < 4; qY_3++) {
    for (i = 0; i < 6; i++) {
      tmp_0[qY_3 + (i << 2)] = 0.0;
      tmp_0[qY_3 + (i << 2)] = tmp_0[(i << 2) + qY_3] + tmp[qY_3] * W[i];
      tmp_0[qY_3 + (i << 2)] = tmp_0[(i << 2) + qY_3] + tmp[qY_3 + 4] * W[i + 6];
      tmp_0[qY_3 + (i << 2)] = tmp_0[(i << 2) + qY_3] + tmp[qY_3 + 8] * W[i + 12];
      tmp_0[qY_3 + (i << 2)] = tmp_0[(i << 2) + qY_3] + tmp[qY_3 + 12] * W[i +
        18];
    }
  }

  tmp_1[0] = 0.0;
  tmp_1[6] = 0.0;
  tmp_1[12] = rtb_gphih;
  tmp_1[1] = 0.0;
  tmp_1[7] = 0.0;
  tmp_1[13] = -rtb_gphih;
  tmp_1[2] = 0.0;
  tmp_1[8] = rtb_gphih;
  tmp_1[14] = 0.0;
  tmp_1[3] = 0.0;
  tmp_1[9] = -rtb_gphih;
  tmp_1[15] = 0.0;
  tmp_1[4] = rtb_gphih;
  tmp_1[10] = 0.0;
  tmp_1[16] = 0.0;
  tmp_1[5] = -rtb_gphih;
  tmp_1[11] = 0.0;
  tmp_1[17] = 0.0;
  for (qY_3 = 0; qY_3 < 4; qY_3++) {
    for (i = 0; i < 3; i++) {
      X[qY_3 + (i << 2)] = 0.0;
      for (rtb_Sum2_g = 0; rtb_Sum2_g < 6; rtb_Sum2_g++) {
        X[qY_3 + (i << 2)] = tmp_0[(rtb_Sum2_g << 2) + qY_3] * tmp_1[6 * i +
          rtb_Sum2_g] + X[(i << 2) + qY_3];
      }
    }
  }

  /* End of MATLAB Function Block: '<S1>/acc_calibration' */

  /* Outport: '<Root>/G_acc' incorporates:
   *  MATLAB Function Block: '<S1>/acc_calibration'
   */
  /* '<S4>:1:12' */
  /* '<S4>:1:15' */
  /* X=(inv(W'*W))*W'*Y; */
  calibrazione_acc_mag0_Y.G_acc[0] = X[0];
  calibrazione_acc_mag0_Y.G_acc[3] = X[4];
  calibrazione_acc_mag0_Y.G_acc[6] = X[8];
  calibrazione_acc_mag0_Y.G_acc[1] = X[1];
  calibrazione_acc_mag0_Y.G_acc[4] = X[5];
  calibrazione_acc_mag0_Y.G_acc[7] = X[9];
  calibrazione_acc_mag0_Y.G_acc[2] = X[2];
  calibrazione_acc_mag0_Y.G_acc[5] = X[6];
  calibrazione_acc_mag0_Y.G_acc[8] = X[10];

  /* Outport: '<Root>/Bias_acc' incorporates:
   *  MATLAB Function Block: '<S1>/acc_calibration'
   */
  calibrazione_acc_mag0_Y.Bias_acc[0] = X[3];
  calibrazione_acc_mag0_Y.Bias_acc[1] = X[7];
  calibrazione_acc_mag0_Y.Bias_acc[2] = X[11];

  /* Sum: '<S3>/Sum' incorporates:
   *  Constant: '<S3>/Date'
   *  Gain: '<S3>/DayToYear'
   *  Gain: '<S3>/MonthToYear'
   */
  rtb_Sum = (calibrazione_acc_mag0_P.DayToYear_Gain *
             calibrazione_acc_mag0_P.Date_Value[0] +
             calibrazione_acc_mag0_P.MonthToYear_Gain *
             calibrazione_acc_mag0_P.Date_Value[1]) +
    calibrazione_acc_mag0_P.Date_Value[2];

  /* Sum: '<S13>/Sum' incorporates:
   *  Constant: '<S13>/epoch'
   */
  rtb_Sum_i = rtb_Sum - calibrazione_acc_mag0_P.epoch_Value;

  /* RelationalOperator: '<S21>/Relational Operator' incorporates:
   *  Memory: '<S21>/otime'
   */
  rtb_RelationalOperator = (rtb_Sum !=
    calibrazione_acc_mag0_DWork.otime_PreviousInput);

  /* Saturate: '<S6>/+//- 180 deg' incorporates:
   *  Inport: '<Root>/position'
   */
  rtb_u80deg = calibrazione_acc_mag0_U.position[1] >=
    calibrazione_acc_mag0_P.u80deg_UpperSat ?
    calibrazione_acc_mag0_P.u80deg_UpperSat : calibrazione_acc_mag0_U.position[1]
    <= calibrazione_acc_mag0_P.u80deg_LowerSat ?
    calibrazione_acc_mag0_P.u80deg_LowerSat : calibrazione_acc_mag0_U.position[1];

  /* Saturate: '<S6>/+//- 90 deg' incorporates:
   *  Inport: '<Root>/position'
   */
  rtb_u0deg = calibrazione_acc_mag0_U.position[0] >=
    calibrazione_acc_mag0_P.u0deg_UpperSat ?
    calibrazione_acc_mag0_P.u0deg_UpperSat : calibrazione_acc_mag0_U.position[0]
    <= calibrazione_acc_mag0_P.u0deg_LowerSat ?
    calibrazione_acc_mag0_P.u0deg_LowerSat : calibrazione_acc_mag0_U.position[0];

  /* Gain: '<S65>/Unit Conversion' */
  rtb_sincos_o1_p_idx = calibrazione_acc_mag0_P.UnitConversion_Gain * rtb_u80deg;
  rtb_sincos_o1_p_idx_0 = calibrazione_acc_mag0_P.UnitConversion_Gain *
    rtb_u0deg;

  /* Trigonometry: '<S18>/sincos' */
  rtb_sincos_o1_idx = sin(rtb_sincos_o1_p_idx_0);
  rtb_sincos_o2_idx = cos(rtb_sincos_o1_p_idx_0);

  /* Outputs for Enabled SubSystem: '<S13>/Convert from geodetic to  spherical coordinates ' incorporates:
   *  EnablePort: '<S17>/Enable'
   */
  /* RelationalOperator: '<S20>/Relational Operator' incorporates:
   *  Constant: '<S17>/cp[1]'
   *  Constant: '<S17>/sp[1]'
   *  Inport: '<S17>/cp[2]'
   *  Inport: '<S17>/sp[2]'
   *  Memory: '<S20>/olon'
   *  Trigonometry: '<S18>/sincos'
   */
  if (rtb_u80deg != calibrazione_acc_mag0_DWork.olon_PreviousInput) {
    calibrazione_acc_mag0_B.sp2 = sin(rtb_sincos_o1_p_idx);
    calibrazione_acc_mag0_B.cp2 = cos(rtb_sincos_o1_p_idx);

    /* Outputs for Iterator SubSystem: '<S17>/For Iterator Subsystem' incorporates:
     *  ForIterator: '<S64>/For Iterator'
     */
    for (s64_iter = 1; (real_T)s64_iter <=
         calibrazione_acc_mag0_P.ForIterator_IterationLimit; s64_iter++) {
      rtb_gphih = (real_T)s64_iter;

      /* UnitDelay: '<S64>/Unit Delay1' */
      rtb_sincos_o1_p_idx_0 = calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE[0];
      rtb_Sum1_b = calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE[1];

      /* Switch: '<S64>/cp[m-1] sp[m-1]' */
      if (!(rtb_gphih > calibrazione_acc_mag0_P.cpm1spm1_Threshold)) {
        rtb_sincos_o1_p_idx_0 = calibrazione_acc_mag0_B.cp2;
        rtb_Sum1_b = calibrazione_acc_mag0_B.sp2;
      }

      /* End of Switch: '<S64>/cp[m-1] sp[m-1]' */

      /* Sum: '<S64>/Sum2' incorporates:
       *  Product: '<S64>/Product1'
       *  Product: '<S64>/Product2'
       */
      rtb_UnitConversion = rtb_sincos_o1_p_idx_0 * calibrazione_acc_mag0_B.sp2 +
        rtb_Sum1_b * calibrazione_acc_mag0_B.cp2;
      for (i = 0; i < 11; i++) {
        /* Assignment: '<S64>/Assignment' incorporates:
         *  Constant: '<S64>/Constant'
         */
        if (s64_iter == 1) {
          calibrazione_acc_mag0_B.Assignment[i] =
            calibrazione_acc_mag0_P.Constant_Value_j[i];
        }

        /* End of Assignment: '<S64>/Assignment' */

        /* Assignment: '<S64>/Assignment1' incorporates:
         *  Constant: '<S64>/Constant1'
         */
        if (s64_iter == 1) {
          calibrazione_acc_mag0_B.Assignment1[i] =
            calibrazione_acc_mag0_P.Constant1_Value_fr[i];
        }

        /* End of Assignment: '<S64>/Assignment1' */
      }

      /* Assignment: '<S64>/Assignment' */
      calibrazione_acc_mag0_B.Assignment[(int32_T)rtb_gphih - 1] =
        rtb_UnitConversion;

      /* Sum: '<S64>/Sum1' incorporates:
       *  Product: '<S64>/Product3'
       *  Product: '<S64>/Product8'
       */
      rtb_b2 = rtb_sincos_o1_p_idx_0 * calibrazione_acc_mag0_B.cp2 - rtb_Sum1_b *
        calibrazione_acc_mag0_B.sp2;

      /* Assignment: '<S64>/Assignment1' */
      calibrazione_acc_mag0_B.Assignment1[(int32_T)rtb_gphih - 1] = rtb_b2;

      /* Update for UnitDelay: '<S64>/Unit Delay1' */
      calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE[0] = rtb_b2;
      calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE[1] = rtb_UnitConversion;
    }

    /* End of Outputs for SubSystem: '<S17>/For Iterator Subsystem' */
    calibrazione_acc_mag0_B.cp1 = calibrazione_acc_mag0_P.cp1_Value;
    calibrazione_acc_mag0_B.sp1 = calibrazione_acc_mag0_P.sp1_Value;
  }

  /* End of RelationalOperator: '<S20>/Relational Operator' */
  /* End of Outputs for SubSystem: '<S13>/Convert from geodetic to  spherical coordinates ' */

  /* Gain: '<S12>/Unit Conversion' incorporates:
   *  Inport: '<Root>/position'
   */
  rtb_UnitConversion_d = calibrazione_acc_mag0_P.UnitConversion_Gain_a *
    calibrazione_acc_mag0_U.position[2];

  /* Gain: '<S6>/Gain' incorporates:
   *  Saturate: '<S6>/0 to 1,000,000 m'
   */
  rtb_Gain = (rtb_UnitConversion_d >=
              calibrazione_acc_mag0_P.to1000000m_UpperSat ?
              calibrazione_acc_mag0_P.to1000000m_UpperSat : rtb_UnitConversion_d
              <= calibrazione_acc_mag0_P.to1000000m_LowerSat ?
              calibrazione_acc_mag0_P.to1000000m_LowerSat : rtb_UnitConversion_d)
    * calibrazione_acc_mag0_P.Gain_Gain_e;

  /* Logic: '<S19>/Logical Operator' incorporates:
   *  Memory: '<S19>/oalt'
   *  Memory: '<S19>/olat'
   *  RelationalOperator: '<S19>/Relational Operator'
   *  RelationalOperator: '<S19>/Relational Operator1'
   */
  rtb_LogicalOperator = ((rtb_u0deg !=
    calibrazione_acc_mag0_DWork.olat_PreviousInput) || (rtb_Gain !=
    calibrazione_acc_mag0_DWork.oalt_PreviousInput));

  /* Product: '<S18>/Product' */
  rtb_gphih = rtb_sincos_o1_idx * rtb_sincos_o1_idx;

  /* Product: '<S18>/Product1' */
  rtb_UnitConversion = rtb_sincos_o2_idx * rtb_sincos_o2_idx;

  /* Outputs for Enabled SubSystem: '<S13>/Convert from geodetic to  spherical coordinates' incorporates:
   *  EnablePort: '<S16>/Enable'
   */
  if (rtb_LogicalOperator) {
    /* Product: '<S16>/b2' incorporates:
     *  Constant: '<S16>/b'
     */
    rtb_b2 = calibrazione_acc_mag0_P.b_Value * calibrazione_acc_mag0_P.b_Value;

    /* Product: '<S16>/a2' incorporates:
     *  Constant: '<S16>/a'
     */
    rtb_a2 = calibrazione_acc_mag0_P.a_Value * calibrazione_acc_mag0_P.a_Value;

    /* Sqrt: '<S59>/sqrt' incorporates:
     *  Product: '<S59>/Product'
     *  Sum: '<S59>/Sum'
     *  Sum: '<S59>/Sum1'
     */
    rtb_q = sqrt(rtb_a2 - (rtb_a2 - rtb_b2) * rtb_gphih);

    /* Product: '<S16>/Product1' */
    rtb_Sum1_b = rtb_q * rtb_Gain;

    /* Sqrt: '<S58>/sqrt' incorporates:
     *  Product: '<S58>/Product10'
     *  Product: '<S58>/Product9'
     *  Sum: '<S58>/Sum7'
     */
    rtb_sincos_o1_p_idx_0 = sqrt(rtb_UnitConversion * rtb_a2 + rtb_b2 *
      rtb_gphih);

    /* Product: '<S61>/a4' */
    rtb_Sum4 = rtb_a2 * rtb_a2;

    /* Sqrt: '<S16>/sqrt' incorporates:
     *  Gain: '<S61>/Gain'
     *  Product: '<S61>/Product1'
     *  Product: '<S61>/Product6'
     *  Product: '<S61>/Product7'
     *  Product: '<S61>/Product8'
     *  Product: '<S61>/b4'
     *  Sum: '<S61>/Sum5'
     *  Sum: '<S61>/Sum6'
     *  Sum: '<S61>/Sum9'
     */
    calibrazione_acc_mag0_B.sqrt_a = sqrt((rtb_Sum4 - (rtb_Sum4 - rtb_b2 *
      rtb_b2) * rtb_gphih) / (rtb_q * rtb_q) +
      (calibrazione_acc_mag0_P.Gain_Gain_a * rtb_Sum1_b + rtb_Gain * rtb_Gain));

    /* Product: '<S56>/Product11' incorporates:
     *  Sum: '<S56>/Sum8'
     */
    calibrazione_acc_mag0_B.Product11 = (rtb_Gain + rtb_sincos_o1_p_idx_0) /
      calibrazione_acc_mag0_B.sqrt_a;

    /* Sum: '<S60>/Sum2' */
    rtb_Sum4 = rtb_a2 + rtb_Sum1_b;

    /* Sum: '<S60>/Sum1' */
    rtb_Sum1_b += rtb_b2;

    /* Product: '<S57>/Product4' incorporates:
     *  Product: '<S57>/Product3'
     *  Product: '<S60>/Product1'
     *  Product: '<S60>/Product2'
     *  Sqrt: '<S57>/sqrt'
     *  Sum: '<S57>/Sum3'
     */
    calibrazione_acc_mag0_B.Product4 = rtb_sincos_o1_idx / sqrt(rtb_Sum4 *
      rtb_Sum4 / (rtb_Sum1_b * rtb_Sum1_b) * rtb_UnitConversion + rtb_gphih);

    /* Product: '<S62>/Product12' incorporates:
     *  Product: '<S62>/Product1'
     *  Sum: '<S62>/Sum1'
     */
    calibrazione_acc_mag0_B.Product12 = (rtb_a2 - rtb_b2) /
      (calibrazione_acc_mag0_B.sqrt_a * rtb_sincos_o1_p_idx_0) *
      rtb_sincos_o2_idx * rtb_sincos_o1_idx;

    /* Sqrt: '<S63>/sqrt' incorporates:
     *  Constant: '<S63>/Constant'
     *  Product: '<S63>/Product5'
     *  Sum: '<S63>/Sum4'
     */
    calibrazione_acc_mag0_B.sqrt_n = sqrt
      (calibrazione_acc_mag0_P.Constant_Value_a -
       calibrazione_acc_mag0_B.Product4 * calibrazione_acc_mag0_B.Product4);
  }

  /* End of Outputs for SubSystem: '<S13>/Convert from geodetic to  spherical coordinates' */

  /* Product: '<S13>/aor' incorporates:
   *  Constant: '<S13>/re'
   */
  rtb_a2 = calibrazione_acc_mag0_P.re_Value / calibrazione_acc_mag0_B.sqrt_a;

  /* Outputs for Iterator SubSystem: '<S13>/Compute magnetic vector in spherical coordinates' incorporates:
   *  ForIterator: '<S15>/For Iterator'
   */
  /* InitializeConditions for UnitDelay: '<S15>/Unit Delay' */
  rtb_q = calibrazione_acc_mag0_P.UnitDelay_X0_b;

  /* InitializeConditions for UnitDelay: '<S15>/Unit Delay2' */
  rtb_Sum4 = calibrazione_acc_mag0_P.UnitDelay2_X0_a[0];
  rtb_sincos_o2_idx = calibrazione_acc_mag0_P.UnitDelay2_X0_a[1];
  rtb_sincos_o1_idx = calibrazione_acc_mag0_P.UnitDelay2_X0_a[2];
  UnitDelay2_DSTATE_idx = calibrazione_acc_mag0_P.UnitDelay2_X0_a[3];
  for (s15_iter = 1; s15_iter <=
       calibrazione_acc_mag0_P.ForIterator_IterationLimit_l; s15_iter++) {
    /* Switch: '<S15>/ar(n)' incorporates:
     *  Product: '<S13>/ar'
     */
    if (!(s15_iter > calibrazione_acc_mag0_P.arn_Threshold)) {
      rtb_q = rtb_a2 * rtb_a2;
    }

    /* End of Switch: '<S15>/ar(n)' */

    /* Product: '<S15>/Product8' */
    rtb_q *= rtb_a2;

    /* Sum: '<S15>/Sum' incorporates:
     *  Constant: '<S15>/Constant'
     */
    qY = s15_iter + calibrazione_acc_mag0_P.Constant_Value_jd;
    if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_jd < 0) &&
                           (qY >= 0))) {
      qY = MIN_int32_T;
    } else {
      if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant_Value_jd > 0) &&
                             (qY <= 0))) {
        qY = MAX_int32_T;
      }
    }

    /* End of Sum: '<S15>/Sum' */

    /* Outputs for Iterator SubSystem: '<S15>/For Iterator Subsystem' incorporates:
     *  ForIterator: '<S23>/For Iterator'
     */
    if (calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[4] != 0) {
      /* InitializeConditions for UnitDelay: '<S24>/Unit Delay1' */
      calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_h =
        calibrazione_acc_mag0_P.UnitDelay1_X0_h;

      /* InitializeConditions for UnitDelay: '<S24>/Unit Delay3' */
      calibrazione_acc_mag0_DWork.UnitDelay3_DSTATE =
        calibrazione_acc_mag0_P.UnitDelay3_X0;

      /* InitializeConditions for UnitDelay: '<S24>/Unit Delay2' */
      calibrazione_acc_mag0_DWork.UnitDelay2_DSTATE_d =
        calibrazione_acc_mag0_P.UnitDelay2_X0;

      /* InitializeConditions for UnitDelay: '<S24>/Unit Delay4' */
      calibrazione_acc_mag0_DWork.UnitDelay4_DSTATE =
        calibrazione_acc_mag0_P.UnitDelay4_X0;
    }

    for (i = 0; i < 7; i++) {
      calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[i] = 1U;
    }

    /* Sum: '<S15>/Sum' incorporates:
     *  Constant: '<S30>/Constant'
     *  Constant: '<S30>/Constant1'
     *  Logic: '<S30>/Logical Operator'
     *  RelationalOperator: '<S30>/Relational Operator'
     *  RelationalOperator: '<S30>/Relational Operator1'
     *  Sum: '<S23>/Sum1'
     */
    for (s23_iter = 1; s23_iter <= qY; s23_iter++) {
      /* Sum: '<S23>/Sum1' incorporates:
       *  Constant: '<S23>/Constant'
       */
      qY_4 = s23_iter - calibrazione_acc_mag0_P.Constant_Value_p;
      if ((s23_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_p >= 0) &&
                             (qY_4 >= 0))) {
        qY_4 = MIN_int32_T;
      } else {
        if ((s23_iter >= 0) && ((calibrazione_acc_mag0_P.Constant_Value_p < 0) &&
             (qY_4 < 0))) {
          qY_4 = MAX_int32_T;
        }
      }

      /* End of Sum: '<S23>/Sum1' */

      /* Outputs for Enabled SubSystem: '<S23>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' incorporates:
       *  EnablePort: '<S25>/Enable'
       */
      if (rtb_LogicalOperator) {
        /* Sum: '<S25>/Sum' incorporates:
         *  Constant: '<S25>/Constant'
         *  Sum: '<S23>/Sum1'
         */
        qY_3 = s15_iter + calibrazione_acc_mag0_P.Constant_Value_bd;
        if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_bd < 0) &&
             (qY_3 >= 0))) {
          qY_3 = MIN_int32_T;
        } else {
          if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant_Value_bd > 0)
               && (qY_3 <= 0))) {
            qY_3 = MAX_int32_T;
          }
        }

        qY_2 = qY_4 + calibrazione_acc_mag0_P.Constant_Value_bd;
        if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant_Value_bd < 0) &&
                           (qY_2 >= 0))) {
          qY_2 = MIN_int32_T;
        } else {
          if ((qY_4 > 0) && ((calibrazione_acc_mag0_P.Constant_Value_bd > 0) &&
                             (qY_2 <= 0))) {
            qY_2 = MAX_int32_T;
          }
        }

        /* End of Sum: '<S25>/Sum' */

        /* If: '<S25>/if n == m elseif (n==1&m==0) elseif (n>1&m~=n)' incorporates:
         *  Sum: '<S23>/Sum1'
         */
        if (s15_iter == qY_4) {
          /* Outputs for IfAction SubSystem: '<S25>/If Action Subsystem' incorporates:
           *  ActionPort: '<S39>/Action Port'
           */
          /* Sum: '<S43>/Sum2' incorporates:
           *  Constant: '<S43>/Constant'
           */
          qY_1 = qY_4 - calibrazione_acc_mag0_P.Constant_Value_bn4;
          if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant_Value_bn4 >= 0) &&
                             (qY_1 >= 0))) {
            qY_1 = MIN_int32_T;
          } else {
            if ((qY_4 >= 0) && ((calibrazione_acc_mag0_P.Constant_Value_bn4 < 0)
                                && (qY_1 < 0))) {
              qY_1 = MAX_int32_T;
            }
          }

          /* End of Sum: '<S43>/Sum2' */

          /* Gain: '<S43>/Gain' incorporates:
           *  Sum: '<S43>/Sum2'
           */
          s64_iter = mul_s32_s32_s32_sat(calibrazione_acc_mag0_P.Gain_Gain_b,
            qY_1);

          /* Selector: '<S39>/Selector' incorporates:
           *  Sum: '<S43>/Sum1'
           */
          qY_1 = s15_iter + s64_iter;
          if ((s15_iter < 0) && ((s64_iter < 0) && (qY_1 >= 0))) {
            qY_1 = MIN_int32_T;
          } else {
            if ((s15_iter > 0) && ((s64_iter > 0) && (qY_1 <= 0))) {
              qY_1 = MAX_int32_T;
            }
          }

          /* End of Selector: '<S39>/Selector' */

          /* Product: '<S39>/Product1' incorporates:
           *  Selector: '<S39>/Selector'
           *  Sum: '<S43>/Sum1'
           *  UnitDelay: '<S25>/Unit Delay1'
           */
          calibrazione_acc_mag0_B.Merge_h[0] =
            calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[qY_1 - 1] *
            calibrazione_acc_mag0_B.sqrt_n;

          /* Selector: '<S39>/Selector' incorporates:
           *  Sum: '<S43>/Sum1'
           */
          qY_1 = s15_iter + s64_iter;
          if ((s15_iter < 0) && ((s64_iter < 0) && (qY_1 >= 0))) {
            qY_1 = MIN_int32_T;
          } else {
            if ((s15_iter > 0) && ((s64_iter > 0) && (qY_1 <= 0))) {
              qY_1 = MAX_int32_T;
            }
          }

          /* End of Selector: '<S39>/Selector' */

          /* Sum: '<S39>/Sum' incorporates:
           *  Product: '<S39>/Product'
           *  Product: '<S39>/Product2'
           *  Selector: '<S39>/Selector'
           *  Selector: '<S39>/Selector1'
           *  Sum: '<S43>/Sum1'
           *  UnitDelay: '<S25>/Unit Delay1'
           */
          calibrazione_acc_mag0_B.Merge_h[1] =
            calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_k[((s15_iter - 1) * 13
            + qY_4) - 1] * calibrazione_acc_mag0_B.sqrt_n +
            calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[qY_1 - 1] *
            calibrazione_acc_mag0_B.Product4;

          /* End of Outputs for SubSystem: '<S25>/If Action Subsystem' */
        } else if ((s15_iter == 1) && (qY_4 == 0)) {
          /* Outputs for IfAction SubSystem: '<S25>/If Action Subsystem1' incorporates:
           *  ActionPort: '<S40>/Action Port'
           */
          /* Selector: '<S40>/Selector' incorporates:
           *  Gain: '<S45>/Gain'
           *  Sum: '<S45>/Sum1'
           */
          i = mul_s32_s32_s32_sat(calibrazione_acc_mag0_P.Gain_Gain_bn, 0);
          qY_1 = 1 + i;
          if ((i > 0) && (qY_1 <= 0)) {
            qY_1 = MAX_int32_T;
          }

          /* End of Selector: '<S40>/Selector' */

          /* Product: '<S40>/Product3' incorporates:
           *  Selector: '<S40>/Selector'
           *  Sum: '<S45>/Sum1'
           *  UnitDelay: '<S25>/Unit Delay1'
           */
          calibrazione_acc_mag0_B.Merge_h[0] =
            calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[qY_1 - 1] *
            calibrazione_acc_mag0_B.Product4;

          /* Selector: '<S40>/Selector1' incorporates:
           *  Constant: '<S46>/Constant'
           *  Sum: '<S46>/Sum'
           */
          qY_1 = calibrazione_acc_mag0_P.Constant_Value_bn44;

          /* Selector: '<S40>/Selector' incorporates:
           *  Gain: '<S45>/Gain'
           *  Sum: '<S45>/Sum1'
           */
          i = mul_s32_s32_s32_sat(calibrazione_acc_mag0_P.Gain_Gain_bn, 0);
          qY_0 = 1 + i;
          if ((i > 0) && (qY_0 <= 0)) {
            qY_0 = MAX_int32_T;
          }

          /* End of Selector: '<S40>/Selector' */

          /* Sum: '<S40>/Sum' incorporates:
           *  Product: '<S40>/Product'
           *  Product: '<S40>/Product2'
           *  Selector: '<S40>/Selector'
           *  Selector: '<S40>/Selector1'
           *  Sum: '<S45>/Sum1'
           *  Sum: '<S46>/Sum'
           *  UnitDelay: '<S25>/Unit Delay1'
           */
          calibrazione_acc_mag0_B.Merge_h[1] =
            calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_k[qY_1 - 1] *
            calibrazione_acc_mag0_B.Product4 -
            calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[qY_0 - 1] *
            calibrazione_acc_mag0_B.sqrt_n;

          /* End of Outputs for SubSystem: '<S25>/If Action Subsystem1' */
        } else {
          if ((s15_iter > 1) && (s15_iter != qY_4)) {
            /* Outputs for IfAction SubSystem: '<S25>/If Action Subsystem2' incorporates:
             *  ActionPort: '<S41>/Action Port'
             */
            /* Sum: '<S49>/Sum' incorporates:
             *  Constant: '<S49>/Constant'
             */
            qY_1 = qY_4 + calibrazione_acc_mag0_P.Constant_Value_bn44a;
            if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant_Value_bn44a < 0)
                               && (qY_1 >= 0))) {
              qY_1 = MIN_int32_T;
            } else {
              if ((qY_4 > 0) && ((calibrazione_acc_mag0_P.Constant_Value_bn44a >
                                  0) && (qY_1 <= 0))) {
                qY_1 = MAX_int32_T;
              }
            }

            s64_iter = qY_1 - 1;

            /* End of Sum: '<S49>/Sum' */

            /* Sum: '<S49>/Sum2' incorporates:
             *  Constant: '<S49>/Constant1'
             */
            qY_1 = s15_iter - calibrazione_acc_mag0_P.Constant1_Value_bn;
            if ((calibrazione_acc_mag0_P.Constant1_Value_bn < 0) && (qY_1 < 0))
            {
              qY_1 = MAX_int32_T;
            }

            /* End of Sum: '<S49>/Sum2' */

            /* Product: '<S41>/Product' incorporates:
             *  Selector: '<S41>/Selector1'
             */
            rtb_gphih = calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_k[(s15_iter
              - 1) * 13 + s64_iter] * calibrazione_acc_mag0_B.Product4;

            /* Sum: '<S50>/Sum2' incorporates:
             *  Constant: '<S50>/Constant1'
             */
            qY_0 = s15_iter - calibrazione_acc_mag0_P.Constant1_Value_bn4;
            if ((calibrazione_acc_mag0_P.Constant1_Value_bn4 < 0) && (qY_0 < 0))
            {
              qY_0 = MAX_int32_T;
            }

            /* End of Sum: '<S50>/Sum2' */

            /* Switch: '<S41>/Switch' incorporates:
             *  Constant: '<S41>/Constant'
             *  DataTypeConversion: '<S50>/Data Type Conversion'
             *  RelationalOperator: '<S50>/Relational Operator'
             *  Selector: '<S41>/Selector1'
             *  Sum: '<S49>/Sum2'
             *  Sum: '<S50>/Sum2'
             */
            if ((real_T)(qY_0 >= qY_4) >
                calibrazione_acc_mag0_P.Switch_Threshold) {
              rtb_UnitConversion =
                calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_k[(qY_1 - 1) * 13 +
                s64_iter];
            } else {
              rtb_UnitConversion = calibrazione_acc_mag0_P.Constant_Value_bn;
            }

            /* End of Switch: '<S41>/Switch' */

            /* Sum: '<S48>/Sum' incorporates:
             *  Constant: '<S48>/Constant'
             */
            qY_1 = s15_iter + calibrazione_acc_mag0_P.Constant_Value_bn44aw;
            if ((calibrazione_acc_mag0_P.Constant_Value_bn44aw > 0) && (qY_1 <=
                 0)) {
              qY_1 = MAX_int32_T;
            }

            qY_0 = qY_4 + calibrazione_acc_mag0_P.Constant_Value_bn44aw;
            if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant_Value_bn44aw <
                                0) && (qY_0 >= 0))) {
              qY_0 = MIN_int32_T;
            } else {
              if ((qY_4 > 0) && ((calibrazione_acc_mag0_P.Constant_Value_bn44aw >
                                  0) && (qY_0 <= 0))) {
                qY_0 = MAX_int32_T;
              }
            }

            /* End of Sum: '<S48>/Sum' */

            /* Product: '<S41>/Product1' incorporates:
             *  Constant: '<S41>/k[13][13]'
             *  Selector: '<S41>/Selector2'
             *  Sum: '<S48>/Sum'
             */
            rtb_b2 = calibrazione_acc_mag0_P.k1313_Value_b[((qY_1 - 1) * 13 +
              qY_0) - 1] * rtb_UnitConversion;

            /* Gain: '<S47>/Gain' */
            s64_iter = mul_s32_s32_s32_sat(calibrazione_acc_mag0_P.Gain_Gain_bn4,
              qY_4);

            /* Sum: '<S51>/Sum2' incorporates:
             *  Constant: '<S51>/Constant1'
             */
            i = s15_iter - calibrazione_acc_mag0_P.Constant1_Value_bn44a;
            if ((calibrazione_acc_mag0_P.Constant1_Value_bn44a < 0) && (i < 0))
            {
              i = MAX_int32_T;
            }

            /* End of Sum: '<S51>/Sum2' */

            /* Switch: '<S41>/Switch1' incorporates:
             *  Constant: '<S41>/Constant1'
             *  DataTypeConversion: '<S51>/Data Type Conversion'
             *  RelationalOperator: '<S51>/Relational Operator'
             *  Selector: '<S41>/Selector'
             *  Sum: '<S47>/Sum1'
             *  Sum: '<S51>/Sum2'
             *  UnitDelay: '<S25>/Unit Delay1'
             */
            if ((real_T)(i >= qY_4) > calibrazione_acc_mag0_P.Switch1_Threshold)
            {
              /* Selector: '<S41>/Selector' incorporates:
               *  Constant: '<S47>/Constant1'
               *  Sum: '<S47>/Sum1'
               *  Sum: '<S47>/Sum2'
               */
              i = s15_iter - calibrazione_acc_mag0_P.Constant1_Value_bn44;
              if ((calibrazione_acc_mag0_P.Constant1_Value_bn44 < 0) && (i < 0))
              {
                i = MAX_int32_T;
              }

              rtb_Sum2_g = i + s64_iter;
              if ((i < 0) && ((s64_iter < 0) && (rtb_Sum2_g >= 0))) {
                rtb_Sum2_g = MIN_int32_T;
              } else {
                if ((i > 0) && ((s64_iter > 0) && (rtb_Sum2_g <= 0))) {
                  rtb_Sum2_g = MAX_int32_T;
                }
              }

              /* End of Selector: '<S41>/Selector' */
              rtb_UnitConversion =
                calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[rtb_Sum2_g - 1];
            } else {
              rtb_UnitConversion = calibrazione_acc_mag0_P.Constant1_Value_b;
            }

            /* End of Switch: '<S41>/Switch1' */

            /* Selector: '<S41>/Selector' incorporates:
             *  Sum: '<S47>/Sum1'
             */
            i = s15_iter + s64_iter;
            if ((s64_iter > 0) && (i <= 0)) {
              i = MAX_int32_T;
            }

            /* End of Selector: '<S41>/Selector' */

            /* Sum: '<S41>/Sum' incorporates:
             *  Product: '<S41>/Product4'
             *  Selector: '<S41>/Selector'
             *  Sum: '<S47>/Sum1'
             *  UnitDelay: '<S25>/Unit Delay1'
             */
            calibrazione_acc_mag0_B.Merge_h[1] = (rtb_gphih -
              calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[i - 1] *
              calibrazione_acc_mag0_B.sqrt_n) - rtb_b2;

            /* Selector: '<S41>/Selector' incorporates:
             *  Sum: '<S47>/Sum1'
             */
            i = s15_iter + s64_iter;
            if ((s64_iter > 0) && (i <= 0)) {
              i = MAX_int32_T;
            }

            /* End of Selector: '<S41>/Selector' */

            /* Sum: '<S41>/Sum1' incorporates:
             *  Constant: '<S41>/k[13][13]'
             *  Product: '<S41>/Product2'
             *  Product: '<S41>/Product3'
             *  Selector: '<S41>/Selector'
             *  Selector: '<S41>/Selector2'
             *  Sum: '<S47>/Sum1'
             *  Sum: '<S48>/Sum'
             *  UnitDelay: '<S25>/Unit Delay1'
             */
            calibrazione_acc_mag0_B.Merge_h[0] =
              calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[i - 1] *
              calibrazione_acc_mag0_B.Product4 -
              calibrazione_acc_mag0_P.k1313_Value_b[((qY_1 - 1) * 13 + qY_0) - 1]
              * rtb_UnitConversion;

            /* End of Outputs for SubSystem: '<S25>/If Action Subsystem2' */
          }
        }

        /* End of If: '<S25>/if n == m elseif (n==1&m==0) elseif (n>1&m~=n)' */

        /* Assignment: '<S25>/Assignment' incorporates:
         *  Sum: '<S25>/Sum'
         *  UnitDelay: '<S25>/Unit Delay'
         */
        if (calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[2] < 2) {
          calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[2] = 2U;
          memcpy((void *)(&calibrazione_acc_mag0_B.Assignment_n[0]), (void *)
                 calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_k, 169U * sizeof
                 (real_T));
        }

        calibrazione_acc_mag0_B.Assignment_n[(qY_2 + 13 * (qY_3 - 1)) - 1] =
          calibrazione_acc_mag0_B.Merge_h[1];

        /* End of Assignment: '<S25>/Assignment' */

        /* Assignment: '<S25>/Assignment_snorm' incorporates:
         *  Constant: '<S42>/Constant'
         *  Gain: '<S42>/Gain'
         *  Sum: '<S25>/Sum'
         *  Sum: '<S42>/Sum1'
         *  Sum: '<S42>/Sum2'
         *  UnitDelay: '<S25>/Unit Delay1'
         */
        if (calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[3] < 2) {
          calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[3] = 2U;
          memcpy((void *)(&calibrazione_acc_mag0_B.Assignment_snorm[0]), (void *)
                 calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g, 169U * sizeof
                 (real_T));
        }

        qY_1 = qY_2 - calibrazione_acc_mag0_P.Constant_Value_e;
        if ((qY_2 < 0) && ((calibrazione_acc_mag0_P.Constant_Value_e >= 0) &&
                           (qY_1 >= 0))) {
          qY_1 = MIN_int32_T;
        } else {
          if ((qY_2 >= 0) && ((calibrazione_acc_mag0_P.Constant_Value_e < 0) &&
                              (qY_1 < 0))) {
            qY_1 = MAX_int32_T;
          }
        }

        calibrazione_acc_mag0_B.Assignment_snorm[(int32_T)((real_T)qY_3 +
          (real_T)mul_s32_s32_s32_sat(calibrazione_acc_mag0_P.Gain_Gain_eu, qY_1))
          - 1] = calibrazione_acc_mag0_B.Merge_h[0];
        for (i = 0; i < 169; i++) {
          /* Update for UnitDelay: '<S25>/Unit Delay' */
          calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_k[i] =
            calibrazione_acc_mag0_B.Assignment_n[i];

          /* Update for UnitDelay: '<S25>/Unit Delay1' */
          calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[i] =
            calibrazione_acc_mag0_B.Assignment_snorm[i];
        }

        /* End of Assignment: '<S25>/Assignment_snorm' */
      }

      /* End of Outputs for SubSystem: '<S23>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' */

      /* Sum: '<S24>/Sum' incorporates:
       *  Constant: '<S24>/Constant'
       *  Sum: '<S23>/Sum1'
       */
      qY_3 = s15_iter + calibrazione_acc_mag0_P.Constant_Value_i;
      if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_i < 0) &&
                             (qY_3 >= 0))) {
        qY_3 = MIN_int32_T;
      } else {
        if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant_Value_i > 0) &&
             (qY_3 <= 0))) {
          qY_3 = MAX_int32_T;
        }
      }

      qY_2 = qY_4 + calibrazione_acc_mag0_P.Constant_Value_i;
      if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant_Value_i < 0) && (qY_2
            >= 0))) {
        qY_2 = MIN_int32_T;
      } else {
        if ((qY_4 > 0) && ((calibrazione_acc_mag0_P.Constant_Value_i > 0) &&
                           (qY_2 <= 0))) {
          qY_2 = MAX_int32_T;
        }
      }

      /* End of Sum: '<S24>/Sum' */

      /* Selector: '<S24>/dp[n][m]' incorporates:
       *  Sum: '<S24>/Sum'
       */
      rtb_gphih = calibrazione_acc_mag0_B.Assignment_n[((qY_3 - 1) * 13 + qY_2)
        - 1];

      /* Outputs for Enabled SubSystem: '<S23>/Time adjust the gauss coefficients' incorporates:
       *  EnablePort: '<S26>/Enable'
       */
      if (rtb_RelationalOperator) {
        /* Outputs for Atomic SubSystem: '<S26>/If Action Subsystem' */
        /* Sum: '<S52>/Sum1' incorporates:
         *  Constant: '<S52>/Constant1'
         *  Sum: '<S23>/Sum1'
         */
        qY_3 = qY_4 + calibrazione_acc_mag0_P.Constant1_Value_n;
        if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant1_Value_n < 0) &&
                           (qY_3 >= 0))) {
          qY_3 = MIN_int32_T;
        } else {
          if ((qY_4 > 0) && ((calibrazione_acc_mag0_P.Constant1_Value_n > 0) &&
                             (qY_3 <= 0))) {
            qY_3 = MAX_int32_T;
          }
        }

        s64_iter = qY_3 - 1;

        /* End of Sum: '<S52>/Sum1' */

        /* Sum: '<S52>/Sum2' incorporates:
         *  Constant: '<S52>/Constant'
         */
        qY_3 = s15_iter + calibrazione_acc_mag0_P.Constant_Value_n;
        if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_n < 0) &&
             (qY_3 >= 0))) {
          qY_3 = MIN_int32_T;
        } else {
          if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant_Value_n > 0) &&
               (qY_3 <= 0))) {
            qY_3 = MAX_int32_T;
          }
        }

        rtb_Sum2_g = qY_3 - 1;

        /* End of Sum: '<S52>/Sum2' */
        /* End of Outputs for SubSystem: '<S26>/If Action Subsystem' */

        /* Assignment: '<S26>/Assignment' incorporates:
         *  Constant: '<S26>/c[maxdef][maxdef]'
         *  Constant: '<S26>/cd[maxdef][maxdef]'
         *  Product: '<S52>/Product'
         *  Selector: '<S52>/c[m][n]'
         *  Selector: '<S52>/cd[m][n]'
         *  Sum: '<S52>/Sum'
         *  UnitDelay: '<S26>/Unit Delay'
         */
        if (calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[5] < 2) {
          calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[5] = 2U;
          memcpy((void *)&Assignment_d[0], (void *)
                 calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_j, 169U * sizeof
                 (real_T));
        }

        /* Outputs for Atomic SubSystem: '<S26>/If Action Subsystem' */
        Assignment_d[s64_iter + 13 * rtb_Sum2_g] =
          calibrazione_acc_mag0_P.cdmaxdefmaxdef_Value[13 * rtb_Sum2_g +
          s64_iter] * rtb_Sum_i + calibrazione_acc_mag0_P.cmaxdefmaxdef_Value[13
          * rtb_Sum2_g + s64_iter];

        /* End of Assignment: '<S26>/Assignment' */
        /* End of Outputs for SubSystem: '<S26>/If Action Subsystem' */

        /* Switch: '<S53>/tc_old' incorporates:
         *  Constant: '<S53>/zeros(maxdef+1,maxdef+1)'
         *  UnitDelay: '<S53>/Unit Delay'
         */
        for (i = 0; i < 169; i++) {
          if (s15_iter > calibrazione_acc_mag0_P.tc_old_Threshold) {
            rtb_tc_old[i] = calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_g[i];
          } else {
            rtb_tc_old[i] = calibrazione_acc_mag0_P.zerosmaxdef1maxdef1_Value[i];
          }
        }

        /* End of Switch: '<S53>/tc_old' */

        /* If: '<S53>/If' incorporates:
         *  Sum: '<S23>/Sum1'
         */
        if (qY_4 != 0) {
          /* Outputs for IfAction SubSystem: '<S53>/If Action Subsystem1' incorporates:
           *  ActionPort: '<S54>/Action Port'
           */
          /* Sum: '<S54>/Sum2' incorporates:
           *  Constant: '<S54>/Constant'
           */
          qY_3 = s15_iter + calibrazione_acc_mag0_P.Constant_Value_bj;
          if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_bj < 0)
               && (qY_3 >= 0))) {
            qY_3 = MIN_int32_T;
          } else {
            if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant_Value_bj >
                                    0) && (qY_3 <= 0))) {
              qY_3 = MAX_int32_T;
            }
          }

          s64_iter = qY_3 - 1;

          /* End of Sum: '<S54>/Sum2' */

          /* Assignment: '<S54>/Assignment2' incorporates:
           *  Constant: '<S26>/c[maxdef][maxdef]'
           *  Constant: '<S26>/cd[maxdef][maxdef]'
           *  Product: '<S54>/Product'
           *  Selector: '<S54>/c[m][n]'
           *  Selector: '<S54>/cd[m][n]'
           *  Sum: '<S54>/Sum'
           */
          if (calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[6] < 2) {
            calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[6] = 2U;
            memcpy((void *)&Assignment[0], (void *)&rtb_tc_old[0], 169U * sizeof
                   (real_T));
          }

          Assignment[s64_iter + 13 * (qY_4 - 1)] =
            calibrazione_acc_mag0_P.cdmaxdefmaxdef_Value[(qY_4 - 1) * 13 +
            s64_iter] * rtb_Sum_i + calibrazione_acc_mag0_P.cmaxdefmaxdef_Value
            [(qY_4 - 1) * 13 + s64_iter];

          /* End of Assignment: '<S54>/Assignment2' */

          /* Gain: '<S54>/Gain' */
          for (i = 0; i < 169; i++) {
            rtb_tc_old[i] = calibrazione_acc_mag0_P.Gain_Gain * Assignment[i];
          }

          /* End of Gain: '<S54>/Gain' */
          /* End of Outputs for SubSystem: '<S53>/If Action Subsystem1' */
        } else {
        }

        for (i = 0; i < 169; i++) {
          /* Sum: '<S26>/Sum2' */
          calibrazione_acc_mag0_B.Sum2_a[i] = Assignment_d[i] + rtb_tc_old[i];

          /* Update for UnitDelay: '<S26>/Unit Delay' */
          calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_j[i] = Assignment_d[i];

          /* Update for UnitDelay: '<S53>/Unit Delay' */
          calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_g[i] = rtb_tc_old[i];
        }

        /* End of If: '<S53>/If' */
      }

      /* End of Outputs for SubSystem: '<S23>/Time adjust the gauss coefficients' */

      /* Sum: '<S29>/Sum4' incorporates:
       *  Constant: '<S29>/Constant1'
       *  Sum: '<S23>/Sum1'
       */
      rtb_UnitConversion = (real_T)qY_4 +
        calibrazione_acc_mag0_P.Constant1_Value_i;

      /* Selector: '<S29>/cp[m+1]' incorporates:
       *  SignalConversion: '<S29>/TmpSignal ConversionAtcp[m+1]Inport1'
       */
      tmp_2[0] = calibrazione_acc_mag0_B.cp1;
      tmp_2[1] = calibrazione_acc_mag0_B.cp2;

      /* SignalConversion: '<S29>/TmpSignal ConversionAtsp[m+1]Inport1' */
      rtb_TmpSignalConversionAtspm1In[0] = calibrazione_acc_mag0_B.sp1;
      rtb_TmpSignalConversionAtspm1In[1] = calibrazione_acc_mag0_B.sp2;
      for (qY_3 = 0; qY_3 < 11; qY_3++) {
        /* Selector: '<S29>/cp[m+1]' incorporates:
         *  SignalConversion: '<S29>/TmpSignal ConversionAtcp[m+1]Inport1'
         */
        tmp_2[qY_3 + 2] = calibrazione_acc_mag0_B.Assignment1[qY_3];
        rtb_TmpSignalConversionAtspm1In[qY_3 + 2] =
          calibrazione_acc_mag0_B.Assignment[qY_3];
      }

      /* End of SignalConversion: '<S29>/TmpSignal ConversionAtsp[m+1]Inport1' */

      /* If: '<S29>/If' incorporates:
       *  Sum: '<S23>/Sum1'
       */
      if (qY_4 == 0) {
        /* Outputs for IfAction SubSystem: '<S29>/If Action Subsystem' incorporates:
         *  ActionPort: '<S35>/Action Port'
         */
        /* Sum: '<S35>/Sum' incorporates:
         *  Constant: '<S35>/Constant'
         */
        rtb_sincos_o1_p_idx = (real_T)s15_iter +
          calibrazione_acc_mag0_P.Constant_Value_g;

        /* Product: '<S35>/Product' incorporates:
         *  Constant: '<S35>/Constant'
         *  Selector: '<S29>/cp[m+1]'
         *  Selector: '<S29>/sp[m+1]'
         *  Selector: '<S35>/Selector'
         *  Sum: '<S35>/Sum'
         */
        calibrazione_acc_mag0_B.Merge[0] = calibrazione_acc_mag0_B.Sum2_a
          [(((int32_T)rtb_sincos_o1_p_idx - 1) * 13 + (int32_T)
            calibrazione_acc_mag0_P.Constant_Value_g) - 1] * tmp_2[(int32_T)
          rtb_UnitConversion - 1];
        calibrazione_acc_mag0_B.Merge[1] = calibrazione_acc_mag0_B.Sum2_a
          [(((int32_T)rtb_sincos_o1_p_idx - 1) * 13 + (int32_T)
            calibrazione_acc_mag0_P.Constant_Value_g) - 1] *
          rtb_TmpSignalConversionAtspm1In[(int32_T)rtb_UnitConversion - 1];

        /* End of Outputs for SubSystem: '<S29>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S29>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S36>/Action Port'
         */
        /* Sum: '<S37>/Sum' incorporates:
         *  Constant: '<S37>/Constant'
         */
        rtb_sincos_o1_p_idx = (real_T)s15_iter +
          calibrazione_acc_mag0_P.Constant_Value_o;
        rtb_sincos_o1_p_idx_0 = (real_T)qY_4 +
          calibrazione_acc_mag0_P.Constant_Value_o;

        /* Selector: '<S36>/Selector' */
        rtb_Sum1_b = rtb_sincos_o1_p_idx;

        /* Product: '<S36>/Product' incorporates:
         *  Selector: '<S29>/cp[m+1]'
         *  Selector: '<S29>/sp[m+1]'
         *  Selector: '<S36>/Selector'
         */
        rtb_sincos_o1_p_idx = calibrazione_acc_mag0_B.Sum2_a[(((int32_T)
          rtb_sincos_o1_p_idx - 1) * 13 + (int32_T)rtb_sincos_o1_p_idx_0) - 1] *
          tmp_2[(int32_T)rtb_UnitConversion - 1];
        rtb_sincos_o1_p_idx_0 = calibrazione_acc_mag0_B.Sum2_a[(((int32_T)
          rtb_Sum1_b - 1) * 13 + (int32_T)rtb_sincos_o1_p_idx_0) - 1] *
          rtb_TmpSignalConversionAtspm1In[(int32_T)rtb_UnitConversion - 1];

        /* Sum: '<S36>/Sum' incorporates:
         *  Constant: '<S38>/Constant'
         *  Product: '<S36>/Product1'
         *  Selector: '<S29>/sp[m+1]'
         *  Selector: '<S36>/Selector1'
         *  Sum: '<S38>/Sum'
         */
        calibrazione_acc_mag0_B.Merge[0] = calibrazione_acc_mag0_B.Sum2_a[((qY_4
          - 1) * 13 + (int32_T)((real_T)s15_iter +
          calibrazione_acc_mag0_P.Constant_Value_oi)) - 1] *
          rtb_TmpSignalConversionAtspm1In[(int32_T)rtb_UnitConversion - 1] +
          rtb_sincos_o1_p_idx;

        /* Sum: '<S36>/Sum1' incorporates:
         *  Constant: '<S38>/Constant'
         *  Product: '<S36>/Product1'
         *  Selector: '<S29>/cp[m+1]'
         *  Selector: '<S36>/Selector1'
         *  Sum: '<S38>/Sum'
         */
        calibrazione_acc_mag0_B.Merge[1] = rtb_sincos_o1_p_idx_0 -
          calibrazione_acc_mag0_B.Sum2_a[((qY_4 - 1) * 13 + (int32_T)((real_T)
          s15_iter + calibrazione_acc_mag0_P.Constant_Value_oi)) - 1] * tmp_2
          [(int32_T)rtb_UnitConversion - 1];

        /* End of Outputs for SubSystem: '<S29>/If Action Subsystem1' */
      }

      /* End of If: '<S29>/If' */

      /* Selector: '<S24>/snorm[n+m*13]' incorporates:
       *  Constant: '<S28>/Constant'
       *  Gain: '<S28>/Gain'
       *  Sum: '<S23>/Sum1'
       *  Sum: '<S28>/Sum1'
       */
      qY_3 = calibrazione_acc_mag0_P.Constant_Value_h + s15_iter;
      if ((calibrazione_acc_mag0_P.Constant_Value_h < 0) && ((s15_iter < 0) &&
           (qY_3 >= 0))) {
        qY_3 = MIN_int32_T;
      } else {
        if ((calibrazione_acc_mag0_P.Constant_Value_h > 0) && ((s15_iter > 0) &&
             (qY_3 <= 0))) {
          qY_3 = MAX_int32_T;
        }
      }

      i = mul_s32_s32_s32_sat(calibrazione_acc_mag0_P.Gain_Gain_a2, qY_4);
      qY_2 = qY_3 + i;
      if ((qY_3 < 0) && ((i < 0) && (qY_2 >= 0))) {
        qY_2 = MIN_int32_T;
      } else {
        if ((qY_3 > 0) && ((i > 0) && (qY_2 <= 0))) {
          qY_2 = MAX_int32_T;
        }
      }

      /* End of Selector: '<S24>/snorm[n+m*13]' */

      /* Product: '<S24>/par' incorporates:
       *  Selector: '<S24>/snorm[n+m*13]'
       *  Sum: '<S28>/Sum1'
       */
      rtb_UnitConversion = calibrazione_acc_mag0_B.Assignment_snorm[qY_2 - 1] *
        rtb_q;

      /* Sum: '<S24>/Sum4' incorporates:
       *  Constant: '<S24>/Constant1'
       *  Sum: '<S23>/Sum1'
       */
      qY_3 = s15_iter + calibrazione_acc_mag0_P.Constant1_Value_k;
      if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant1_Value_k < 0) &&
                             (qY_3 >= 0))) {
        qY_3 = MIN_int32_T;
      } else {
        if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant1_Value_k > 0) &&
             (qY_3 <= 0))) {
          qY_3 = MAX_int32_T;
        }
      }

      qY_2 = qY_4 + calibrazione_acc_mag0_P.Constant1_Value_k;
      if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant1_Value_k < 0) &&
                         (qY_2 >= 0))) {
        qY_2 = MIN_int32_T;
      } else {
        if ((qY_4 > 0) && ((calibrazione_acc_mag0_P.Constant1_Value_k > 0) &&
                           (qY_2 <= 0))) {
          qY_2 = MAX_int32_T;
        }
      }

      /* End of Sum: '<S24>/Sum4' */

      /* Outputs for Enabled SubSystem: '<S24>/Special case - North//South Geographic Pole' incorporates:
       *  EnablePort: '<S27>/Enable'
       */
      if ((calibrazione_acc_mag0_B.sqrt_n ==
           calibrazione_acc_mag0_P.Constant1_Value_f) &&
          (calibrazione_acc_mag0_P.Constant_Value_ji == qY_4)) {
        if (!calibrazione_acc_mag0_DWork.SpecialcaseNorthSouthGeographic) {
          calibrazione_acc_mag0_DWork.SpecialcaseNorthSouthGeographic = TRUE;
        }

        /* If: '<S27>/n ==1' */
        if (s15_iter == 1) {
          /* Outputs for IfAction SubSystem: '<S27>/If Action Subsystem1' incorporates:
           *  ActionPort: '<S31>/Action Port'
           */
          /* Assignment: '<S31>/Assignment2' incorporates:
           *  Constant: '<S31>/Constant'
           *  Selector: '<S31>/pp[n-1]'
           *  Sum: '<S31>/Sum2'
           *  UnitDelay: '<S27>/Unit Delay1'
           */
          if (calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[0] < 2) {
            calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[0] = 2U;
            memcpy((void *)(&calibrazione_acc_mag0_B.Assignment2_h1[0]), (void *)
                   calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i, 13U * sizeof
                   (real_T));
          }

          calibrazione_acc_mag0_B.Assignment2_h1[(int32_T)(1.0 +
            calibrazione_acc_mag0_P.Constant_Value) - 1] =
            calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i[0];

          /* End of Assignment: '<S31>/Assignment2' */
          /* End of Outputs for SubSystem: '<S27>/If Action Subsystem1' */
        } else {
          /* Outputs for IfAction SubSystem: '<S27>/If Action Subsystem2' incorporates:
           *  ActionPort: '<S32>/Action Port'
           */
          /* Sum: '<S34>/Sum' incorporates:
           *  Constant: '<S34>/Constant'
           */
          qY_1 = s15_iter + calibrazione_acc_mag0_P.Constant_Value_b2;
          if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_b2 < 0)
               && (qY_1 >= 0))) {
            qY_1 = MIN_int32_T;
          } else {
            if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant_Value_b2 >
                                    0) && (qY_1 <= 0))) {
              qY_1 = MAX_int32_T;
            }
          }

          qY_0 = qY_4 + calibrazione_acc_mag0_P.Constant_Value_b2;
          if ((qY_4 < 0) && ((calibrazione_acc_mag0_P.Constant_Value_b2 < 0) &&
                             (qY_0 >= 0))) {
            qY_0 = MIN_int32_T;
          } else {
            if ((qY_4 > 0) && ((calibrazione_acc_mag0_P.Constant_Value_b2 > 0) &&
                               (qY_0 <= 0))) {
              qY_0 = MAX_int32_T;
            }
          }

          /* End of Sum: '<S34>/Sum' */

          /* Assignment: '<S32>/Assignment2' incorporates:
           *  Constant: '<S32>/Constant'
           *  Sum: '<S32>/Sum2'
           *  UnitDelay: '<S27>/Unit Delay1'
           */
          if (calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[1] < 2) {
            calibrazione_acc_mag0_DWork.ForIterator_IterationMarker[1] = 2U;
            memcpy((void *)(&calibrazione_acc_mag0_B.Assignment2_h[0]), (void *)
                   calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i, 13U * sizeof
                   (real_T));
          }

          qY_4 = s15_iter + calibrazione_acc_mag0_P.Constant_Value_b2j;
          if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant_Value_b2j < 0)
               && (qY_4 >= 0))) {
            qY_4 = MIN_int32_T;
          } else {
            if ((s15_iter > 0) && ((calibrazione_acc_mag0_P.Constant_Value_b2j >
                  0) && (qY_4 <= 0))) {
              qY_4 = MAX_int32_T;
            }
          }

          /* End of Assignment: '<S32>/Assignment2' */

          /* Selector: '<S32>/pp[n-2] pp[n-1]' incorporates:
           *  Constant: '<S33>/Constant1'
           *  Sum: '<S33>/Sum2'
           */
          i = s15_iter - calibrazione_acc_mag0_P.Constant1_Value_b2;
          if ((s15_iter < 0) && ((calibrazione_acc_mag0_P.Constant1_Value_b2 >=
                                  0) && (i >= 0))) {
            i = MIN_int32_T;
          } else {
            if ((s15_iter >= 0) && ((calibrazione_acc_mag0_P.Constant1_Value_b2 <
                  0) && (i < 0))) {
              i = MAX_int32_T;
            }
          }

          /* End of Selector: '<S32>/pp[n-2] pp[n-1]' */

          /* Assignment: '<S32>/Assignment2' incorporates:
           *  Constant: '<S32>/k[13][13]'
           *  Product: '<S32>/Product1'
           *  Product: '<S32>/Product2'
           *  Selector: '<S32>/Selector2'
           *  Selector: '<S32>/pp[n-2] pp[n-1]'
           *  Sum: '<S32>/Sum1'
           *  Sum: '<S32>/Sum2'
           *  Sum: '<S33>/Sum2'
           *  Sum: '<S34>/Sum'
           *  UnitDelay: '<S27>/Unit Delay1'
           */
          calibrazione_acc_mag0_B.Assignment2_h[qY_4 - 1] =
            calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i[s15_iter - 1] *
            calibrazione_acc_mag0_B.Product4 -
            calibrazione_acc_mag0_P.k1313_Value[((qY_1 - 1) * 13 + qY_0) - 1] *
            calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i[i - 1];

          /* End of Outputs for SubSystem: '<S27>/If Action Subsystem2' */
        }

        /* End of If: '<S27>/n ==1' */

        /* SignalConversion: '<S27>/TmpSignal ConversionAtpp[n]Inport1' incorporates:
         *  UnitDelay: '<S27>/Unit Delay1'
         */
        rtb_TmpSignalConversionAtspm1In[0] =
          calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i[0];
        rtb_TmpSignalConversionAtspm1In[1] =
          calibrazione_acc_mag0_B.Assignment2_h1[1];
        memcpy((void *)&rtb_TmpSignalConversionAtspm1In[2], (void *)
               (&calibrazione_acc_mag0_B.Assignment2_h[2]), 11U * sizeof(real_T));

        /* Product: '<S27>/Product2' incorporates:
         *  Constant: '<S27>/Constant'
         *  Constant: '<S27>/Constant1'
         *  Selector: '<S27>/pp[n]'
         *  Sum: '<S27>/Sum2'
         */
        calibrazione_acc_mag0_B.Product2 = rtb_TmpSignalConversionAtspm1In
          [(int32_T)((real_T)s15_iter + calibrazione_acc_mag0_P.Constant1_Value)
          - 1] * rtb_q * calibrazione_acc_mag0_P.Constant_Value_b *
          calibrazione_acc_mag0_B.Merge[1];

        /* Update for UnitDelay: '<S27>/Unit Delay1' */
        memcpy((void *)calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i, (void *)
               &rtb_TmpSignalConversionAtspm1In[0], 13U * sizeof(real_T));
      } else {
        if (calibrazione_acc_mag0_DWork.SpecialcaseNorthSouthGeographic) {
          /* Disable for Outport: '<S27>/bpp' */
          calibrazione_acc_mag0_B.Product2 = calibrazione_acc_mag0_P.bpp_Y0;
          calibrazione_acc_mag0_DWork.SpecialcaseNorthSouthGeographic = FALSE;
        }
      }

      /* End of Outputs for SubSystem: '<S24>/Special case - North//South Geographic Pole' */

      /* Sum: '<S24>/Sum1' incorporates:
       *  Constant: '<S30>/Constant'
       *  Constant: '<S30>/Constant1'
       *  Logic: '<S30>/Logical Operator'
       *  Product: '<S24>/Product'
       *  RelationalOperator: '<S30>/Relational Operator'
       *  RelationalOperator: '<S30>/Relational Operator1'
       *  Sum: '<S23>/Sum1'
       *  UnitDelay: '<S24>/Unit Delay1'
       */
      calibrazione_acc_mag0_B.Sum1 =
        calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_h -
        calibrazione_acc_mag0_B.Merge[0] * rtb_gphih * rtb_q;

      /* Sum: '<S24>/Sum2' incorporates:
       *  Constant: '<S24>/fm'
       *  Product: '<S24>/Product1'
       *  Selector: '<S24>/fm[m]'
       *  Sum: '<S24>/Sum4'
       *  UnitDelay: '<S24>/Unit Delay3'
       */
      calibrazione_acc_mag0_B.Sum2 = calibrazione_acc_mag0_P.fm_Value[qY_2 - 1] *
        rtb_UnitConversion * calibrazione_acc_mag0_B.Merge[1] +
        calibrazione_acc_mag0_DWork.UnitDelay3_DSTATE;

      /* Sum: '<S24>/Sum3' incorporates:
       *  Constant: '<S24>/fn'
       *  Product: '<S24>/Product2'
       *  Selector: '<S24>/fn[m]'
       *  Sum: '<S24>/Sum4'
       *  UnitDelay: '<S24>/Unit Delay2'
       */
      calibrazione_acc_mag0_B.Sum3 = calibrazione_acc_mag0_P.fn_Value[qY_3 - 1] *
        rtb_UnitConversion * calibrazione_acc_mag0_B.Merge[0] +
        calibrazione_acc_mag0_DWork.UnitDelay2_DSTATE_d;

      /* Sum: '<S24>/Sum5' incorporates:
       *  UnitDelay: '<S24>/Unit Delay4'
       */
      calibrazione_acc_mag0_B.Sum5 =
        calibrazione_acc_mag0_DWork.UnitDelay4_DSTATE +
        calibrazione_acc_mag0_B.Product2;

      /* Update for UnitDelay: '<S24>/Unit Delay1' */
      calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_h =
        calibrazione_acc_mag0_B.Sum1;

      /* Update for UnitDelay: '<S24>/Unit Delay3' */
      calibrazione_acc_mag0_DWork.UnitDelay3_DSTATE =
        calibrazione_acc_mag0_B.Sum2;

      /* Update for UnitDelay: '<S24>/Unit Delay2' */
      calibrazione_acc_mag0_DWork.UnitDelay2_DSTATE_d =
        calibrazione_acc_mag0_B.Sum3;

      /* Update for UnitDelay: '<S24>/Unit Delay4' */
      calibrazione_acc_mag0_DWork.UnitDelay4_DSTATE =
        calibrazione_acc_mag0_B.Sum5;
    }

    /* End of Sum: '<S15>/Sum' */
    /* End of Outputs for SubSystem: '<S15>/For Iterator Subsystem' */

    /* Sum: '<S15>/Sum1' incorporates:
     *  UnitDelay: '<S15>/Unit Delay2'
     */
    rtb_Sum1_e_idx_1 = rtb_Sum4 + calibrazione_acc_mag0_B.Sum1;
    rtb_Sum1_e_idx_0 = rtb_sincos_o2_idx + calibrazione_acc_mag0_B.Sum2;
    rtb_Sum1_e_idx_2 = rtb_sincos_o1_idx + calibrazione_acc_mag0_B.Sum3;
    rtb_Sum1_e_idx = UnitDelay2_DSTATE_idx + calibrazione_acc_mag0_B.Sum5;

    /* Update for UnitDelay: '<S15>/Unit Delay2' */
    rtb_Sum4 = rtb_Sum1_e_idx_1;
    rtb_sincos_o2_idx = rtb_Sum1_e_idx_0;
    rtb_sincos_o1_idx = rtb_Sum1_e_idx_2;
    UnitDelay2_DSTATE_idx = rtb_Sum1_e_idx;
  }

  /* End of Outputs for SubSystem: '<S13>/Compute magnetic vector in spherical coordinates' */

  /* Switch: '<S67>/Switch' incorporates:
   *  Product: '<S67>/Product'
   */
  if (calibrazione_acc_mag0_B.sqrt_n != 0.0) {
    rtb_UnitConversion = rtb_Sum1_e_idx_0 / calibrazione_acc_mag0_B.sqrt_n;
  } else {
    rtb_UnitConversion = rtb_Sum1_e_idx;
  }

  /* End of Switch: '<S67>/Switch' */

  /* Sum: '<S66>/Sum1' incorporates:
   *  Product: '<S66>/Product1'
   *  Product: '<S66>/Product4'
   */
  rtb_b2 = (0.0 - calibrazione_acc_mag0_B.Product11 * rtb_Sum1_e_idx_1) -
    rtb_Sum1_e_idx_2 * calibrazione_acc_mag0_B.Product12;

  /* Trigonometry: '<S69>/Trigonometric Function1' */
  rtb_gphih = rt_atan2d_snf(rtb_UnitConversion, rtb_b2);

  /* Sum: '<S68>/Sum1' incorporates:
   *  Product: '<S68>/Product1'
   *  Product: '<S68>/Product4'
   */
  rtb_a2 = calibrazione_acc_mag0_B.Product12 * rtb_Sum1_e_idx_1 -
    rtb_Sum1_e_idx_2 * calibrazione_acc_mag0_B.Product11;

  /* Sum: '<S69>/Sum' incorporates:
   *  Product: '<S69>/Product'
   *  Product: '<S69>/Product1'
   */
  rtb_b2 = rtb_UnitConversion * rtb_UnitConversion + rtb_b2 * rtb_b2;

  /* Gain: '<S14>/Unit Conversion' incorporates:
   *  Gain: '<S70>/Unit Conversion'
   */
  rtb_sincos_o1_p_idx = calibrazione_acc_mag0_P.UnitConversion_Gain_k *
    rtb_gphih * calibrazione_acc_mag0_P.UnitConversion_Gain_d;

  /* Trigonometry: '<S10>/sincos' incorporates:
   *  Gain: '<S14>/Unit Conversion'
   *  Gain: '<S71>/Unit Conversion'
   *  Sqrt: '<S69>/sqrt1'
   *  Trigonometry: '<S69>/Trigonometric Function'
   */
  rtb_gphih = cos(calibrazione_acc_mag0_P.UnitConversion_Gain_kp * rt_atan2d_snf
                  (rtb_a2, sqrt(rtb_b2)) *
                  calibrazione_acc_mag0_P.UnitConversion_Gain_d);

  /* Sum: '<S69>/Sum1' incorporates:
   *  Product: '<S69>/Product2'
   */
  rtb_b2 += rtb_a2 * rtb_a2;

  /* Sqrt: '<S69>/sqrt' */
  rtb_b2 = sqrt(rtb_b2);

  /* Gain: '<S1>/Gain1' incorporates:
   *  Gain: '<S3>/Gain1'
   *  Gain: '<S6>/Power Conversion'
   *  Product: '<S10>/h1'
   *  Product: '<S10>/x1'
   *  Trigonometry: '<S10>/sincos'
   */
  rtb_Sum1_b = rtb_gphih * rtb_b2 * cos(rtb_sincos_o1_p_idx) *
    calibrazione_acc_mag0_P.PowerConversion_Gain *
    calibrazione_acc_mag0_P.Gain1_Gain * calibrazione_acc_mag0_P.Gain1_Gain_o;

  /* MATLAB Function Block: '<S1>/mag_calibration' incorporates:
   *  Inport: '<Root>/mag_x'
   *  Inport: '<Root>/mag_y'
   *  Inport: '<Root>/mag_z'
   */

  /* MATLAB Function 'calibrazione_acc_mag/mag_calibration': '<S5>:1' */
  /* H=[]; */
  /* '<S5>:1:5' */
  memset((void *)&H[0], 0, 400U * sizeof(real_T));

  /* '<S5>:1:8' */
  memset((void *)&rtb_G[0], 0, 9U * sizeof(real_T));

  /* f=[0:1000:1]; */
  /* j=[0:1000:1]; */
  /* k=[0:1000:1]; */
  /* B=[]; */
  /* G=[]; */
  /* Hb=[]; */
  /* '<S5>:1:15' */
  /* '<S5>:1:19' */
  for (s64_iter = 0; s64_iter < 10; s64_iter++) {
    /* '<S5>:1:19' */
    /* per far si che H abbia almeno 10 elemnti così m>n */
    /* '<S5>:1:20' */
    H[s64_iter] = rt_powd_snf(calibrazione_acc_mag0_U.mag_x[s64_iter], 2.0);

    /* '<S5>:1:21' */
    H[40 + s64_iter] = 2.0 * calibrazione_acc_mag0_U.mag_x[s64_iter] *
      calibrazione_acc_mag0_U.mag_y[s64_iter];

    /* '<S5>:1:22' */
    H[80 + s64_iter] = 2.0 * calibrazione_acc_mag0_U.mag_x[s64_iter] *
      calibrazione_acc_mag0_U.mag_z[s64_iter];

    /* '<S5>:1:23' */
    H[120 + s64_iter] = rt_powd_snf(calibrazione_acc_mag0_U.mag_y[s64_iter], 2.0);

    /* '<S5>:1:24' */
    H[160 + s64_iter] = 2.0 * calibrazione_acc_mag0_U.mag_y[s64_iter] *
      calibrazione_acc_mag0_U.mag_z[s64_iter];

    /* '<S5>:1:25' */
    H[200 + s64_iter] = rt_powd_snf(calibrazione_acc_mag0_U.mag_z[s64_iter], 2.0);

    /* '<S5>:1:26' */
    H[240 + s64_iter] = calibrazione_acc_mag0_U.mag_x[s64_iter];

    /* '<S5>:1:27' */
    H[280 + s64_iter] = calibrazione_acc_mag0_U.mag_y[s64_iter];

    /* '<S5>:1:28' */
    H[320 + s64_iter] = calibrazione_acc_mag0_U.mag_z[s64_iter];

    /* '<S5>:1:29' */
    H[360 + s64_iter] = 1.0;

    /* '<S5>:1:19' */
  }

  /* for i=11:1:length(magx)-1 %ora inizia l'algoritmo */
  /* '<S5>:1:35' */
  for (s64_iter = 0; s64_iter < 29; s64_iter++) {
    /* '<S5>:1:35' */
    /* ora inizia l'algoritmo */
    /* '<S5>:1:37' */
    H[s64_iter + 10] = rt_powd_snf(calibrazione_acc_mag0_U.mag_x[s64_iter + 10],
      2.0);

    /* '<S5>:1:38' */
    H[s64_iter + 50] = calibrazione_acc_mag0_U.mag_x[s64_iter + 10] * 2.0 *
      calibrazione_acc_mag0_U.mag_y[s64_iter + 10];

    /* '<S5>:1:39' */
    H[s64_iter + 90] = calibrazione_acc_mag0_U.mag_x[s64_iter + 10] * 2.0 *
      calibrazione_acc_mag0_U.mag_z[s64_iter + 10];

    /* '<S5>:1:40' */
    H[s64_iter + 130] = rt_powd_snf(calibrazione_acc_mag0_U.mag_y[s64_iter + 10],
      2.0);

    /* '<S5>:1:41' */
    H[s64_iter + 170] = calibrazione_acc_mag0_U.mag_y[s64_iter + 10] * 2.0 *
      calibrazione_acc_mag0_U.mag_z[s64_iter + 10];

    /* '<S5>:1:42' */
    H[s64_iter + 210] = rt_powd_snf(calibrazione_acc_mag0_U.mag_z[s64_iter + 10],
      2.0);

    /* '<S5>:1:43' */
    H[s64_iter + 250] = calibrazione_acc_mag0_U.mag_x[s64_iter + 10];

    /* '<S5>:1:44' */
    H[s64_iter + 290] = calibrazione_acc_mag0_U.mag_y[s64_iter + 10];

    /* '<S5>:1:45' */
    H[s64_iter + 330] = calibrazione_acc_mag0_U.mag_z[s64_iter + 10];

    /* '<S5>:1:46' */
    H[s64_iter + 370] = 1.0;

    /* '<S5>:1:47' */
    /*  uso la notazione dell'articolo e creo HH */
    for (qY_3 = 0; qY_3 < 10; qY_3++) {
      for (i = 0; i < 10; i++) {
        H_0[qY_3 + 10 * i] = 0.0;
        for (rtb_Sum2_g = 0; rtb_Sum2_g < 40; rtb_Sum2_g++) {
          H_0[qY_3 + 10 * i] = H[40 * qY_3 + rtb_Sum2_g] * H[40 * i + rtb_Sum2_g]
            + H_0[10 * i + qY_3];
        }
      }
    }

    calibrazione_acc_mag0_svd(H_0, K, P, L);

    /* HH è una matrice quadrata da cui k è uguale a L */
    /* '<S5>:1:49' */
    /*  mi fido dell'articolo e prendo la decima colonna che in effetti è relativa al 10° valore singolare che è il + piccolo */
    /* '<S5>:1:50' */
    /* '<S5>:1:51' */
    /* '<S5>:1:52' */
    /* '<S5>:1:53' */
    /* '<S5>:1:54' */
    /* '<S5>:1:55' */
    /* '<S5>:1:56' */
    /* '<S5>:1:57' */
    /* '<S5>:1:58' */
    /* '<S5>:1:59' */
    /* '<S5>:1:60' */
    /* '<S5>:1:61' */
    K_0[0] = K[90];
    K_0[3] = K[91];
    K_0[6] = K[92];
    K_0[1] = K[91];
    K_0[4] = K[93];
    K_0[7] = K[94];
    K_0[2] = K[92];
    K_0[5] = K[94];
    K_0[8] = K[95];
    calibrazione_acc_mag0_inv(K_0, U);
    for (qY_3 = 0; qY_3 < 3; qY_3++) {
      rtb_b[qY_3] = U[qY_3 + 6] * -0.5 * K[98] + (U[qY_3 + 3] * -0.5 * K[97] +
        -0.5 * U[qY_3] * K[96]);
    }

    /* '<S5>:1:62' */
    /* '<S5>:1:63' */
    /* '<S5>:1:64' */
    /* creo B per notare una convergenza */
    /* '<S5>:1:68' */
    rtb_gphih = rt_powd_snf((((((2.0 * K[91] * rtb_b[0] * rtb_b[1] + K[90] *
      rt_powd_snf(rtb_b[0], 2.0)) + 2.0 * K[92] * rtb_b[0] * rtb_b[2]) + K[93] *
      rt_powd_snf(rtb_b[1], 2.0)) + 2.0 * K[94] * rtb_b[1] * rtb_b[2]) + 2.0 *
      K[95] * rt_powd_snf(rtb_b[2], 2.0)) - K[99], -1.0) * (rtb_Sum1_b *
      rtb_Sum1_b);

    /*  credo di poter elevare alla -1 tanto è uno scalare */
    /* '<S5>:1:69' */
    for (qY_3 = 0; qY_3 < 10; qY_3++) {
      gsi[qY_3] = K[90 + qY_3] * rtb_gphih;
    }

    /* '<S5>:1:70' */
    /* '<S5>:1:71' */
    /* '<S5>:1:72' */
    /* '<S5>:1:73' */
    /* '<S5>:1:74' */
    /* '<S5>:1:75' */
    /* '<S5>:1:80' */
    K_0[0] = gsi[0];
    K_0[3] = gsi[1];
    K_0[6] = gsi[2];
    K_0[1] = gsi[1];
    K_0[4] = gsi[3];
    K_0[7] = gsi[4];
    K_0[2] = gsi[2];
    K_0[5] = gsi[4];
    K_0[8] = gsi[5];
    calibrazione_acc_mag0_svd_d(K_0, U, A, V);

    /* per come è costruita gamma è quadrata e simmetrica da cui U è uguale a V */
    /* '<S5>:1:82' */
    calibrazione_acc_mag0_sqrt(A);
    for (qY_3 = 0; qY_3 < 3; qY_3++) {
      for (i = 0; i < 3; i++) {
        K_0[qY_3 + 3 * i] = 0.0;
        K_0[qY_3 + 3 * i] = K_0[3 * i + qY_3] + A[3 * i] * V[qY_3];
        K_0[qY_3 + 3 * i] = A[3 * i + 1] * V[qY_3 + 3] + K_0[3 * i + qY_3];
        K_0[qY_3 + 3 * i] = A[3 * i + 2] * V[qY_3 + 6] + K_0[3 * i + qY_3];
      }
    }

    for (qY_3 = 0; qY_3 < 3; qY_3++) {
      for (i = 0; i < 3; i++) {
        rtb_G[qY_3 + 3 * i] = 0.0;
        rtb_G[qY_3 + 3 * i] = rtb_G[3 * i + qY_3] + K_0[qY_3] * V[i];
        rtb_G[qY_3 + 3 * i] = rtb_G[3 * i + qY_3] + K_0[qY_3 + 3] * V[i + 3];
        rtb_G[qY_3 + 3 * i] = rtb_G[3 * i + qY_3] + K_0[qY_3 + 6] * V[i + 6];
      }
    }

    /* '<S5>:1:83' */
    rtb_sincos_o1_p_idx_0 = rtb_b[2];
    rtb_b[2] = rtb_sincos_o1_p_idx_0;

    /* f(i)=norm(hm); %norma del vettore misurato */
    /* j(i)=norm(hb); %norma del vettore calibrato */
    /* k(i)=norm(h);  %norma del vettore reale */
    /* '<S5>:1:35' */
  }

  /* End of MATLAB Function Block: '<S1>/mag_calibration' */

  /* Outport: '<Root>/G_mag' */
  memcpy((void *)&calibrazione_acc_mag0_Y.G_mag[0], (void *)&rtb_G[0], 9U *
         sizeof(real_T));

  /* Outport: '<Root>/Bias_mag' */
  calibrazione_acc_mag0_Y.Bias_mag[0] = rtb_b[0];
  calibrazione_acc_mag0_Y.Bias_mag[1] = rtb_b[1];
  calibrazione_acc_mag0_Y.Bias_mag[2] = rtb_b[2];

  /* Assertion: '<S7>/Assertion' incorporates:
   *  Constant: '<S7>/max_val'
   *  Constant: '<S7>/min_val'
   *  Logic: '<S7>/conjunction'
   *  RelationalOperator: '<S7>/max_relop'
   *  RelationalOperator: '<S7>/min_relop'
   */
  utAssert((calibrazione_acc_mag0_P.min_val_Value <= rtb_UnitConversion_d) &&
           (rtb_UnitConversion_d <= calibrazione_acc_mag0_P.max_val_Value));

  /* Assertion: '<S8>/Assertion' incorporates:
   *  Constant: '<S8>/max_val'
   *  Constant: '<S8>/min_val'
   *  Inport: '<Root>/position'
   *  Logic: '<S8>/conjunction'
   *  RelationalOperator: '<S8>/max_relop'
   *  RelationalOperator: '<S8>/min_relop'
   */
  utAssert((calibrazione_acc_mag0_P.min_val_Value_e <=
            calibrazione_acc_mag0_U.position[0]) &&
           (calibrazione_acc_mag0_U.position[0] <=
            calibrazione_acc_mag0_P.max_val_Value_g));

  /* Assertion: '<S9>/Assertion' incorporates:
   *  Constant: '<S9>/max_val'
   *  Constant: '<S9>/min_val'
   *  Inport: '<Root>/position'
   *  Logic: '<S9>/conjunction'
   *  RelationalOperator: '<S9>/max_relop'
   *  RelationalOperator: '<S9>/min_relop'
   */
  utAssert((calibrazione_acc_mag0_P.min_val_Value_eh <=
            calibrazione_acc_mag0_U.position[1]) &&
           (calibrazione_acc_mag0_U.position[1] <=
            calibrazione_acc_mag0_P.max_val_Value_k));

  /* Assertion: '<S11>/Assertion' incorporates:
   *  Constant: '<S11>/max_val'
   *  Constant: '<S11>/min_val'
   *  Logic: '<S11>/conjunction'
   *  RelationalOperator: '<S11>/max_relop'
   *  RelationalOperator: '<S11>/min_relop'
   */
  utAssert((calibrazione_acc_mag0_P.min_val_Value_c <= rtb_Sum_i) && (rtb_Sum_i <=
            calibrazione_acc_mag0_P.max_val_Value_gm));

  /* Update for Memory: '<S21>/otime' */
  calibrazione_acc_mag0_DWork.otime_PreviousInput = rtb_Sum;

  /* Update for Memory: '<S20>/olon' */
  calibrazione_acc_mag0_DWork.olon_PreviousInput = rtb_u80deg;

  /* Update for Memory: '<S19>/olat' */
  calibrazione_acc_mag0_DWork.olat_PreviousInput = rtb_u0deg;

  /* Update for Memory: '<S19>/oalt' */
  calibrazione_acc_mag0_DWork.oalt_PreviousInput = rtb_Gain;
}

/* Model initialize function */
void calibrazione_acc_mag0_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(calibrazione_acc_mag0_M, (NULL));

  /* block I/O */
  (void) memset(((void *) &calibrazione_acc_mag0_B), 0,
                sizeof(BlockIO_calibrazione_acc_mag0));

  /* states (dwork) */
  (void) memset((void *)&calibrazione_acc_mag0_DWork, 0,
                sizeof(D_Work_calibrazione_acc_mag0));

  /* external inputs */
  (void) memset((void *)&calibrazione_acc_mag0_U, 0,
                sizeof(ExternalInputs_calibrazione_acc));

  /* external outputs */
  (void) memset((void *)&calibrazione_acc_mag0_Y, 0,
                sizeof(ExternalOutputs_calibrazione_ac));

  {
    int32_T i;

    /* Start for Enabled SubSystem: '<S13>/Convert from geodetic to  spherical coordinates ' */
    /* Start for Iterator SubSystem: '<S17>/For Iterator Subsystem' */
    /* InitializeConditions for UnitDelay: '<S64>/Unit Delay1' */
    calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE[0] =
      calibrazione_acc_mag0_P.UnitDelay1_X0_k;
    calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE[1] =
      calibrazione_acc_mag0_P.UnitDelay1_X0_k;

    /* End of Start for SubSystem: '<S17>/For Iterator Subsystem' */

    /* VirtualOutportStart for Outport: '<S17>/sp[13]' */
    calibrazione_acc_mag0_B.sp1 = calibrazione_acc_mag0_P.sp13_Y0[0];
    calibrazione_acc_mag0_B.sp2 = calibrazione_acc_mag0_P.sp13_Y0[1];

    /* VirtualOutportStart for Outport: '<S17>/cp[13]' */
    calibrazione_acc_mag0_B.cp1 = calibrazione_acc_mag0_P.cp13_Y0[0];
    calibrazione_acc_mag0_B.cp2 = calibrazione_acc_mag0_P.cp13_Y0[1];
    for (i = 0; i < 11; i++) {
      /* VirtualOutportStart for Outport: '<S17>/sp[13]' */
      calibrazione_acc_mag0_B.Assignment[i] = calibrazione_acc_mag0_P.sp13_Y0[i
        + 2];
      calibrazione_acc_mag0_B.Assignment1[i] = calibrazione_acc_mag0_P.cp13_Y0[i
        + 2];
    }

    /* End of Start for SubSystem: '<S13>/Convert from geodetic to  spherical coordinates ' */

    /* Start for Iterator SubSystem: '<S13>/Compute magnetic vector in spherical coordinates' */
    /* Start for Iterator SubSystem: '<S15>/For Iterator Subsystem' */
    /* Start for Enabled SubSystem: '<S23>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' */
    /* Start for Enabled SubSystem: '<S23>/Time adjust the gauss coefficients' */
    for (i = 0; i < 169; i++) {
      /* InitializeConditions for UnitDelay: '<S25>/Unit Delay' */
      calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_k[i] =
        calibrazione_acc_mag0_P.UnitDelay_X0[i];

      /* InitializeConditions for UnitDelay: '<S25>/Unit Delay1' */
      calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_g[i] =
        calibrazione_acc_mag0_P.UnitDelay1_X0_d[i];

      /* InitializeConditions for UnitDelay: '<S26>/Unit Delay' */
      calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_j[i] =
        calibrazione_acc_mag0_P.UnitDelay_X0_g[i];

      /* InitializeConditions for UnitDelay: '<S53>/Unit Delay' */
      calibrazione_acc_mag0_DWork.UnitDelay_DSTATE_g[i] =
        calibrazione_acc_mag0_P.UnitDelay_X0_o[i];
    }

    /* End of Outport: '<S17>/cp[13]' */
    /* End of Start for SubSystem: '<S23>/Time adjust the gauss coefficients' */
    /* End of Start for SubSystem: '<S23>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' */

    /* Start for Enabled SubSystem: '<S24>/Special case - North//South Geographic Pole' */
    /* InitializeConditions for UnitDelay: '<S27>/Unit Delay1' */
    memcpy((void *)calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_i, (void *)
           calibrazione_acc_mag0_P.UnitDelay1_X0, 13U * sizeof(real_T));

    /* VirtualOutportStart for Outport: '<S27>/bpp' */
    calibrazione_acc_mag0_B.Product2 = calibrazione_acc_mag0_P.bpp_Y0;

    /* End of Start for SubSystem: '<S24>/Special case - North//South Geographic Pole' */

    /* InitializeConditions for UnitDelay: '<S24>/Unit Delay1' */
    calibrazione_acc_mag0_DWork.UnitDelay1_DSTATE_h =
      calibrazione_acc_mag0_P.UnitDelay1_X0_h;

    /* InitializeConditions for UnitDelay: '<S24>/Unit Delay3' */
    calibrazione_acc_mag0_DWork.UnitDelay3_DSTATE =
      calibrazione_acc_mag0_P.UnitDelay3_X0;

    /* InitializeConditions for UnitDelay: '<S24>/Unit Delay2' */
    calibrazione_acc_mag0_DWork.UnitDelay2_DSTATE_d =
      calibrazione_acc_mag0_P.UnitDelay2_X0;

    /* InitializeConditions for UnitDelay: '<S24>/Unit Delay4' */
    calibrazione_acc_mag0_DWork.UnitDelay4_DSTATE =
      calibrazione_acc_mag0_P.UnitDelay4_X0;

    /* End of Start for SubSystem: '<S15>/For Iterator Subsystem' */
    /* End of Start for SubSystem: '<S13>/Compute magnetic vector in spherical coordinates' */
  }

  /* InitializeConditions for Memory: '<S21>/otime' */
  calibrazione_acc_mag0_DWork.otime_PreviousInput =
    calibrazione_acc_mag0_P.otime_X0;

  /* InitializeConditions for Memory: '<S20>/olon' */
  calibrazione_acc_mag0_DWork.olon_PreviousInput =
    calibrazione_acc_mag0_P.olon_X0;

  /* InitializeConditions for Memory: '<S19>/olat' */
  calibrazione_acc_mag0_DWork.olat_PreviousInput =
    calibrazione_acc_mag0_P.olat_X0;

  /* InitializeConditions for Memory: '<S19>/oalt' */
  calibrazione_acc_mag0_DWork.oalt_PreviousInput =
    calibrazione_acc_mag0_P.oalt_X0;
}

/* Model terminate function */
void calibrazione_acc_mag0_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
