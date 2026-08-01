/*
 * File: modelv2.c
 *
 * Code generated for Simulink model 'modelv2'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Sun Jun 28 16:23:16 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 10
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "modelv2.h"
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include "math.h"

/* Named constants for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
#define degrees                        (9)
#define ny                             (3)
#define p                              (15)

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Continuous states */
X rtX;

/*
 * Mission parameter: desired RELATIVE approach/docking setpoint
 * [rel_x, rel_y, rel_yaw], expressed in the same relative frame as the
 * controller's state (chaser pose with respect to the target). This model
 * has no separate "target dynamics" -- modelv2_derivatives() shows the
 * continuous state's derivative depends only on the state itself and the
 * control input, confirming the 3-element state was designed from the
 * start to BE the relative pose, not the chaser's absolute pose. This
 * global replaces what was originally a hardcoded fixed setpoint
 * (1.1401911496249473, 1.3034355291722768, 0.5) from a 'static_ref' block.
 * It is a mission-design parameter (e.g. a station-keeping offset or final
 * docking pose), not something to overwrite every cycle from a live
 * measurement -- see the ROS 2 node for how it's set.
 */
real_T rtP_RelativeSetpoint[3] = { 1.1401911496249473, 1.3034355291722768, 0.5 };

/* Disabled State Vector */
XDis rtXDis;

/* Block signals and states (default storage) */
DW rtDW;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_roundd_snf(real_T u);
extern real_T rt_hypotd_snf(real_T u0, real_T u1);
static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

/* private model entry point functions */
extern void modelv2_derivatives(void);

/* Forward declaration for local functions */
static void mpc_plantupdate(const real_T a[9], real_T b[12], real_T c[9], real_T
  b_A[9], real_T b_B_2[24], real_T b_C[9], const real_T b_D[24], const int32_T
  b_mvindex[4], const int32_T b_myindex[3], const real_T b_Uscale[4], const
  real_T b_Yscale[3], real_T Bu[12], real_T Bv[3], real_T Cm[9], real_T Dv[3],
  real_T Dvm[3], real_T QQ[9], real_T RR[9], real_T NN[9]);
static int32_T xpotrf(real_T b_A[81]);
static real_T minimum(const real_T x[9]);
static void trisolve(const real_T b_A[81], real_T b_B_3[81]);
static real_T norm(const real_T x[9]);
static real_T maximum(const real_T x[9]);
static real_T xnrm2(int32_T n, const real_T x[81], int32_T ix0);
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[81], int32_T ia0,
                  const real_T x[81], int32_T ix0, real_T y[9]);
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[9], real_T b_A[81], int32_T ia0);
static real_T KWIKfactor(const real_T b_Ac[288], const int32_T iC[32], int32_T
  nA, const real_T b_Linv[81], real_T RLinv[81], real_T b_D[81], real_T b_H[81],
  int32_T n);
static void DropConstraint(int32_T kDrop, boolean_T iA[32], int32_T *nA, int32_T
  iC[32]);
static void qpkwik(const real_T b_Linv[81], const real_T b_Hinv[81], const
                   real_T f[9], const real_T b_Ac[288], const real_T b[32],
                   boolean_T iA[32], int32_T maxiter, real_T FeasTol, real_T x[9],
                   real_T lambda[32], int32_T *status);
static void mpcblock_optimizer(const real_T rseq[45], const real_T vseq[16],
  const real_T x[3], const real_T old_u[4], const boolean_T iA[32], const real_T
  b_Mlim[32], real_T b_Mx[96], real_T b_Mu1[128], real_T b_Mv[512], const real_T
  b_utarget[60], const real_T b_uoff[4], real_T b_H[81], real_T b_Ac[288], const
  real_T b_Wy[3], const real_T b_Wdu[4], const real_T b_Jm[480], const real_T
  b_Wu[4], const real_T b_I1[240], const real_T b_A[144], const real_T Bu[192],
  const real_T Bv[48], const real_T b_C[144], const real_T Dv[48], const int32_T
  b_Mrows[32], real_T u[4], real_T useq[64], real_T *status, boolean_T iAout[32]);
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/* Return rtInf needed by the generated code. */
static real_T rtGetInf(void)
{
  return rtInf;
}

/* Get rtInfF needed by the generated code. */
static real32_T rtGetInfF(void)
{
  return rtInfF;
}

/* Return rtMinusInf needed by the generated code. */
static real_T rtGetMinusInf(void)
{
  return rtMinusInf;
}

/* Return rtMinusInfF needed by the generated code. */
static real32_T rtGetMinusInfF(void)
{
  return rtMinusInfF;
}

/* Return rtNaN needed by the generated code. */
static real_T rtGetNaN(void)
{
  return rtNaN;
}

/* Return rtNaNF needed by the generated code. */
static real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  modelv2_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  modelv2_step();
  modelv2_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  modelv2_step();
  modelv2_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  modelv2_step();
  modelv2_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void mpc_plantupdate(const real_T a[9], real_T b[12], real_T c[9], real_T
  b_A[9], real_T b_B_2[24], real_T b_C[9], const real_T b_D[24], const int32_T
  b_mvindex[4], const int32_T b_myindex[3], const real_T b_Uscale[4], const
  real_T b_Yscale[3], real_T Bu[12], real_T Bv[3], real_T Cm[9], real_T Dv[3],
  real_T Dvm[3], real_T QQ[9], real_T RR[9], real_T NN[9])
{
  real_T b_tmp[42];
  real_T CovMat[36];
  real_T b_Uscale_0;
  real_T b_Yscale_0;
  real_T b_Yscale_1;
  int32_T b_myindex_0;
  int32_T b_myindex_1;
  int32_T b_tmp_0;
  int32_T i;
  int8_T UnknownIn[7];
  for (i = 0; i < 4; i++) {
    b_Uscale_0 = b_Uscale[i];
    b[3 * i] *= b_Uscale_0;
    b_tmp_0 = 3 * i + 1;
    b[b_tmp_0] *= b_Uscale_0;
    b_tmp_0 = 3 * i + 2;
    b[b_tmp_0] *= b_Uscale_0;
  }

  b_Uscale_0 = b_Yscale[0];
  b_Yscale_0 = b_Yscale[1];
  b_Yscale_1 = b_Yscale[2];
  for (i = 0; i < 3; i++) {
    c[3 * i] /= b_Uscale_0;
    b_tmp_0 = 3 * i + 1;
    c[b_tmp_0] /= b_Yscale_0;
    b_tmp_0 = 3 * i + 2;
    c[b_tmp_0] /= b_Yscale_1;
  }

  memcpy(&b_A[0], &a[0], 9U * sizeof(real_T));
  for (i = 0; i < 4; i++) {
    b_tmp_0 = (b_mvindex[i] - 1) * 3;
    b_B_2[3 * i] = b[b_tmp_0];
    b_B_2[3 * i + 1] = b[b_tmp_0 + 1];
    b_B_2[3 * i + 2] = b[b_tmp_0 + 2];
  }

  memcpy(&b_C[0], &c[0], 9U * sizeof(real_T));
  for (i = 0; i < 4; i++) {
    Bu[3 * i] = b_B_2[3 * i];
    b_tmp_0 = 3 * i + 1;
    Bu[b_tmp_0] = b_B_2[b_tmp_0];
    b_tmp_0 = 3 * i + 2;
    Bu[b_tmp_0] = b_B_2[b_tmp_0];
  }

  for (i = 0; i < 3; i++) {
    Bv[i] = b_B_2[i + 12];
    Cm[3 * i] = c[(3 * i + b_myindex[0]) - 1];
    Cm[3 * i + 1] = c[(3 * i + b_myindex[1]) - 1];
    Cm[3 * i + 2] = c[(3 * i + b_myindex[2]) - 1];
    Dv[i] = b_D[i + 12];
    Dvm[i] = b_D[b_myindex[i] + 11];
  }

  UnknownIn[0] = 1;
  UnknownIn[1] = 2;
  UnknownIn[2] = 3;
  UnknownIn[3] = 4;
  UnknownIn[4] = 6;
  UnknownIn[5] = 7;
  UnknownIn[6] = 8;
  b_tmp_0 = b_myindex[0];
  b_myindex_0 = b_myindex[1];
  b_myindex_1 = b_myindex[2];
  for (i = 0; i < 7; i++) {
    int32_T b_tmp_tmp;
    b_tmp_tmp = (UnknownIn[i] - 1) * 3;
    b_tmp[6 * i] = b_B_2[b_tmp_tmp];
    b_tmp[6 * i + 3] = b_D[(b_tmp_tmp + b_tmp_0) - 1];
    b_tmp[6 * i + 1] = b_B_2[b_tmp_tmp + 1];
    b_tmp[6 * i + 4] = b_D[(b_tmp_tmp + b_myindex_0) - 1];
    b_tmp[6 * i + 2] = b_B_2[b_tmp_tmp + 2];
    b_tmp[6 * i + 5] = b_D[(b_tmp_tmp + b_myindex_1) - 1];
  }

  for (i = 0; i < 6; i++) {
    for (b_tmp_0 = 0; b_tmp_0 < 6; b_tmp_0++) {
      b_Uscale_0 = 0.0;
      for (b_myindex_0 = 0; b_myindex_0 < 7; b_myindex_0++) {
        b_Uscale_0 += b_tmp[6 * b_myindex_0 + i] * b_tmp[6 * b_myindex_0 +
          b_tmp_0];
      }

      CovMat[i + 6 * b_tmp_0] = b_Uscale_0;
    }
  }

  for (i = 0; i < 3; i++) {
    QQ[3 * i] = CovMat[6 * i];
    b_tmp_0 = (i + 3) * 6;
    RR[3 * i] = CovMat[b_tmp_0 + 3];
    NN[3 * i] = CovMat[b_tmp_0];
    b_myindex_0 = 3 * i + 1;
    QQ[b_myindex_0] = CovMat[6 * i + 1];
    RR[b_myindex_0] = CovMat[b_tmp_0 + 4];
    NN[b_myindex_0] = CovMat[b_tmp_0 + 1];
    b_myindex_0 = 3 * i + 2;
    QQ[b_myindex_0] = CovMat[6 * i + 2];
    RR[b_myindex_0] = CovMat[b_tmp_0 + 5];
    NN[b_myindex_0] = CovMat[b_tmp_0 + 2];
  }
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static int32_T xpotrf(real_T b_A[81])
{
  int32_T b_k;
  int32_T info;
  int32_T j;
  int32_T jm1;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 9)) {
    real_T c;
    real_T ssq;
    int32_T idxAjj;
    idxAjj = j * 9 + j;
    ssq = 0.0;
    if (j >= 1) {
      for (b_k = 0; b_k < j; b_k++) {
        c = b_A[b_k * 9 + j];
        ssq += c * c;
      }
    }

    ssq = b_A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      b_A[idxAjj] = ssq;
      if (j + 1 < 9) {
        if (j != 0) {
          int32_T b_iy;
          b_iy = ((j - 1) * 9 + j) + 2;
          for (b_k = j + 2; b_k <= b_iy; b_k += 9) {
            int32_T d;
            jm1 = b_k - j;
            c = -b_A[div_nde_s32_floor(jm1 - 2, 9) * 9 + j];
            d = jm1 + 7;
            for (jm1 = b_k; jm1 <= d; jm1++) {
              int32_T tmp;
              tmp = ((idxAjj + jm1) - b_k) + 1;
              b_A[tmp] += b_A[jm1 - 1] * c;
            }
          }
        }

        ssq = 1.0 / ssq;
        jm1 = (idxAjj - j) + 9;
        for (b_k = idxAjj + 2; b_k <= jm1; b_k++) {
          b_A[b_k - 1] *= ssq;
        }
      }

      j++;
    } else {
      b_A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static real_T minimum(const real_T x[9])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 10)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 10; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (ex > x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
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

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void trisolve(const real_T b_A[81], real_T b_B_3[81])
{
  int32_T b_k;
  int32_T i;
  int32_T j;
  for (j = 0; j < 9; j++) {
    int32_T jBcol;
    jBcol = 9 * j;
    for (b_k = 0; b_k < 9; b_k++) {
      real_T b_B_4;
      int32_T b_B_tmp;
      int32_T kAcol;
      kAcol = 9 * b_k;
      b_B_tmp = b_k + jBcol;
      b_B_4 = b_B_3[b_B_tmp];
      if (b_B_4 != 0.0) {
        b_B_3[b_B_tmp] = b_B_4 / b_A[b_k + kAcol];
        for (i = b_k + 2; i < 10; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          b_B_3[tmp] -= b_A[(i + kAcol) - 1] * b_B_3[b_B_tmp];
        }
      }
    }
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static real_T norm(const real_T x[9])
{
  real_T scale;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 9; k++) {
    real_T absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static real_T maximum(const real_T x[9])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 10)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 10; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static real_T xnrm2(int32_T n, const real_T x[81], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[81], int32_T ia0,
                  const real_T x[81], int32_T ix0, real_T y[9])
{
  int32_T b_iy;
  int32_T ia;
  if ((b_m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 9 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 9) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nde_s32_floor(b_iy - ia0, 9);
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[9], real_T b_A[81], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = (b_m + jA) - 1;
        for (ijA = jA; ijA <= b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 9;
    }
  }
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static real_T KWIKfactor(const real_T b_Ac[288], const int32_T iC[32], int32_T
  nA, const real_T b_Linv[81], real_T RLinv[81], real_T b_D[81], real_T b_H[81],
  int32_T n)
{
  real_T R[81];
  real_T TL[81];
  real_T b_A[81];
  real_T tau[9];
  real_T work[9];
  real_T RLinv_0;
  real_T Status;
  real_T b_A_0;
  real_T beta1;
  int32_T b_coltop;
  int32_T b_lastv;
  int32_T coltop;
  int32_T exitg1;
  int32_T ii;
  int32_T k_i;
  int32_T knt;
  boolean_T exitg2;
  Status = 1.0;
  memset(&RLinv[0], 0, 81U * sizeof(real_T));
  for (k_i = 0; k_i < nA; k_i++) {
    b_lastv = iC[k_i];
    for (b_coltop = 0; b_coltop < 9; b_coltop++) {
      RLinv_0 = 0.0;
      for (knt = 0; knt < 9; knt++) {
        RLinv_0 += b_Ac[((knt << 5) + b_lastv) - 1] * b_Linv[9 * knt + b_coltop];
      }

      RLinv[b_coltop + 9 * k_i] = RLinv_0;
    }
  }

  memcpy(&b_A[0], &RLinv[0], 81U * sizeof(real_T));
  memset(&tau[0], 0, 9U * sizeof(real_T));
  memset(&work[0], 0, 9U * sizeof(real_T));
  for (k_i = 0; k_i < 9; k_i++) {
    ii = k_i * 9 + k_i;
    if (k_i + 1 < 9) {
      RLinv_0 = b_A[ii];
      b_lastv = ii + 2;
      tau[k_i] = 0.0;
      beta1 = xnrm2(8 - k_i, b_A, ii + 2);
      if (beta1 != 0.0) {
        b_A_0 = b_A[ii];
        beta1 = rt_hypotd_snf(b_A_0, beta1);
        if (b_A_0 >= 0.0) {
          beta1 = -beta1;
        }

        if (fabs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          coltop = (ii - k_i) + 9;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
              b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            RLinv_0 *= 9.9792015476736E+291;
          } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

          beta1 = rt_hypotd_snf(RLinv_0, xnrm2(8 - k_i, b_A, ii + 2));
          if (RLinv_0 >= 0.0) {
            beta1 = -beta1;
          }

          tau[k_i] = (beta1 - RLinv_0) / beta1;
          RLinv_0 = 1.0 / (RLinv_0 - beta1);
          for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
            b_A[b_coltop - 1] *= RLinv_0;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 1.0020841800044864E-292;
          }

          RLinv_0 = beta1;
        } else {
          tau[k_i] = (beta1 - b_A_0) / beta1;
          RLinv_0 = 1.0 / (b_A_0 - beta1);
          b_coltop = (ii - k_i) + 9;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            b_A[knt - 1] *= RLinv_0;
          }

          RLinv_0 = beta1;
        }
      }

      b_A[ii] = 1.0;
      if (tau[k_i] != 0.0) {
        b_lastv = 9 - k_i;
        knt = (ii - k_i) + 8;
        while ((b_lastv > 0) && (b_A[knt] == 0.0)) {
          b_lastv--;
          knt--;
        }

        knt = 8 - k_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = ((knt - 1) * 9 + ii) + 9;
          coltop = b_coltop;
          do {
            exitg1 = 0;
            if (coltop + 1 <= b_coltop + b_lastv) {
              if (b_A[coltop] != 0.0) {
                exitg1 = 1;
              } else {
                coltop++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        knt = 0;
      }

      if (b_lastv > 0) {
        xgemv(b_lastv, knt, b_A, ii + 10, b_A, ii + 1, work);
        xgerc(b_lastv, knt, -tau[k_i], ii + 1, work, b_A, ii + 10);
      }

      b_A[ii] = RLinv_0;
    } else {
      tau[8] = 0.0;
    }
  }

  for (k_i = 0; k_i < 9; k_i++) {
    for (ii = 0; ii <= k_i; ii++) {
      R[ii + 9 * k_i] = b_A[9 * k_i + ii];
    }

    for (ii = k_i + 2; ii < 10; ii++) {
      R[(ii + 9 * k_i) - 1] = 0.0;
    }

    work[k_i] = 0.0;
  }

  for (k_i = 8; k_i >= 0; k_i--) {
    b_lastv = (k_i * 9 + k_i) + 10;
    if (k_i + 1 < 9) {
      b_A[b_lastv - 10] = 1.0;
      if (tau[k_i] != 0.0) {
        knt = 9 - k_i;
        b_coltop = b_lastv - k_i;
        while ((knt > 0) && (b_A[b_coltop - 2] == 0.0)) {
          knt--;
          b_coltop--;
        }

        b_coltop = 8 - k_i;
        exitg2 = false;
        while ((!exitg2) && (b_coltop > 0)) {
          coltop = (b_coltop - 1) * 9 + b_lastv;
          ii = coltop;
          do {
            exitg1 = 0;
            if (ii <= (coltop + knt) - 1) {
              if (b_A[ii - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ii++;
              }
            } else {
              b_coltop--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        knt = 0;
        b_coltop = 0;
      }

      if (knt > 0) {
        xgemv(knt, b_coltop, b_A, b_lastv, b_A, b_lastv - 9, work);
        xgerc(knt, b_coltop, -tau[k_i], b_lastv - 9, work, b_A, b_lastv);
      }

      b_coltop = b_lastv - k_i;
      for (knt = b_lastv - 8; knt < b_coltop; knt++) {
        b_A[knt - 1] *= -tau[k_i];
      }
    }

    b_A[b_lastv - 10] = 1.0 - tau[k_i];
    for (knt = 0; knt < k_i; knt++) {
      b_A[(b_lastv - knt) - 11] = 0.0;
    }
  }

  k_i = 0;
  do {
    exitg1 = 0;
    if (k_i <= nA - 1) {
      if (fabs(R[9 * k_i + k_i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        k_i++;
      }
    } else {
      for (k_i = 0; k_i < n; k_i++) {
        for (ii = 0; ii < n; ii++) {
          RLinv_0 = 0.0;
          for (b_coltop = 0; b_coltop < 9; b_coltop++) {
            RLinv_0 += b_Linv[9 * k_i + b_coltop] * b_A[9 * ii + b_coltop];
          }

          TL[k_i + 9 * ii] = RLinv_0;
        }
      }

      memset(&RLinv[0], 0, 81U * sizeof(real_T));
      for (k_i = nA; k_i >= 1; k_i--) {
        b_coltop = (k_i - 1) * 9;
        knt = (k_i + b_coltop) - 1;
        RLinv[knt] = 1.0;
        for (ii = k_i; ii <= nA; ii++) {
          coltop = ((ii - 1) * 9 + k_i) - 1;
          RLinv[coltop] /= R[knt];
        }

        if (k_i > 1) {
          for (ii = 0; ii <= k_i - 2; ii++) {
            for (b_lastv = k_i; b_lastv <= nA; b_lastv++) {
              knt = (b_lastv - 1) * 9;
              coltop = knt + ii;
              RLinv[coltop] -= RLinv[(knt + k_i) - 1] * R[b_coltop + ii];
            }
          }
        }
      }

      for (k_i = 0; k_i < n; k_i++) {
        for (ii = k_i + 1; ii <= n; ii++) {
          b_coltop = (ii - 1) * 9 + k_i;
          b_H[b_coltop] = 0.0;
          for (b_lastv = nA + 1; b_lastv <= n; b_lastv++) {
            knt = (b_lastv - 1) * 9;
            b_H[b_coltop] -= TL[(knt + ii) - 1] * TL[knt + k_i];
          }

          b_H[(ii + 9 * k_i) - 1] = b_H[b_coltop];
        }
      }

      for (k_i = 0; k_i < nA; k_i++) {
        for (ii = 0; ii < n; ii++) {
          b_coltop = 9 * k_i + ii;
          b_D[b_coltop] = 0.0;
          for (b_lastv = k_i + 1; b_lastv <= nA; b_lastv++) {
            knt = (b_lastv - 1) * 9;
            b_D[b_coltop] += TL[knt + ii] * RLinv[knt + k_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void DropConstraint(int32_T kDrop, boolean_T iA[32], int32_T *nA, int32_T
  iC[32])
{
  int32_T i;
  if (kDrop > 0) {
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      for (i = kDrop; i < *nA; i++) {
        iC[i - 1] = iC[i];
      }
    }

    iC[*nA - 1] = 0;
    (*nA)--;
  }
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void qpkwik(const real_T b_Linv[81], const real_T b_Hinv[81], const
                   real_T f[9], const real_T b_Ac[288], const real_T b[32],
                   boolean_T iA[32], int32_T maxiter, real_T FeasTol, real_T x[9],
                   real_T lambda[32], int32_T *status)
{
  real_T RLinv[81];
  real_T U[81];
  real_T b_D[81];
  real_T b_H[81];
  real_T cTol[32];
  real_T Opt[18];
  real_T Rhs[18];
  real_T r[9];
  real_T z[9];
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T rMin;
  real_T t;
  int32_T iC[32];
  int32_T U_tmp;
  int32_T b_exponent;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exponent;
  int32_T i;
  int32_T iC_0;
  int32_T iSave;
  int32_T nA;
  int32_T tmp;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  memset(&x[0], 0, 9U * sizeof(real_T));
  memset(&lambda[0], 0, sizeof(real_T) << 5U);
  *status = 1;
  memset(&r[0], 0, 9U * sizeof(real_T));
  rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 32; i++) {
    cTol[i] = 1.0;
    iC[i] = 0;
  }

  nA = 0;
  for (tmp = 0; tmp < 32; tmp++) {
    if (iA[tmp]) {
      nA++;
      iC[nA - 1] = tmp + 1;
    }
  }

  guard1 = false;
  if (nA > 0) {
    memset(&Opt[0], 0, 18U * sizeof(real_T));
    for (i = 0; i < 9; i++) {
      Rhs[i] = f[i];
      Rhs[i + 9] = 0.0;
    }

    DualFeasible = false;
    tmp = (int32_T)rt_roundd_snf(0.3 * (real_T)nA);
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (*status <= maxiter)) {
        Xnorm0 = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, b_D, b_H, degrees);
        if (Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            nA = 0;
            memset(&iC[0], 0, sizeof(int32_T) << 5U);
            for (i = 0; i < 32; i++) {
              iA[i] = false;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < nA; i++) {
            Rhs[i + 9] = b[iC[i] - 1];
            for (iSave = i + 1; iSave <= nA; iSave++) {
              U_tmp = (9 * i + iSave) - 1;
              U[U_tmp] = 0.0;
              for (iC_0 = 0; iC_0 < nA; iC_0++) {
                U[U_tmp] += RLinv[(9 * iC_0 + iSave) - 1] * RLinv[9 * iC_0 + i];
              }

              U[i + 9 * (iSave - 1)] = U[U_tmp];
            }
          }

          for (i = 0; i < 9; i++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 9; iC_0++) {
              Xnorm0 += b_H[9 * iC_0 + i] * Rhs[iC_0];
            }

            Opt[i] = Xnorm0;
            for (iSave = 0; iSave < nA; iSave++) {
              Opt[i] += b_D[9 * iSave + i] * Rhs[iSave + 9];
            }
          }

          for (i = 0; i < nA; i++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 9; iC_0++) {
              Xnorm0 += b_D[9 * i + iC_0] * Rhs[iC_0];
            }

            Opt[i + 9] = Xnorm0;
            for (iSave = 0; iSave < nA; iSave++) {
              Opt[i + 9] += U[9 * iSave + i] * Rhs[iSave + 9];
            }
          }

          Xnorm0 = -1.0E-12;
          i = -1;
          for (iSave = 0; iSave < nA; iSave++) {
            cMin = Opt[iSave + 9];
            lambda[iC[iSave] - 1] = cMin;
            if ((cMin < Xnorm0) && (iSave + 1 <= nA)) {
              i = iSave;
              Xnorm0 = cMin;
            }
          }

          if (i + 1 <= 0) {
            DualFeasible = true;
            memcpy(&x[0], &Opt[0], 9U * sizeof(real_T));
          } else {
            (*status)++;
            if (tmp <= 5) {
              iC_0 = 5;
            } else {
              iC_0 = tmp;
            }

            if (*status > iC_0) {
              nA = 0;
              memset(&iC[0], 0, sizeof(int32_T) << 5U);
              for (i = 0; i < 32; i++) {
                iA[i] = false;
              }

              ColdReset = true;
            } else {
              lambda[iC[i] - 1] = 0.0;
              DropConstraint(i + 1, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lambda[0], 0, sizeof(real_T) << 5U);
          for (tmp = 0; tmp < 9; tmp++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 9; iC_0++) {
              Xnorm0 += -b_Hinv[9 * iC_0 + tmp] * f[iC_0];
            }

            x[tmp] = Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (tmp = 0; tmp < 9; tmp++) {
      Xnorm0 = 0.0;
      for (iC_0 = 0; iC_0 < 9; iC_0++) {
        Xnorm0 += -b_Hinv[9 * iC_0 + tmp] * f[iC_0];
      }

      x[tmp] = Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    Xnorm0 = norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      cMin = -FeasTol;
      tmp = -1;
      for (i = 0; i < 32; i++) {
        if (!cTolComputed) {
          for (iSave = 0; iSave < 9; iSave++) {
            z[iSave] = fabs(b_Ac[(iSave << 5) + i] * x[iSave]);
          }

          cTol[i] = fmax(cTol[i], maximum(z));
        }

        if (!iA[i]) {
          t = 0.0;
          for (iC_0 = 0; iC_0 < 9; iC_0++) {
            t += b_Ac[(iC_0 << 5) + i] * x[iC_0];
          }

          cVal = (t - b[i]) / cTol[i];
          if (cVal < cMin) {
            cMin = cVal;
            tmp = i;
          }
        }
      }

      cTolComputed = true;
      if (tmp + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((tmp + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (nA == 0) {
              for (iC_0 = 0; iC_0 < 9; iC_0++) {
                cMin = 0.0;
                for (i = 0; i < 9; i++) {
                  cMin += b_Hinv[9 * i + iC_0] * b_Ac[(i << 5) + tmp];
                }

                z[iC_0] = cMin;
              }

              guard2 = true;
            } else {
              cMin = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, b_D, b_H, degrees);
              if (cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (iC_0 = 0; iC_0 < 81; iC_0++) {
                  U[iC_0] = -b_H[iC_0];
                }

                for (iC_0 = 0; iC_0 < 9; iC_0++) {
                  cMin = 0.0;
                  for (i = 0; i < 9; i++) {
                    cMin += U[9 * i + iC_0] * b_Ac[(i << 5) + tmp];
                  }

                  z[iC_0] = cMin;
                }

                for (i = 0; i < nA; i++) {
                  t = 0.0;
                  for (iC_0 = 0; iC_0 < 9; iC_0++) {
                    t += b_Ac[(iC_0 << 5) + tmp] * b_D[9 * i + iC_0];
                  }

                  r[i] = t;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              i = 0;
              cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (nA > 0) {
                iSave = 0;
                exitg4 = false;
                while ((!exitg4) && (iSave <= nA - 1)) {
                  if (r[iSave] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    iSave++;
                  }
                }
              }

              if ((nA != 0) && (!ColdReset)) {
                for (iSave = 0; iSave < nA; iSave++) {
                  cVal = r[iSave];
                  if (cVal > 1.0E-12) {
                    cVal = lambda[iC[iSave] - 1] / cVal;
                    if ((i == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      i = iSave + 1;
                    }
                  }
                }

                if (i > 0) {
                  cMin = rMin;
                  DualFeasible = false;
                }
              }

              cVal = 0.0;
              for (iSave = 0; iSave < 9; iSave++) {
                cVal += b_Ac[(iSave << 5) + tmp] * z[iSave];
              }

              if (cVal <= 0.0) {
                cVal = 0.0;
                ColdReset = true;
              } else {
                t = 0.0;
                for (iC_0 = 0; iC_0 < 9; iC_0++) {
                  t += b_Ac[(iC_0 << 5) + tmp] * x[iC_0];
                }

                cVal = (b[tmp] - t) / cVal;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  t = cMin;
                } else if (DualFeasible) {
                  t = cVal;
                } else if (cMin < cVal) {
                  t = cMin;
                } else {
                  t = cVal;
                }

                for (iSave = 0; iSave < nA; iSave++) {
                  iC_0 = iC[iSave];
                  lambda[iC_0 - 1] -= t * r[iSave];
                  if ((iC_0 <= 32) && (lambda[iC_0 - 1] < 0.0)) {
                    lambda[iC_0 - 1] = 0.0;
                  }
                }

                lambda[tmp] += t;
                frexp(1.0, &exponent);
                if (fabs(t - cMin) < 2.2204460492503131E-16) {
                  DropConstraint(i, iA, &nA, iC);
                }

                if (!ColdReset) {
                  for (iC_0 = 0; iC_0 < 9; iC_0++) {
                    x[iC_0] += t * z[iC_0];
                  }

                  frexp(1.0, &b_exponent);
                  if (fabs(t - cVal) < 2.2204460492503131E-16) {
                    if (nA == degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      nA++;
                      iC[nA - 1] = tmp + 1;
                      i = nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (i + 1 > 1)) {
                        iC_0 = iC[i - 1];
                        if (iC[i] > iC_0) {
                          exitg4 = true;
                        } else {
                          iSave = iC[i];
                          iC[i] = iC_0;
                          iC[i - 1] = iSave;
                          i--;
                        }
                      }

                      iA[tmp] = true;
                      tmp = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cMin = norm(x);
            if (fabs(cMin - Xnorm0) > 0.001) {
              Xnorm0 = cMin;
              for (tmp = 0; tmp < 32; tmp++) {
                cTol[tmp] = fmax(fabs(b[tmp]), 1.0);
              }

              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void mpcblock_optimizer(const real_T rseq[45], const real_T vseq[16],
  const real_T x[3], const real_T old_u[4], const boolean_T iA[32], const real_T
  b_Mlim[32], real_T b_Mx[96], real_T b_Mu1[128], real_T b_Mv[512], const real_T
  b_utarget[60], const real_T b_uoff[4], real_T b_H[81], real_T b_Ac[288], const
  real_T b_Wy[3], const real_T b_Wdu[4], const real_T b_Jm[480], const real_T
  b_Wu[4], const real_T b_I1[240], const real_T b_A[144], const real_T Bu[192],
  const real_T Bv[48], const real_T b_C[144], const real_T Dv[48], const int32_T
  b_Mrows[32], real_T u[4], real_T useq[64], real_T *status, boolean_T iAout[32])
{
  real_T I2Jm[480];
  real_T WduJm[480];
  real_T WuI2Jm[480];
  real_T WySuJm[360];
  real_T c_SuJm[360];
  real_T c_Su1[180];
  real_T c_Sx[135];
  real_T c_Kv[128];
  real_T AA[126];
  real_T L[81];
  real_T b_I1_0[32];
  real_T c_Ku1[32];
  real_T c_Su1_0[32];
  real_T c_Kx[24];
  real_T b_C_1[12];
  real_T AA_0[9];
  real_T b_C_0[9];
  real_T Bv_0;
  real_T normH;
  real_T old_u_0;
  real_T old_u_1;
  real_T old_u_2;
  real_T old_u_3;
  real_T s;
  int32_T AA_tmp;
  int32_T Tries;
  int32_T b_a_tmp;
  int32_T c_Hv_tmp;
  int32_T i;
  int32_T kidx;
  int32_T kidx_tmp;
  int16_T ixw;
  int8_T a[3600];
  int8_T b[81];
  int8_T b_B_0[16];
  int8_T rows[3];
  int8_T b_B_1;
  int8_T c_A;
  int8_T rows_0;
  int8_T rows_1;
  int8_T rows_2;
  static const int8_T c_A_0[225] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1 };

  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  memset(&useq[0], 0, sizeof(real_T) << 6U);
  for (i = 0; i < 32; i++) {
    iAout[i] = false;
  }

  normH = Bv[0];
  s = Bv[1];
  Bv_0 = Bv[2];
  for (i = 0; i < 3; i++) {
    rtDW.c_Hv[i] = (b_C[i + 9] * normH + b_C[i + 12] * s) + b_C[i + 15] * Bv_0;
    rtDW.c_Hv[i + 45] = Dv[i + 3];
  }

  for (i = 0; i < 14; i++) {
    c_Hv_tmp = (i + 2) * 45;
    rtDW.c_Hv[c_Hv_tmp] = 0.0;
    rtDW.c_Hv[c_Hv_tmp + 1] = 0.0;
    rtDW.c_Hv[c_Hv_tmp + 2] = 0.0;
  }

  for (i = 0; i < 16; i++) {
    memset(&rtDW.c_Hv[i * 45 + 3], 0, 42U * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    normH = b_A[3 * i + 1];
    s = b_A[3 * i];
    Bv_0 = b_A[3 * i + 2];
    for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
      b_C_0[b_a_tmp + 3 * i] = (b_C[b_a_tmp + 12] * normH + b_C[b_a_tmp + 9] * s)
        + b_C[b_a_tmp + 15] * Bv_0;
    }
  }

  for (i = 0; i < 3; i++) {
    c_Sx[45 * i] = b_C_0[3 * i];
    c_Sx[45 * i + 1] = b_C_0[3 * i + 1];
    c_Sx[45 * i + 2] = b_C_0[3 * i + 2];
  }

  for (i = 0; i < 42; i++) {
    c_Sx[i + 3] = 0.0;
    c_Sx[i + 48] = 0.0;
    c_Sx[i + 93] = 0.0;
  }

  for (i = 0; i < 4; i++) {
    normH = Bu[3 * i + 1];
    s = Bu[3 * i];
    Bv_0 = Bu[3 * i + 2];
    for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
      b_C_1[b_a_tmp + 3 * i] = (b_C[b_a_tmp + 12] * normH + b_C[b_a_tmp + 9] * s)
        + b_C[b_a_tmp + 15] * Bv_0;
    }
  }

  for (i = 0; i < 4; i++) {
    c_Su1[45 * i] = b_C_1[3 * i];
    c_Su1[45 * i + 1] = b_C_1[3 * i + 1];
    c_Su1[45 * i + 2] = b_C_1[3 * i + 2];
  }

  for (i = 0; i < 42; i++) {
    c_Su1[i + 3] = 0.0;
    c_Su1[i + 48] = 0.0;
    c_Su1[i + 93] = 0.0;
    c_Su1[i + 138] = 0.0;
  }

  for (i = 0; i < 4; i++) {
    rtDW.b_a[45 * i] = c_Su1[45 * i];
    b_a_tmp = 45 * i + 1;
    rtDW.b_a[b_a_tmp] = c_Su1[b_a_tmp];
    b_a_tmp = 45 * i + 2;
    rtDW.b_a[b_a_tmp] = c_Su1[b_a_tmp];
  }

  for (i = 0; i < 56; i++) {
    b_a_tmp = (i + 4) * 45;
    rtDW.b_a[b_a_tmp] = 0.0;
    rtDW.b_a[b_a_tmp + 1] = 0.0;
    rtDW.b_a[b_a_tmp + 2] = 0.0;
  }

  for (i = 0; i < 60; i++) {
    memset(&rtDW.b_a[i * 45 + 3], 0, 42U * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    AA[3 * i] = b_A[3 * i + 9];
    AA[3 * i + 1] = b_A[3 * i + 10];
    AA[3 * i + 2] = b_A[3 * i + 11];
  }

  for (i = 0; i < 39; i++) {
    AA_tmp = (i + 3) * 3;
    AA[AA_tmp] = 0.0;
    AA[AA_tmp + 1] = 0.0;
    AA[AA_tmp + 2] = 0.0;
  }

  for (kidx = 0; kidx < 14; kidx++) {
    kidx_tmp = (kidx + 1) * 3;
    for (i = 0; i < 3; i++) {
      rows_2 = (int8_T)((int8_T)(kidx_tmp + 1) + i);
      rows[i] = rows_2;
      b_a_tmp = (kidx + 2) * 9 + i;
      normH = b_C[b_a_tmp];
      s = b_C[b_a_tmp + 3];
      Bv_0 = b_C[b_a_tmp + 6];
      for (b_a_tmp = 0; b_a_tmp < 4; b_a_tmp++) {
        AA_tmp = (45 * b_a_tmp + rows_2) - 1;
        c_Su1[AA_tmp] = 0.0;
        Tries = (kidx + 1) * 12 + 3 * b_a_tmp;
        c_Su1[AA_tmp] += normH * Bu[Tries];
        c_Su1[AA_tmp] += Bu[Tries + 1] * s;
        c_Su1[AA_tmp] += Bu[Tries + 2] * Bv_0;
      }
    }

    Tries = (kidx + 1) << 2;
    rows_2 = rows[0];
    rows_0 = rows[1];
    rows_1 = rows[2];
    for (i = 0; i < 4; i++) {
      b_a_tmp = (i + Tries) * 45;
      rtDW.b_a[(rows_2 + b_a_tmp) - 1] = c_Su1[(45 * i + rows_2) - 1];
      rtDW.b_a[(rows_0 + b_a_tmp) - 1] = c_Su1[(45 * i + rows_0) - 1];
      rtDW.b_a[(rows_1 + b_a_tmp) - 1] = c_Su1[(45 * i + rows_1) - 1];
    }

    for (Tries = 0; Tries <= kidx; Tries++) {
      c_Hv_tmp = kidx - Tries;
      b_a_tmp = c_Hv_tmp * 3;
      for (i = 0; i < 3; i++) {
        AA_tmp = (i + b_a_tmp) * 3;
        AA_0[3 * i] = AA[AA_tmp];
        AA_0[3 * i + 1] = AA[AA_tmp + 1];
        AA_0[3 * i + 2] = AA[AA_tmp + 2];
      }

      for (i = 0; i < 3; i++) {
        b_a_tmp = (kidx + 2) * 9 + i;
        normH = b_C[b_a_tmp + 3];
        s = b_C[b_a_tmp];
        Bv_0 = b_C[b_a_tmp + 6];
        for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
          b_C_0[i + 3 * b_a_tmp] = (AA_0[3 * b_a_tmp + 1] * normH + AA_0[3 *
            b_a_tmp] * s) + AA_0[3 * b_a_tmp + 2] * Bv_0;
        }

        normH = b_C_0[i + 3];
        s = b_C_0[i];
        Bv_0 = b_C_0[i + 6];
        for (b_a_tmp = 0; b_a_tmp < 4; b_a_tmp++) {
          AA_tmp = 3 * b_a_tmp + 12 * c_Hv_tmp;
          b_C_1[i + 3 * b_a_tmp] = ((Bu[AA_tmp + 1] * normH + Bu[AA_tmp] * s) +
            Bu[AA_tmp + 2] * Bv_0) + c_Su1[(45 * b_a_tmp + rows[i]) - 1];
        }
      }

      rows_2 = rows[0];
      rows_0 = rows[1];
      rows_1 = rows[2];
      for (i = 0; i < 4; i++) {
        c_Su1[(rows_2 + 45 * i) - 1] = b_C_1[3 * i];
        c_Su1[(rows_0 + 45 * i) - 1] = b_C_1[3 * i + 1];
        c_Su1[(rows_1 + 45 * i) - 1] = b_C_1[3 * i + 2];
      }

      c_Hv_tmp <<= 2;
      for (i = 0; i < 4; i++) {
        b_a_tmp = (i + c_Hv_tmp) * 45;
        rtDW.b_a[(rows_2 + b_a_tmp) - 1] = c_Su1[(45 * i + rows_2) - 1];
        rtDW.b_a[(rows_0 + b_a_tmp) - 1] = c_Su1[(45 * i + rows_0) - 1];
        rtDW.b_a[(rows_1 + b_a_tmp) - 1] = c_Su1[(45 * i + rows_1) - 1];
      }
    }

    c_Hv_tmp = (kidx + 2) * 3;
    Tries = (kidx + 2) * 45;
    rtDW.c_Hv[(rows[0] + Tries) - 1] = Dv[c_Hv_tmp];
    rtDW.c_Hv[(rows[1] + Tries) - 1] = Dv[c_Hv_tmp + 1];
    rtDW.c_Hv[(rows[2] + Tries) - 1] = Dv[c_Hv_tmp + 2];
    normH = Bv[kidx_tmp];
    s = Bv[kidx_tmp + 1];
    Bv_0 = Bv[kidx_tmp + 2];
    for (i = 0; i < 3; i++) {
      c_Hv_tmp = ((kidx + 1) * 45 + rows[i]) - 1;
      rtDW.c_Hv[c_Hv_tmp] = 0.0;
      Tries = (kidx + 2) * 9 + i;
      rtDW.c_Hv[c_Hv_tmp] += b_C[Tries] * normH;
      rtDW.c_Hv[c_Hv_tmp] += b_C[Tries + 3] * s;
      rtDW.c_Hv[c_Hv_tmp] += b_C[Tries + 6] * Bv_0;
    }

    for (Tries = 0; Tries <= kidx; Tries++) {
      b_a_tmp = Tries * 3;
      for (i = 0; i < 3; i++) {
        AA_tmp = (i + b_a_tmp) * 3;
        AA_0[3 * i] = AA[AA_tmp];
        AA_0[3 * i + 1] = AA[AA_tmp + 1];
        AA_0[3 * i + 2] = AA[AA_tmp + 2];
      }

      for (i = 0; i < 3; i++) {
        c_Hv_tmp = (45 * Tries + rows[i]) - 1;
        rtDW.c_Hv[c_Hv_tmp] = 0.0;
        b_a_tmp = (kidx + 2) * 9 + i;
        normH = b_C[b_a_tmp + 3];
        s = b_C[b_a_tmp];
        Bv_0 = b_C[b_a_tmp + 6];
        for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
          rtDW.c_Hv[c_Hv_tmp] += ((AA_0[3 * b_a_tmp + 1] * normH + AA_0[3 *
            b_a_tmp] * s) + AA_0[3 * b_a_tmp + 2] * Bv_0) * Bv[3 * Tries +
            b_a_tmp];
        }
      }
    }

    for (i = 0; i < 3; i++) {
      b_a_tmp = (kidx + 2) * 9 + i;
      normH = b_C[b_a_tmp + 3];
      s = b_C[b_a_tmp];
      Bv_0 = b_C[b_a_tmp + 6];
      for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
        b_C_0[i + 3 * b_a_tmp] = (AA[3 * b_a_tmp + 1] * normH + AA[3 * b_a_tmp] *
          s) + AA[3 * b_a_tmp + 2] * Bv_0;
      }

      rows_2 = rows[i];
      normH = b_C_0[i];
      s = b_C_0[i + 3];
      Bv_0 = b_C_0[i + 6];
      for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
        Tries = (45 * b_a_tmp + rows_2) - 1;
        c_Sx[Tries] = 0.0;
        c_Sx[Tries] += b_A[3 * b_a_tmp] * normH;
        c_Sx[Tries] += b_A[3 * b_a_tmp + 1] * s;
        c_Sx[Tries] += b_A[3 * b_a_tmp + 2] * Bv_0;
      }
    }

    if (kidx + 2 < 15) {
      for (Tries = 0; Tries <= kidx; Tries++) {
        c_Hv_tmp = Tries * 3;
        for (i = 0; i < 3; i++) {
          AA_tmp = (i + c_Hv_tmp) * 3;
          AA_0[3 * i] = AA[AA_tmp];
          AA_0[3 * i + 1] = AA[AA_tmp + 1];
          AA_0[3 * i + 2] = AA[AA_tmp + 2];
        }

        for (i = 0; i < 3; i++) {
          b_a_tmp = (kidx + 2) * 9 + i;
          normH = b_A[b_a_tmp];
          s = b_A[b_a_tmp + 3];
          Bv_0 = b_A[b_a_tmp + 6];
          for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
            AA_tmp = (b_a_tmp + c_Hv_tmp) * 3 + i;
            AA[AA_tmp] = 0.0;
            AA[AA_tmp] += AA_0[3 * b_a_tmp] * normH;
            AA[AA_tmp] += AA_0[3 * b_a_tmp + 1] * s;
            AA[AA_tmp] += AA_0[3 * b_a_tmp + 2] * Bv_0;
          }
        }
      }

      for (i = 0; i < 3; i++) {
        AA_tmp = (kidx + 2) * 9 + 3 * i;
        Tries = (i + kidx_tmp) * 3;
        AA[Tries] = b_A[AA_tmp];
        AA[Tries + 1] = b_A[AA_tmp + 1];
        AA[Tries + 2] = b_A[AA_tmp + 2];
      }
    }
  }

  for (i = 0; i < 8; i++) {
    for (b_a_tmp = 0; b_a_tmp < 45; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 60; kidx++) {
        normH += rtDW.b_a[45 * kidx + b_a_tmp] * b_Jm[60 * i + kidx];
      }

      c_SuJm[b_a_tmp + 45 * i] = normH;
    }
  }

  if (b_Mrows[0] > 0) {
    kidx = 0;
    exitg1 = false;
    while ((!exitg1) && (kidx < 32)) {
      if (b_Mrows[kidx] <= 45) {
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 8; i++) {
          b_Ac[kidx + (i << 5)] = -c_SuJm[(45 * i + kidx_tmp) - 1];
        }

        kidx_tmp = b_Mrows[kidx];
        b_Mx[kidx] = -c_Sx[kidx_tmp - 1];
        b_Mx[kidx + 32] = -c_Sx[kidx_tmp + 44];
        b_Mx[kidx + 64] = -c_Sx[kidx_tmp + 89];
        kidx_tmp = b_Mrows[kidx];
        b_Mu1[kidx] = -c_Su1[kidx_tmp - 1];
        b_Mu1[kidx + 32] = -c_Su1[kidx_tmp + 44];
        b_Mu1[kidx + 64] = -c_Su1[kidx_tmp + 89];
        b_Mu1[kidx + 96] = -c_Su1[kidx_tmp + 134];
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 16; i++) {
          b_Mv[kidx + (i << 5)] = -rtDW.c_Hv[(45 * i + kidx_tmp) - 1];
        }

        kidx++;
      } else if (b_Mrows[kidx] <= 90) {
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 8; i++) {
          b_Ac[kidx + (i << 5)] = c_SuJm[(45 * i + kidx_tmp) - 46];
        }

        kidx_tmp = b_Mrows[kidx];
        b_Mx[kidx] = c_Sx[kidx_tmp - 46];
        b_Mx[kidx + 32] = c_Sx[kidx_tmp - 1];
        b_Mx[kidx + 64] = c_Sx[kidx_tmp + 44];
        kidx_tmp = b_Mrows[kidx];
        b_Mu1[kidx] = c_Su1[kidx_tmp - 46];
        b_Mu1[kidx + 32] = c_Su1[kidx_tmp - 1];
        b_Mu1[kidx + 64] = c_Su1[kidx_tmp + 44];
        b_Mu1[kidx + 96] = c_Su1[kidx_tmp + 89];
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 16; i++) {
          b_Mv[kidx + (i << 5)] = rtDW.c_Hv[(45 * i + kidx_tmp) - 46];
        }

        kidx++;
      } else {
        exitg1 = true;
      }
    }
  }

  for (i = 0; i < 16; i++) {
    b_B_0[i] = 0;
  }

  b_B_0[0] = 1;
  b_B_0[5] = 1;
  b_B_0[10] = 1;
  b_B_0[15] = 1;
  kidx = -1;
  for (Tries = 0; Tries < 15; Tries++) {
    for (c_Hv_tmp = 0; c_Hv_tmp < 4; c_Hv_tmp++) {
      i = c_Hv_tmp << 2;
      rows_2 = b_B_0[i];
      rows_0 = b_B_0[i + 1];
      rows_1 = b_B_0[i + 2];
      b_B_1 = b_B_0[i + 3];
      for (i = 0; i < 15; i++) {
        c_A = c_A_0[15 * Tries + i];
        a[kidx + 1] = (int8_T)(c_A * rows_2);
        a[kidx + 2] = (int8_T)(c_A * rows_0);
        a[kidx + 3] = (int8_T)(c_A * rows_1);
        a[kidx + 4] = (int8_T)(c_A * b_B_1);
        kidx += 4;
      }
    }
  }

  for (i = 0; i < 8; i++) {
    for (b_a_tmp = 0; b_a_tmp < 60; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 60; kidx++) {
        normH += (real_T)a[60 * kidx + b_a_tmp] * b_Jm[60 * i + kidx];
      }

      I2Jm[b_a_tmp + 60 * i] = normH;
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 45; kidx++) {
    normH = b_Wy[ixw - 1];
    for (i = 0; i < 8; i++) {
      kidx_tmp = 45 * i + kidx;
      WySuJm[kidx_tmp] = c_SuJm[kidx_tmp] * normH;
    }

    ixw++;
    if (ixw > 3) {
      ixw = 1;
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 60; kidx++) {
    normH = b_Wu[ixw - 1];
    for (i = 0; i < 8; i++) {
      kidx_tmp = 60 * i + kidx;
      WuI2Jm[kidx_tmp] = I2Jm[kidx_tmp] * normH;
    }

    ixw++;
    if (ixw > 4) {
      ixw = 1;
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 60; kidx++) {
    normH = b_Wdu[ixw - 1];
    for (i = 0; i < 8; i++) {
      kidx_tmp = 60 * i + kidx;
      WduJm[kidx_tmp] = b_Jm[kidx_tmp] * normH;
    }

    ixw++;
    if (ixw > 4) {
      ixw = 1;
    }
  }

  for (i = 0; i < 8; i++) {
    for (b_a_tmp = 0; b_a_tmp < 8; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 45; kidx++) {
        normH += c_SuJm[45 * i + kidx] * WySuJm[45 * b_a_tmp + kidx];
      }

      s = 0.0;
      Bv_0 = 0.0;
      for (kidx = 0; kidx < 60; kidx++) {
        kidx_tmp = 60 * i + kidx;
        Tries = 60 * b_a_tmp + kidx;
        Bv_0 += b_Jm[kidx_tmp] * WduJm[Tries];
        s += I2Jm[kidx_tmp] * WuI2Jm[Tries];
      }

      b_H[i + 9 * b_a_tmp] = (normH + Bv_0) + s;
    }
  }

  for (i = 0; i < 4; i++) {
    for (b_a_tmp = 0; b_a_tmp < 8; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 45; kidx++) {
        normH += c_Su1[45 * i + kidx] * WySuJm[45 * b_a_tmp + kidx];
      }

      AA_tmp = (b_a_tmp << 2) + i;
      c_Su1_0[AA_tmp] = normH;
      normH = 0.0;
      for (kidx = 0; kidx < 60; kidx++) {
        normH += b_I1[60 * i + kidx] * WuI2Jm[60 * b_a_tmp + kidx];
      }

      b_I1_0[AA_tmp] = normH;
    }
  }

  for (i = 0; i < 32; i++) {
    c_Ku1[i] = c_Su1_0[i] + b_I1_0[i];
  }

  for (i = 0; i < 480; i++) {
    WuI2Jm[i] = -WuI2Jm[i];
  }

  for (i = 0; i < 3; i++) {
    for (b_a_tmp = 0; b_a_tmp < 8; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 45; kidx++) {
        normH += c_Sx[45 * i + kidx] * WySuJm[45 * b_a_tmp + kidx];
      }

      c_Kx[i + 3 * b_a_tmp] = normH;
    }
  }

  for (i = 0; i < 16; i++) {
    for (b_a_tmp = 0; b_a_tmp < 8; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 45; kidx++) {
        normH += rtDW.c_Hv[45 * i + kidx] * WySuJm[45 * b_a_tmp + kidx];
      }

      c_Kv[i + (b_a_tmp << 4)] = normH;
    }
  }

  for (i = 0; i < 360; i++) {
    WySuJm[i] = -WySuJm[i];
  }

  kidx = 0;
  memcpy(&L[0], &b_H[0], 81U * sizeof(real_T));
  Tries = xpotrf(L);
  guard1 = false;
  if (Tries == 0) {
    for (Tries = 0; Tries < 9; Tries++) {
      b_C_0[Tries] = L[9 * Tries + Tries];
    }

    if (minimum(b_C_0) > 1.4901161193847656E-7) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    normH = 0.0;
    Tries = 0;
    exitg2 = false;
    while ((!exitg2) && (Tries < 9)) {
      s = 0.0;
      for (c_Hv_tmp = 0; c_Hv_tmp < 9; c_Hv_tmp++) {
        s += fabs(b_H[9 * c_Hv_tmp + Tries]);
      }

      if (rtIsNaN(s)) {
        normH = (rtNaN);
        exitg2 = true;
      } else {
        if (s > normH) {
          normH = s;
        }

        Tries++;
      }
    }

    if (normH >= 1.0E+10) {
      kidx = 2;
    } else {
      Tries = 0;
      exitg1 = false;
      while ((!exitg1) && (Tries <= 4)) {
        normH = rt_powd_snf(10.0, (real_T)Tries) * 1.4901161193847656E-7;
        memset(&b[0], 0, 81U * sizeof(int8_T));
        for (kidx = 0; kidx < 9; kidx++) {
          b[kidx + 9 * kidx] = 1;
        }

        for (i = 0; i < 81; i++) {
          s = normH * (real_T)b[i] + b_H[i];
          b_H[i] = s;
          L[i] = s;
        }

        kidx = xpotrf(L);
        guard2 = false;
        if (kidx == 0) {
          for (kidx = 0; kidx < 9; kidx++) {
            b_C_0[kidx] = L[9 * kidx + kidx];
          }

          if (minimum(b_C_0) > 1.4901161193847656E-7) {
            kidx = 1;
            exitg1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          kidx = 3;
          Tries++;
        }
      }
    }
  }

  if (kidx > 1) {
    u[0] = old_u[0] + b_uoff[0];
    u[1] = old_u[1] + b_uoff[1];
    u[2] = old_u[2] + b_uoff[2];
    u[3] = old_u[3] + b_uoff[3];
    normH = u[0];
    s = u[1];
    Bv_0 = u[2];
    old_u_0 = u[3];
    for (i = 0; i < 16; i++) {
      useq[i] = normH;
      useq[i + 16] = s;
      useq[i + 32] = Bv_0;
      useq[i + 48] = old_u_0;
    }

    *status = -2.0;
  } else {
    memset(&b[0], 0, 81U * sizeof(int8_T));
    for (kidx = 0; kidx < 9; kidx++) {
      b[kidx + 9 * kidx] = 1;
    }

    for (kidx = 0; kidx < 9; kidx++) {
      for (Tries = 0; Tries < 9; Tries++) {
        i = 9 * kidx + Tries;
        b_H[i] = b[i];
      }

      b_C_0[kidx] = 0.0;
    }

    trisolve(L, b_H);
    for (kidx = 0; kidx < 8; kidx++) {
      s = 0.0;
      for (i = 0; i < 45; i++) {
        s += WySuJm[45 * kidx + i] * rseq[i];
      }

      normH = 0.0;
      for (i = 0; i < 16; i++) {
        normH += c_Kv[(kidx << 4) + i] * vseq[i];
      }

      Bv_0 = 0.0;
      for (i = 0; i < 60; i++) {
        Bv_0 += WuI2Jm[60 * kidx + i] * b_utarget[i];
      }

      i = kidx << 2;
      b_C_0[kidx] = (((((c_Kx[3 * kidx + 1] * x[1] + c_Kx[3 * kidx] * x[0]) +
                        c_Kx[3 * kidx + 2] * x[2]) + s) + (((c_Ku1[i + 1] *
        old_u[1] + c_Ku1[i] * old_u[0]) + c_Ku1[i + 2] * old_u[2]) + c_Ku1[i + 3]
        * old_u[3])) + normH) + Bv_0;
    }

    normH = x[1];
    s = x[0];
    Bv_0 = x[2];
    old_u_0 = old_u[1];
    old_u_1 = old_u[0];
    old_u_2 = old_u[2];
    old_u_3 = old_u[3];
    for (i = 0; i < 32; i++) {
      iAout[i] = iA[i];
      c_Ku1[i] = ((b_Mx[i + 32] * normH + b_Mx[i] * s) + b_Mx[i + 64] * Bv_0) +
        b_Mlim[i];
      b_I1_0[i] = ((b_Mu1[i + 32] * old_u_0 + b_Mu1[i] * old_u_1) + b_Mu1[i + 64]
                   * old_u_2) + b_Mu1[i + 96] * old_u_3;
    }

    for (i = 0; i < 9; i++) {
      for (b_a_tmp = 0; b_a_tmp < 9; b_a_tmp++) {
        s = 0.0;
        for (kidx = 0; kidx < 9; kidx++) {
          s += b_H[9 * i + kidx] * b_H[9 * b_a_tmp + kidx];
        }

        L[i + 9 * b_a_tmp] = s;
      }
    }

    for (i = 0; i < 32; i++) {
      s = 0.0;
      for (b_a_tmp = 0; b_a_tmp < 16; b_a_tmp++) {
        s += b_Mv[(b_a_tmp << 5) + i] * vseq[b_a_tmp];
      }

      c_Su1_0[i] = -((c_Ku1[i] + b_I1_0[i]) + s);
    }

    qpkwik(b_H, L, b_C_0, b_Ac, c_Su1_0, iAout, 164, 1.0E-6, AA_0, c_Ku1, &kidx);
    if ((kidx < 0) || (kidx == 0)) {
      memset(&AA_0[0], 0, 9U * sizeof(real_T));
    }

    *status = kidx;
    u[0] = (old_u[0] + AA_0[0]) + b_uoff[0];
    u[1] = (old_u[1] + AA_0[1]) + b_uoff[1];
    u[2] = (old_u[2] + AA_0[2]) + b_uoff[2];
    u[3] = (old_u[3] + AA_0[3]) + b_uoff[3];
  }
}

/* Model step function */
void modelv2_step(void)
{
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  {
    real_T Bu[192];
    real_T Cm[144];
    real_T b_A[144];
    real_T b_utarget[60];
    real_T Bv[48];
    real_T Dv[48];
    real_T Dvm[48];
    real_T rseq[45];
    real_T b_Mlim[32];
    real_T vseq[16];
    real_T Bd[12];
    real_T L[9];
    real_T Nk[9];
    real_T Qk[9];
    real_T Rk[9];
    real_T c_A[9];
    real_T a21;
    real_T maxval;
    int32_T r1;
    int32_T r2;
    int32_T r3;
    int8_T Ad[9];
    int8_T b_I[9];
    static const real_T b_B_5[384] = { 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
      -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
      0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
      -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    real_T tmp_3[512];
    real_T B_o[192];
    real_T A[144];
    real_T C[144];
    real_T b_C_tmp[144];
    real_T tmp_2[96];
    real_T U[64];
    real_T Cm_0[9];
    real_T c_A_tmp[9];
    real_T rtb_LastPcov[9];
    real_T rtb_LastPcov_0[9];
    real_T U_tmp[4];
    real_T rtb_last_mv_0[4];
    real_T tmp_1[4];
    real_T tmp_4[4];
    real_T ref[3];
    real_T ref_0[3];
    real_T rtb_last_x[3];
    real_T Rk_tmp_0;
    real_T Rk_tmp_1;
    real_T Rk_tmp_2;
    real_T b_b;
    real_T c_tmp;
    real_T ref_1;
    real_T ref_2;
    real_T ref_3;
    real_T s_tmp;
    real_T tmp_5;
    int32_T tmp[4];
    int32_T tmp_0[3];
    int32_T Bv_tmp;
    int32_T Rk_tmp;
    int32_T i;
    boolean_T rtb_Memory[32];
    static const real_T b_b_0[12] = { 0.012125, -0.012125, -0.034642857142857142,
      0.012125, 0.012125, 0.034642857142857142, 0.012125, -0.012125,
      0.034642857142857142, 0.012125, 0.012125, -0.034642857142857142 };

    static const int8_T b[144] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0,
      0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0,
      1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0,
      0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1,
      1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0,
      0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1,
      0, 0, 0, 1 };

    static const int8_T c[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    static const real_T d[24] = { 0.0012125000000000003, -0.0012125000000000003,
      -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
      0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
      -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
      0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0 };

    static const real_T e[24] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
    };

    static const int8_T f[32] = { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
      12, 12, 12, 12, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };

    static const int8_T b_D[384] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    static const real_T q[128] = { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
      -0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -1.0, -0.0, -0.0,
      -0.0, -1.0, -0.0, -0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
      -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    static const real_T r[81] = { 1076.128352040816, -930.09885204081615,
      930.09885204081615, -784.259352040816, 971.95528571428542,
      -840.08928571428555, 840.08928571428544, -708.36328571428544, 0.0,
      -930.09885204081615, 1076.128352040816, -784.259352040816,
      930.09885204081615, -840.08928571428555, 971.95528571428542,
      -708.36328571428544, 840.08928571428544, 0.0, 930.09885204081615,
      -784.259352040816, 1076.128352040816, -930.09885204081615,
      840.08928571428544, -708.36328571428544, 971.95528571428542,
      -840.08928571428555, 0.0, -784.259352040816, 930.09885204081615,
      -930.09885204081615, 1076.128352040816, -708.36328571428544,
      840.08928571428544, -840.08928571428555, 971.95528571428542, 0.0,
      971.95528571428542, -840.08928571428555, 840.08928571428544,
      -708.36328571428544, 880.88760267857117, -761.33091517857122,
      761.33091517857122, -641.95422767857121, 0.0, -840.08928571428555,
      971.95528571428542, -708.36328571428544, 840.08928571428544,
      -761.33091517857122, 880.88760267857117, -641.95422767857121,
      761.33091517857122, 0.0, 840.08928571428544, -708.36328571428544,
      971.95528571428542, -840.08928571428555, 761.33091517857122,
      -641.95422767857121, 880.88760267857117, -761.33091517857122, 0.0,
      -708.36328571428544, 840.08928571428544, -840.08928571428555,
      971.95528571428542, -641.95422767857121, 761.33091517857122,
      -761.33091517857122, 880.88760267857117, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 100000.0 };

    static const real_T t[288] = { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
      -0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -1.0, -0.0,
      -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0,
      -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
      -0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -1.0,
      -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
      -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
      -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    static const real_T u[3] = { 40000.0, 40000.0, 62500.0 };

    static const real_T v[480] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    static const real_T ab[240] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

    static const int32_T b_Mrows[32] = { 91, 92, 93, 94, 95, 96, 97, 98, 151,
      152, 153, 154, 155, 156, 157, 158, 211, 212, 213, 214, 215, 216, 217, 218,
      219, 220, 221, 222, 223, 224, 225, 226 };

    real_T t_0[288];
    real_T q_0[128];
    real_T r_0[81];
    real_T b_B_6[24];
    real_T d_0[24];

    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  Integrator: '<S3>/Integrator'
     *  MATLAB Function: '<S3>/MATLAB Function'
     */
    c_tmp = cos(rtX.Integrator_CSTATE[2]);
    s_tmp = sin(rtX.Integrator_CSTATE[2]);
    for (r2 = 0; r2 < 9; r2++) {
      Ad[r2] = 0;
    }

    Qk[0] = 0.1 * c_tmp;
    Qk[3] = 0.1 * -s_tmp;
    Qk[6] = 0.0;
    Qk[1] = 0.1 * s_tmp;
    Qk[4] = 0.1 * c_tmp;
    Qk[7] = 0.0;
    Ad[0] = 1;
    Qk[2] = 0.0;
    Ad[4] = 1;
    Qk[5] = 0.0;
    Ad[8] = 1;
    Qk[8] = 0.1;
    for (r2 = 0; r2 < 4; r2++) {
      maxval = b_b_0[3 * r2 + 1];
      a21 = b_b_0[3 * r2];
      b_b = b_b_0[3 * r2 + 2];
      for (r1 = 0; r1 < 3; r1++) {
        Bd[r1 + 3 * r2] = (Qk[r1 + 3] * maxval + a21 * Qk[r1]) + Qk[r1 + 6] *
          b_b;
      }
    }

    for (r1 = 0; r1 < 16; r1++) {
      for (r2 = 0; r2 < 3; r2++) {
        i = 3 * r2 + 9 * r1;
        A[i] = Ad[3 * r2];
        A[i + 1] = Ad[3 * r2 + 1];
        A[i + 2] = Ad[3 * r2 + 2];
      }

      for (r2 = 0; r2 < 4; r2++) {
        i = 3 * r2 + 12 * r1;
        B_o[i] = Bd[3 * r2];
        B_o[i + 1] = Bd[3 * r2 + 1];
        B_o[i + 2] = Bd[3 * r2 + 2];
      }

      for (r2 = 0; r2 < 9; r2++) {
        b_I[r2] = 0;
      }

      b_I[0] = 1;
      b_I[4] = 1;
      b_I[8] = 1;
      for (r2 = 0; r2 < 3; r2++) {
        i = 3 * r2 + 9 * r1;
        C[i] = b_I[3 * r2];
        C[i + 1] = b_I[3 * r2 + 1];
        C[i + 2] = b_I[3 * r2 + 2];
      }
    }

    /* End of MATLAB Function: '<Root>/MATLAB Function' */
    if (rtmIsMajorTimeStep(rtM)) {
      /* Memory: '<S5>/LastPcov' */
      memcpy(&rtb_LastPcov[0], &rtDW.LastPcov_PreviousInput[0], 9U * sizeof
             (real_T));

      /* Memory: '<S5>/Memory' */
      for (i = 0; i < 32; i++) {
        rtb_Memory[i] = rtDW.Memory_PreviousInput[i];
      }

      /* End of Memory: '<S5>/Memory' */

      /* Memory: '<S5>/last_x' */
      rtb_last_x[0] = rtDW.last_x_PreviousInput[0];
      rtb_last_x[1] = rtDW.last_x_PreviousInput[1];
      rtb_last_x[2] = rtDW.last_x_PreviousInput[2];

      /* MATLAB Function: '<S33>/FixedHorizonOptimizer' incorporates:
       *  Integrator: '<S3>/Integrator'
       *  Memory: '<S5>/LastPcov'
       *  UnitDelay: '<S5>/last_mv'
       */
      for (r2 = 0; r2 < 144; r2++) {
        r1 = b[r2];
        b_C_tmp[r2] = r1;
        b_A[r2] = r1;
      }

      memset(&Bu[0], 0, 192U * sizeof(real_T));
      memset(&Bv[0], 0, 48U * sizeof(real_T));
      memset(&Dv[0], 0, 48U * sizeof(real_T));
      memset(&Dvm[0], 0, 48U * sizeof(real_T));
      memset(&Cm[0], 0, 144U * sizeof(real_T));
      for (r2 = 0; r2 < 9; r2++) {
        Qk[r2] = c[r2];
      }

      for (r2 = 0; r2 < 3; r2++) {
        r1 = (int32_T)Qk[3 * r2];
        b_A[3 * r2] = r1;
        b_C_tmp[3 * r2] = r1;
        i = 3 * r2 + 1;
        r1 = (int32_T)Qk[i];
        b_A[i] = r1;
        b_C_tmp[i] = r1;
        i = 3 * r2 + 2;
        r1 = (int32_T)Qk[i];
        b_A[i] = r1;
        b_C_tmp[i] = r1;
      }

      tmp[0] = 1;
      tmp[1] = 2;
      tmp[2] = 3;
      tmp[3] = 4;
      tmp_0[0] = 1;
      tmp_0[1] = 2;
      tmp_0[2] = 3;
      tmp_1[0] = 1.0;
      tmp_1[1] = 1.0;
      tmp_1[2] = 1.0;
      tmp_1[3] = 1.0;
      ref[0] = 1.0;
      ref[1] = 1.0;
      ref[2] = 1.0;
      memcpy(&Bd[0], &(*(real_T (*)[12])&B_o[0])[0], 12U * sizeof(real_T));
      memcpy(&L[0], &(*(real_T (*)[9])&C[0])[0], 9U * sizeof(real_T));
      memcpy(&d_0[0], &d[0], 24U * sizeof(real_T));
      mpc_plantupdate(&A[0], Bd, L, &b_A[0], d_0, &b_C_tmp[0], e, tmp, tmp_0,
                      tmp_1, ref, &Bu[0], &Bv[0], &Cm[0], &Dv[0], &Dvm[0], Qk,
                      Rk, Nk);
      tmp[0] = 1;
      tmp[1] = 2;
      tmp[2] = 3;
      tmp[3] = 4;
      tmp_0[0] = 1;
      tmp_0[1] = 2;
      tmp_0[2] = 3;
      tmp_1[0] = 1.0;
      tmp_1[1] = 1.0;
      tmp_1[2] = 1.0;
      tmp_1[3] = 1.0;
      ref[0] = 1.0;
      ref[1] = 1.0;
      ref[2] = 1.0;
      for (i = 0; i < 15; i++) {
        for (r2 = 0; r2 < 8; r2++) {
          r1 = (i + 1) * 24 + 3 * r2;
          d_0[3 * r2] = b_D[r1];
          d_0[3 * r2 + 1] = b_D[r1 + 1];
          d_0[3 * r2 + 2] = b_D[r1 + 2];
        }

        r2 = (i + 1) * 9;
        r1 = (i + 1) * 12;
        r3 = (i + 1) * 3;
        memcpy(&Bd[0], &(*(real_T (*)[12])&B_o[r1])[0], 12U * sizeof(real_T));
        memcpy(&L[0], &(*(real_T (*)[9])&C[r2])[0], 9U * sizeof(real_T));
        memcpy(&b_B_6[0], &(*(real_T (*)[24])&b_B_5[24 * (i + 1)])[0], 24U *
               sizeof(real_T));
        mpc_plantupdate(&A[r2], Bd, L, &b_A[r2], b_B_6, &b_C_tmp[r2], d_0, tmp,
                        tmp_0, tmp_1, ref, &Bu[r1], &Bv[r3], &Cm[r2], &Dv[r3],
                        &Dvm[r3], Qk, Rk, Nk);
      }

      for (i = 0; i < 32; i++) {
        b_Mlim[i] = f[i];
      }

      U_tmp[0] = 0.0;
      U_tmp[1] = 0.0;
      U_tmp[2] = 0.0;
      U_tmp[3] = 0.0;
      memset(&b_utarget[0], 0, 60U * sizeof(real_T));
      Bv[0] = 0.0;
      Bv[1] = 0.0;
      Bv[2] = 0.0;
      for (r1 = 0; r1 < 15; r1++) {
        for (r2 = 0; r2 < 3; r2++) {
          i = (r1 + 1) * 12 + r2;
          r3 = (r1 + 1) * 9 + r2;
          Bv_tmp = (r1 + 1) * 3 + r2;
          Bv[Bv_tmp] = (((Bu[i + 3] * 0.0 + Bu[i] * 0.0) + Bu[i + 6] * 0.0) +
                        Bu[i + 9] * 0.0) + ((b_A[r3 + 3] * 0.0 + b_A[r3] * 0.0)
            + b_A[r3 + 6] * 0.0);
          Dv[Bv_tmp] = (b_C_tmp[r3 + 3] * 0.0 + b_C_tmp[r3] * 0.0) + b_C_tmp[r3
            + 6] * 0.0;
        }
      }

      for (i = 0; i < 16; i++) {
        vseq[i] = 1.0;
      }

      for (i = 0; i < 15; i++) {
        /* Reference held constant across the prediction horizon at the
         * mission-parameter relative setpoint (see rtP_RelativeSetpoint).
         * This is NOT re-measured from CV each cycle -- it's the desired
         * relative pose the chaser should hold/reach with respect to the
         * target (e.g. a docking approach point), matching the original
         * model's fixed-setpoint design. */
        rseq[i * ny] = rtP_RelativeSetpoint[0];
        rseq[i * ny + 1] = rtP_RelativeSetpoint[1];
        rseq[i * ny + 2] = rtP_RelativeSetpoint[2];
      }

      memset(&L[0], 0, 9U * sizeof(real_T));
      for (i = 0; i < 3; i++) {
        L[i + 3 * i] = 1.0;
      }

      for (r2 = 0; r2 < 3; r2++) {
        maxval = rtb_LastPcov[3 * r2 + 1];
        a21 = rtb_LastPcov[3 * r2];
        b_b = rtb_LastPcov[3 * r2 + 2];
        for (i = 0; i < 3; i++) {
          r1 = 3 * r2 + i;
          c_A_tmp[r2 + 3 * i] = Cm[r1];
          Cm_0[r1] = (Cm[i + 3] * maxval + a21 * Cm[i]) + Cm[i + 6] * b_b;
        }
      }

      for (r2 = 0; r2 < 3; r2++) {
        maxval = Cm_0[r2 + 3];
        a21 = Cm_0[r2];
        b_b = Cm_0[r2 + 6];
        for (r1 = 0; r1 < 3; r1++) {
          i = 3 * r1 + r2;
          c_A[i] = ((c_A_tmp[3 * r1 + 1] * maxval + c_A_tmp[3 * r1] * a21) +
                    c_A_tmp[3 * r1 + 2] * b_b) + Rk[i];
        }
      }

      r1 = 0;
      r2 = 1;
      r3 = 2;
      maxval = fabs(c_A[0]);
      a21 = fabs(c_A[1]);
      if (a21 > maxval) {
        maxval = a21;
        r1 = 1;
        r2 = 0;
      }

      if (fabs(c_A[2]) > maxval) {
        r1 = 2;
        r2 = 1;
        r3 = 0;
      }

      c_A[r2] /= c_A[r1];
      c_A[r3] /= c_A[r1];
      c_A[r2 + 3] -= c_A[r1 + 3] * c_A[r2];
      c_A[r3 + 3] -= c_A[r1 + 3] * c_A[r3];
      c_A[r2 + 6] -= c_A[r1 + 6] * c_A[r2];
      c_A[r3 + 6] -= c_A[r1 + 6] * c_A[r3];
      if (fabs(c_A[r3 + 3]) > fabs(c_A[r2 + 3])) {
        i = r2;
        r2 = r3;
        r3 = i;
      }

      c_A[r3 + 3] /= c_A[r2 + 3];
      c_A[r3 + 6] -= c_A[r3 + 3] * c_A[r2 + 6];
      Rk[3 * r1] = L[0] / c_A[r1];
      maxval = c_A[r1 + 3];
      Rk[3 * r2] = L[3] - Rk[3 * r1] * maxval;
      a21 = c_A[r1 + 6];
      Rk[3 * r3] = L[6] - Rk[3 * r1] * a21;
      b_b = c_A[r2 + 3];
      Rk[3 * r2] /= b_b;
      Rk_tmp_0 = c_A[r2 + 6];
      Rk[3 * r3] -= Rk[3 * r2] * Rk_tmp_0;
      Rk_tmp_1 = c_A[r3 + 6];
      Rk[3 * r3] /= Rk_tmp_1;
      Rk_tmp_2 = c_A[r3 + 3];
      Rk[3 * r2] -= Rk[3 * r3] * Rk_tmp_2;
      Rk[3 * r1] -= Rk[3 * r3] * c_A[r3];
      Rk[3 * r1] -= Rk[3 * r2] * c_A[r2];
      i = 3 * r1 + 1;
      Rk[i] = L[1] / c_A[r1];
      Bv_tmp = 3 * r2 + 1;
      Rk[Bv_tmp] = L[4] - Rk[i] * maxval;
      Rk_tmp = 3 * r3 + 1;
      Rk[Rk_tmp] = L[7] - Rk[i] * a21;
      Rk[Bv_tmp] /= b_b;
      Rk[Rk_tmp] -= Rk[Bv_tmp] * Rk_tmp_0;
      Rk[Rk_tmp] /= Rk_tmp_1;
      Rk[Bv_tmp] -= Rk[Rk_tmp] * Rk_tmp_2;
      Rk[i] -= Rk[Rk_tmp] * c_A[r3];
      Rk[i] -= Rk[Bv_tmp] * c_A[r2];
      i = 3 * r1 + 2;
      Rk[i] = L[2] / c_A[r1];
      Bv_tmp = 3 * r2 + 2;
      Rk[Bv_tmp] = L[5] - Rk[i] * maxval;
      Rk_tmp = 3 * r3 + 2;
      Rk[Rk_tmp] = L[8] - Rk[i] * a21;
      Rk[Bv_tmp] /= b_b;
      Rk[Rk_tmp] -= Rk[Bv_tmp] * Rk_tmp_0;
      Rk[Rk_tmp] /= Rk_tmp_1;
      Rk[Bv_tmp] -= Rk[Rk_tmp] * Rk_tmp_2;
      Rk[i] -= Rk[Rk_tmp] * c_A[r3];
      Rk[i] -= Rk[Bv_tmp] * c_A[r2];
      for (r2 = 0; r2 < 3; r2++) {
        maxval = b_A[r2 + 3];
        a21 = b_A[r2];
        b_b = b_A[r2 + 6];
        for (r1 = 0; r1 < 3; r1++) {
          c_A[r2 + 3 * r1] = (rtb_LastPcov[3 * r1 + 1] * maxval + rtb_LastPcov[3
                              * r1] * a21) + rtb_LastPcov[3 * r1 + 2] * b_b;
        }

        maxval = c_A[r2 + 3];
        a21 = c_A[r2];
        b_b = c_A[r2 + 6];
        for (r1 = 0; r1 < 3; r1++) {
          i = 3 * r1 + r2;
          Cm_0[i] = ((c_A_tmp[3 * r1 + 1] * maxval + c_A_tmp[3 * r1] * a21) +
                     c_A_tmp[3 * r1 + 2] * b_b) + Nk[i];
        }

        Rk_tmp_0 = Cm_0[r2 + 3];
        Rk_tmp_1 = Cm_0[r2];
        Rk_tmp_2 = Cm_0[r2 + 6];
        for (r1 = 0; r1 < 3; r1++) {
          L[r2 + 3 * r1] = (Rk[3 * r1 + 1] * Rk_tmp_0 + Rk[3 * r1] * Rk_tmp_1) +
            Rk[3 * r1 + 2] * Rk_tmp_2;
        }

        ref[r2] = (((Bu[r2 + 3] * 0.0 + Bu[r2] * 0.0) + Bu[r2 + 6] * 0.0) +
                   Bu[r2 + 9] * 0.0) + rtb_last_x[r2];
      }

      for (r2 = 0; r2 < 3; r2++) {
        Rk_tmp_0 = 0.0;
        maxval = rtb_LastPcov[r2 + 3];
        a21 = rtb_LastPcov[r2];
        b_b = rtb_LastPcov[r2 + 6];
        for (r1 = 0; r1 < 3; r1++) {
          r3 = 3 * r1 + r2;
          Rk_tmp_0 += Cm[r3] * ref[r1];
          Nk[r3] = (c_A_tmp[3 * r1 + 1] * maxval + c_A_tmp[3 * r1] * a21) +
            c_A_tmp[3 * r1 + 2] * b_b;
        }

        rtb_last_x[r2] = rtX.Integrator_CSTATE[r2] - (Rk_tmp_0 + Dvm[r2]);
        maxval = Nk[r2 + 3];
        a21 = Nk[r2];
        b_b = Nk[r2 + 6];
        for (r1 = 0; r1 < 3; r1++) {
          rtb_LastPcov_0[r2 + 3 * r1] = (Rk[3 * r1 + 1] * maxval + Rk[3 * r1] *
            a21) + Rk[3 * r1 + 2] * b_b;
        }
      }

      maxval = rtb_last_x[1];
      a21 = rtb_last_x[0];
      b_b = rtb_last_x[2];
      for (r2 = 0; r2 < 3; r2++) {
        ref_0[r2] = ((rtb_LastPcov_0[r2 + 3] * maxval + rtb_LastPcov_0[r2] * a21)
                     + rtb_LastPcov_0[r2 + 6] * b_b) + ref[r2];
      }

      rtb_last_mv_0[0] = rtDW.last_mv_DSTATE[0];
      rtb_last_mv_0[1] = rtDW.last_mv_DSTATE[1];
      rtb_last_mv_0[2] = rtDW.last_mv_DSTATE[2];
      rtb_last_mv_0[3] = rtDW.last_mv_DSTATE[3];
      memset(&tmp_2[0], 0, 96U * sizeof(real_T));
      memset(&tmp_3[0], 0, sizeof(real_T) << 9U);
      tmp_1[0] = 0.040000000000000008;
      tmp_1[1] = 0.040000000000000008;
      tmp_1[2] = 0.040000000000000008;
      tmp_1[3] = 0.040000000000000008;
      tmp_4[0] = 0.010000000000000002;
      tmp_4[1] = 0.010000000000000002;
      tmp_4[2] = 0.010000000000000002;
      tmp_4[3] = 0.010000000000000002;
      memcpy(&q_0[0], &q[0], sizeof(real_T) << 7);
      memcpy(&r_0[0], &r[0], 81U * sizeof(real_T));
      memcpy(&t_0[0], &t[0], 288U * sizeof(real_T));
      mpcblock_optimizer(rseq, vseq, ref_0, rtb_last_mv_0, rtb_Memory, b_Mlim,
                         tmp_2, q_0, tmp_3, b_utarget, U_tmp, r_0, t_0, u, tmp_1,
                         v, tmp_4, ab, b_A, Bu, Bv, b_C_tmp, Dv, b_Mrows, rtDW.u,
                         U, &maxval, rtDW.iAout);
      for (r2 = 0; r2 < 3; r2++) {
        Rk_tmp_0 = Cm_0[r2 + 3];
        Rk_tmp_1 = Cm_0[r2];
        Rk_tmp_2 = Cm_0[r2 + 6];
        maxval = c_A[r2 + 3];
        a21 = c_A[r2];
        b_b = c_A[r2 + 6];
        for (r1 = 0; r1 < 3; r1++) {
          i = 3 * r1 + r2;
          c_A_tmp[i] = (L[r1 + 3] * Rk_tmp_0 + Rk_tmp_1 * L[r1]) + L[r1 + 6] *
            Rk_tmp_2;
          rtb_LastPcov[i] = (b_A[r1 + 3] * maxval + a21 * b_A[r1]) + b_A[r1 + 6]
            * b_b;
        }
      }

      for (r2 = 0; r2 < 9; r2++) {
        Rk[r2] = (rtb_LastPcov[r2] - c_A_tmp[r2]) + Qk[r2];
      }

      Rk_tmp_0 = rtDW.u[0];
      Rk_tmp_1 = rtDW.u[1];
      Rk_tmp_2 = rtDW.u[2];
      tmp_5 = rtDW.u[3];
      ref_1 = ref[1];
      ref_2 = ref[0];
      ref_3 = ref[2];
      maxval = rtb_last_x[1];
      a21 = rtb_last_x[0];
      b_b = rtb_last_x[2];
      for (r2 = 0; r2 < 3; r2++) {
        rtDW.Pk1[3 * r2] = (Rk[3 * r2] + Rk[r2]) * 0.5;
        r1 = 3 * r2 + 1;
        rtDW.Pk1[r1] = (Rk[r2 + 3] + Rk[r1]) * 0.5;
        r1 = 3 * r2 + 2;
        rtDW.Pk1[r1] = (Rk[r2 + 6] + Rk[r1]) * 0.5;
        rtDW.xk1[r2] = (((((Bu[r2 + 3] * Rk_tmp_1 + Bu[r2] * Rk_tmp_0) + Bu[r2 +
                           6] * Rk_tmp_2) + Bu[r2 + 9] * tmp_5) + ((b_A[r2 + 3] *
          ref_1 + b_A[r2] * ref_2) + b_A[r2 + 6] * ref_3)) + Bv[r2]) + ((L[r2 +
          3] * maxval + L[r2] * a21) + L[r2 + 6] * b_b);
      }

      /* End of MATLAB Function: '<S33>/FixedHorizonOptimizer' */
    }

    /* MATLAB Function: '<S3>/MATLAB Function' */
    Qk[0] = c_tmp;
    Qk[3] = -s_tmp;
    Qk[6] = 0.0;
    Qk[1] = s_tmp;
    Qk[4] = c_tmp;
    Qk[7] = 0.0;
    Qk[2] = 0.0;
    Qk[5] = 0.0;
    Qk[8] = 1.0;
    for (r2 = 0; r2 < 3; r2++) {
      c_tmp = 0.0;
      s_tmp = Qk[r2 + 3];
      maxval = Qk[r2];
      i = (int32_T)Qk[r2 + 6];
      for (r1 = 0; r1 < 4; r1++) {
        c_tmp += ((b_b_0[3 * r1 + 1] * s_tmp + b_b_0[3 * r1] * maxval) + b_b_0[3
                  * r1 + 2] * (real_T)i) * rtDW.u[r1];
      }

      rtDW.x_dot[r2] = c_tmp;
    }
  }

  if (rtmIsMajorTimeStep(rtM)) {
    int32_T i;
    if (rtmIsMajorTimeStep(rtM)) {
      /* Update for Memory: '<S5>/LastPcov' */
      memcpy(&rtDW.LastPcov_PreviousInput[0], &rtDW.Pk1[0], 9U * sizeof(real_T));

      /* Update for Memory: '<S5>/Memory' */
      for (i = 0; i < 32; i++) {
        rtDW.Memory_PreviousInput[i] = rtDW.iAout[i];
      }

      /* End of Update for Memory: '<S5>/Memory' */

      /* Update for UnitDelay: '<S5>/last_mv' */
      rtDW.last_mv_DSTATE[0] = rtDW.u[0];
      rtDW.last_mv_DSTATE[1] = rtDW.u[1];
      rtDW.last_mv_DSTATE[2] = rtDW.u[2];
      rtDW.last_mv_DSTATE[3] = rtDW.u[3];

      /* Update for Memory: '<S5>/last_x' */
      rtDW.last_x_PreviousInput[0] = rtDW.xk1[0];
      rtDW.last_x_PreviousInput[1] = rtDW.xk1[1];
      rtDW.last_x_PreviousInput[2] = rtDW.xk1[2];
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.1s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.1, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void modelv2_derivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<S3>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = rtDW.x_dot[0];
  _rtXdot->Integrator_CSTATE[1] = rtDW.x_dot[1];
  _rtXdot->Integrator_CSTATE[2] = rtDW.x_dot[2];
}

/* Model initialize function */
void modelv2_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&rtM->solverInfo, (boolean_T**)
      &rtM->contStateDisabled);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetIsContModeFrozen(&rtM->solverInfo, false);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->intgData.f[3] = rtM->odeF[3];
  rtM->contStates = ((X *) &rtX);
  rtM->contStateDisabled = ((XDis *) &rtXDis);
  rtM->Timing.tStart = (0.0);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetSolverName(&rtM->solverInfo,"ode4");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.1;

  /* InitializeConditions for Integrator: '<S3>/Integrator' */
  rtX.Integrator_CSTATE[0] = 0.0;
  rtX.Integrator_CSTATE[1] = 0.0;
  rtX.Integrator_CSTATE[2] = 0.0;

  /* InitializeConditions for Memory: '<S5>/LastPcov' */
  memcpy(&rtDW.LastPcov_PreviousInput[0], &rtConstP.LastPcov_InitialCondition[0],
         9U * sizeof(real_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
