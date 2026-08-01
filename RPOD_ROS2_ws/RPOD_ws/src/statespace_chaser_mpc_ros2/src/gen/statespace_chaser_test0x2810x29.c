/*
 * File: statespace_chaser_test0x2810x29.c
 *
 * Code generated for Simulink model 'statespace_chaser_test0x2810x29'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Tue Jul  7 04:31:24 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 10
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "statespace_chaser_test0x2810x29.h"
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include "math.h"

/* Named constants for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
#define degrees                        (25)
#define ny                             (3)
#define p                              (25)

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
 * External input: the target's current tumble/rotation angle (radians),
 * as measured by the CV pipeline each cycle. This replaces the original
 * generated code's assumption of a fixed, known tumble rate
 * (theta_t = 0.1 * simulated_time) -- see the patched comment at its use
 * site (search "rtP_TargetTumbleAngle" in this file). A real target's
 * tumble rate and phase are not known in advance; they must be estimated
 * from live measurements (e.g. the yaw component of the CV-measured
 * relative orientation) and written here every control cycle.
 */
real_T rtP_TargetTumbleAngle = 0.0;

/*
 * Mission parameters: approach-distance schedule (hold -> ramp down ->
 * hold at final standoff) and docking-point geometry (center offset +
 * arm/port offset), all originally hardcoded literals in the generated
 * reference-sequence computation. Defaults below match the original
 * generated code's values exactly; change via the ROS 2 node's parameters
 * to tune the approach profile without recompiling.
 */
real_T rtP_ApproachHoldRadius   = 0.5;    /* original: held constant while Clock < 5.0 */
real_T rtP_ApproachFinalRadius  = 0.05;   /* original: held constant after Clock > 20.0 */
real_T rtP_ApproachRampStartTime = 5.0;   /* original: ramp begins at t = 5.0 s */
real_T rtP_ApproachRampEndTime   = 20.0;  /* original: ramp ends at t = 20.0 s */
real_T rtP_DockCenterX = 1.5;             /* original: hardcoded 1.5 */
real_T rtP_DockCenterY = 1.5;             /* original: hardcoded 1.5 (same literal, kept separate for tunability) */
real_T rtP_DockArmOffset = 0.4;           /* original: hardcoded 0.4 */

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
extern void statespace_chaser_test0x2810x29_derivatives(void);

/* Forward declaration for local functions */
static void mpc_plantupdate(const real_T a[9], real_T b[12], real_T c[9], real_T
  b_A[9], real_T b_B_2[24], real_T b_C[9], const real_T b_D[24], const int32_T
  b_mvindex[4], const int32_T b_myindex[3], const real_T b_Uscale[4], const
  real_T b_Yscale[3], real_T Bu[12], real_T Bv[3], real_T Cm[9], real_T Dv[3],
  real_T Dvm[3], real_T QQ[9], real_T RR[9], real_T NN[9]);
static int32_T xpotrf(real_T b_A[625]);
static real_T minimum(const real_T x[25]);
static void trisolve(const real_T b_A[625], real_T b_B_3[625]);
static real_T norm(const real_T x[25]);
static real_T maximum(const real_T x[25]);
static real_T xnrm2(int32_T n, const real_T x[625], int32_T ix0);
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[625], int32_T ia0,
                  const real_T x[625], int32_T ix0, real_T y[25]);
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[25], real_T b_A[625], int32_T ia0);
static real_T KWIKfactor(const real_T b_Ac[2400], const int32_T iC[96], int32_T
  nA, const real_T b_Linv[625], real_T RLinv[625], real_T b_D[625], real_T b_H
  [625], int32_T n);
static void DropConstraint(int32_T kDrop, boolean_T iA[96], int32_T *nA, int32_T
  iC[96]);
static void qpkwik(const real_T b_Linv[625], const real_T b_Hinv[625], const
                   real_T f[25], const real_T b_Ac[2400], const real_T b[96],
                   boolean_T iA[96], int32_T maxiter, real_T FeasTol, real_T x
                   [25], real_T lambda[96], int32_T *status);
static void mpcblock_optimizer(const real_T rseq[75], const real_T vseq[26],
  const real_T x[3], const real_T old_u[4], const boolean_T iA[96], const real_T
  b_Mlim[96], real_T b_Mx[288], real_T b_Mu1[384], real_T b_Mv[2496], const
  real_T b_utarget[100], const real_T b_uoff[4], real_T b_H[625], real_T b_Ac
  [2400], const real_T b_Wy[3], const real_T b_Wdu[4], const real_T b_Jm[2400],
  const real_T b_Wu[4], const real_T b_I1[400], const real_T b_A[234], const
  real_T Bu[312], const real_T Bv[78], const real_T b_C[234], const real_T Dv[78],
  const int32_T b_Mrows[96], real_T u[4], real_T useq[104], real_T *status,
  boolean_T iAout[96]);
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
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  statespace_chaser_test0x2810x29_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  statespace_chaser_test0x2810x29_step();
  statespace_chaser_test0x2810x29_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  statespace_chaser_test0x2810x29_step();
  statespace_chaser_test0x2810x29_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
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
static int32_T xpotrf(real_T b_A[625])
{
  int32_T b_k;
  int32_T info;
  int32_T j;
  int32_T jm1;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 25)) {
    real_T c;
    real_T ssq;
    int32_T idxAjj;
    idxAjj = j * 25 + j;
    ssq = 0.0;
    if (j >= 1) {
      for (b_k = 0; b_k < j; b_k++) {
        c = b_A[b_k * 25 + j];
        ssq += c * c;
      }
    }

    ssq = b_A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      b_A[idxAjj] = ssq;
      if (j + 1 < 25) {
        if (j != 0) {
          int32_T b_iy;
          b_iy = ((j - 1) * 25 + j) + 2;
          for (b_k = j + 2; b_k <= b_iy; b_k += 25) {
            int32_T d;
            jm1 = b_k - j;
            c = -b_A[div_nde_s32_floor(jm1 - 2, 25) * 25 + j];
            d = jm1 + 23;
            for (jm1 = b_k; jm1 <= d; jm1++) {
              int32_T tmp;
              tmp = ((idxAjj + jm1) - b_k) + 1;
              b_A[tmp] += b_A[jm1 - 1] * c;
            }
          }
        }

        ssq = 1.0 / ssq;
        jm1 = (idxAjj - j) + 25;
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
static real_T minimum(const real_T x[25])
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
    while ((!exitg1) && (k < 26)) {
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
    for (k = idx + 1; k < 26; k++) {
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
static void trisolve(const real_T b_A[625], real_T b_B_3[625])
{
  int32_T b_k;
  int32_T i;
  int32_T j;
  for (j = 0; j < 25; j++) {
    int32_T jBcol;
    jBcol = 25 * j;
    for (b_k = 0; b_k < 25; b_k++) {
      real_T b_B_4;
      int32_T b_B_tmp;
      int32_T kAcol;
      kAcol = 25 * b_k;
      b_B_tmp = b_k + jBcol;
      b_B_4 = b_B_3[b_B_tmp];
      if (b_B_4 != 0.0) {
        b_B_3[b_B_tmp] = b_B_4 / b_A[b_k + kAcol];
        for (i = b_k + 2; i < 26; i++) {
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
static real_T norm(const real_T x[25])
{
  real_T scale;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 25; k++) {
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
static real_T maximum(const real_T x[25])
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
    while ((!exitg1) && (k < 26)) {
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
    for (k = idx + 1; k < 26; k++) {
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
static real_T xnrm2(int32_T n, const real_T x[625], int32_T ix0)
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
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[625], int32_T ia0,
                  const real_T x[625], int32_T ix0, real_T y[25])
{
  int32_T b_iy;
  int32_T ia;
  if ((b_m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 25 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 25) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nde_s32_floor(b_iy - ia0, 25);
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[25], real_T b_A[625], int32_T ia0)
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

      jA += 25;
    }
  }
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static real_T KWIKfactor(const real_T b_Ac[2400], const int32_T iC[96], int32_T
  nA, const real_T b_Linv[625], real_T RLinv[625], real_T b_D[625], real_T b_H
  [625], int32_T n)
{
  real_T tau[25];
  real_T work[25];
  real_T RLinv_0;
  real_T Status;
  real_T b_A;
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
  memset(&RLinv[0], 0, 625U * sizeof(real_T));
  for (k_i = 0; k_i < nA; k_i++) {
    b_lastv = iC[k_i];
    for (b_coltop = 0; b_coltop < 25; b_coltop++) {
      RLinv_0 = 0.0;
      for (knt = 0; knt < 25; knt++) {
        RLinv_0 += b_Ac[(96 * knt + b_lastv) - 1] * b_Linv[25 * knt + b_coltop];
      }

      RLinv[b_coltop + 25 * k_i] = RLinv_0;
    }
  }

  memcpy(&rtDW.b_A[0], &RLinv[0], 625U * sizeof(real_T));
  memset(&tau[0], 0, 25U * sizeof(real_T));
  memset(&work[0], 0, 25U * sizeof(real_T));
  for (k_i = 0; k_i < 25; k_i++) {
    ii = k_i * 25 + k_i;
    if (k_i + 1 < 25) {
      RLinv_0 = rtDW.b_A[ii];
      b_lastv = ii + 2;
      tau[k_i] = 0.0;
      beta1 = xnrm2(24 - k_i, rtDW.b_A, ii + 2);
      if (beta1 != 0.0) {
        b_A = rtDW.b_A[ii];
        beta1 = rt_hypotd_snf(b_A, beta1);
        if (b_A >= 0.0) {
          beta1 = -beta1;
        }

        if (fabs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          coltop = (ii - k_i) + 25;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
              rtDW.b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            RLinv_0 *= 9.9792015476736E+291;
          } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

          beta1 = rt_hypotd_snf(RLinv_0, xnrm2(24 - k_i, rtDW.b_A, ii + 2));
          if (RLinv_0 >= 0.0) {
            beta1 = -beta1;
          }

          tau[k_i] = (beta1 - RLinv_0) / beta1;
          RLinv_0 = 1.0 / (RLinv_0 - beta1);
          for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
            rtDW.b_A[b_coltop - 1] *= RLinv_0;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 1.0020841800044864E-292;
          }

          RLinv_0 = beta1;
        } else {
          tau[k_i] = (beta1 - b_A) / beta1;
          RLinv_0 = 1.0 / (b_A - beta1);
          b_coltop = (ii - k_i) + 25;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            rtDW.b_A[knt - 1] *= RLinv_0;
          }

          RLinv_0 = beta1;
        }
      }

      rtDW.b_A[ii] = 1.0;
      if (tau[k_i] != 0.0) {
        b_lastv = 25 - k_i;
        knt = (ii - k_i) + 24;
        while ((b_lastv > 0) && (rtDW.b_A[knt] == 0.0)) {
          b_lastv--;
          knt--;
        }

        knt = 24 - k_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = ((knt - 1) * 25 + ii) + 25;
          coltop = b_coltop;
          do {
            exitg1 = 0;
            if (coltop + 1 <= b_coltop + b_lastv) {
              if (rtDW.b_A[coltop] != 0.0) {
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
        xgemv(b_lastv, knt, rtDW.b_A, ii + 26, rtDW.b_A, ii + 1, work);
        xgerc(b_lastv, knt, -tau[k_i], ii + 1, work, rtDW.b_A, ii + 26);
      }

      rtDW.b_A[ii] = RLinv_0;
    } else {
      tau[24] = 0.0;
    }
  }

  for (k_i = 0; k_i < 25; k_i++) {
    for (ii = 0; ii <= k_i; ii++) {
      rtDW.R[ii + 25 * k_i] = rtDW.b_A[25 * k_i + ii];
    }

    for (ii = k_i + 2; ii < 26; ii++) {
      rtDW.R[(ii + 25 * k_i) - 1] = 0.0;
    }

    work[k_i] = 0.0;
  }

  for (k_i = 24; k_i >= 0; k_i--) {
    b_lastv = (k_i * 25 + k_i) + 26;
    if (k_i + 1 < 25) {
      rtDW.b_A[b_lastv - 26] = 1.0;
      if (tau[k_i] != 0.0) {
        knt = 25 - k_i;
        b_coltop = b_lastv - k_i;
        while ((knt > 0) && (rtDW.b_A[b_coltop - 2] == 0.0)) {
          knt--;
          b_coltop--;
        }

        b_coltop = 24 - k_i;
        exitg2 = false;
        while ((!exitg2) && (b_coltop > 0)) {
          coltop = (b_coltop - 1) * 25 + b_lastv;
          ii = coltop;
          do {
            exitg1 = 0;
            if (ii <= (coltop + knt) - 1) {
              if (rtDW.b_A[ii - 1] != 0.0) {
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
        xgemv(knt, b_coltop, rtDW.b_A, b_lastv, rtDW.b_A, b_lastv - 25, work);
        xgerc(knt, b_coltop, -tau[k_i], b_lastv - 25, work, rtDW.b_A, b_lastv);
      }

      b_coltop = b_lastv - k_i;
      for (knt = b_lastv - 24; knt < b_coltop; knt++) {
        rtDW.b_A[knt - 1] *= -tau[k_i];
      }
    }

    rtDW.b_A[b_lastv - 26] = 1.0 - tau[k_i];
    for (knt = 0; knt < k_i; knt++) {
      rtDW.b_A[(b_lastv - knt) - 27] = 0.0;
    }
  }

  k_i = 0;
  do {
    exitg1 = 0;
    if (k_i <= nA - 1) {
      if (fabs(rtDW.R[25 * k_i + k_i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        k_i++;
      }
    } else {
      for (k_i = 0; k_i < n; k_i++) {
        for (ii = 0; ii < n; ii++) {
          RLinv_0 = 0.0;
          for (b_coltop = 0; b_coltop < 25; b_coltop++) {
            RLinv_0 += b_Linv[25 * k_i + b_coltop] * rtDW.b_A[25 * ii + b_coltop];
          }

          rtDW.TL[k_i + 25 * ii] = RLinv_0;
        }
      }

      memset(&RLinv[0], 0, 625U * sizeof(real_T));
      for (k_i = nA; k_i >= 1; k_i--) {
        b_coltop = (k_i - 1) * 25;
        knt = (k_i + b_coltop) - 1;
        RLinv[knt] = 1.0;
        for (ii = k_i; ii <= nA; ii++) {
          coltop = ((ii - 1) * 25 + k_i) - 1;
          RLinv[coltop] /= rtDW.R[knt];
        }

        if (k_i > 1) {
          for (ii = 0; ii <= k_i - 2; ii++) {
            for (b_lastv = k_i; b_lastv <= nA; b_lastv++) {
              knt = (b_lastv - 1) * 25;
              coltop = knt + ii;
              RLinv[coltop] -= RLinv[(knt + k_i) - 1] * rtDW.R[b_coltop + ii];
            }
          }
        }
      }

      for (k_i = 0; k_i < n; k_i++) {
        for (ii = k_i + 1; ii <= n; ii++) {
          b_coltop = (ii - 1) * 25 + k_i;
          b_H[b_coltop] = 0.0;
          for (b_lastv = nA + 1; b_lastv <= n; b_lastv++) {
            knt = (b_lastv - 1) * 25;
            b_H[b_coltop] -= rtDW.TL[(knt + ii) - 1] * rtDW.TL[knt + k_i];
          }

          b_H[(ii + 25 * k_i) - 1] = b_H[b_coltop];
        }
      }

      for (k_i = 0; k_i < nA; k_i++) {
        for (ii = 0; ii < n; ii++) {
          b_coltop = 25 * k_i + ii;
          b_D[b_coltop] = 0.0;
          for (b_lastv = k_i + 1; b_lastv <= nA; b_lastv++) {
            knt = (b_lastv - 1) * 25;
            b_D[b_coltop] += rtDW.TL[knt + ii] * RLinv[knt + k_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S33>/FixedHorizonOptimizer' */
static void DropConstraint(int32_T kDrop, boolean_T iA[96], int32_T *nA, int32_T
  iC[96])
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
static void qpkwik(const real_T b_Linv[625], const real_T b_Hinv[625], const
                   real_T f[25], const real_T b_Ac[2400], const real_T b[96],
                   boolean_T iA[96], int32_T maxiter, real_T FeasTol, real_T x
                   [25], real_T lambda[96], int32_T *status)
{
  real_T cTol[96];
  real_T Opt[50];
  real_T Rhs[50];
  real_T r[25];
  real_T z[25];
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T rMin;
  real_T t;
  int32_T iC[96];
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
  memset(&x[0], 0, 25U * sizeof(real_T));
  memset(&lambda[0], 0, 96U * sizeof(real_T));
  *status = 1;
  memset(&r[0], 0, 25U * sizeof(real_T));
  rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 96; i++) {
    cTol[i] = 1.0;
    iC[i] = 0;
  }

  nA = 0;
  for (tmp = 0; tmp < 96; tmp++) {
    if (iA[tmp]) {
      nA++;
      iC[nA - 1] = tmp + 1;
    }
  }

  guard1 = false;
  if (nA > 0) {
    memset(&Opt[0], 0, 50U * sizeof(real_T));
    for (i = 0; i < 25; i++) {
      Rhs[i] = f[i];
      Rhs[i + 25] = 0.0;
    }

    DualFeasible = false;
    tmp = (int32_T)rt_roundd_snf(0.3 * (real_T)nA);
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (*status <= maxiter)) {
        Xnorm0 = KWIKfactor(b_Ac, iC, nA, b_Linv, rtDW.RLinv, rtDW.b_D, rtDW.b_H,
                            degrees);
        if (Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            nA = 0;
            memset(&iA[0], 0, 96U * sizeof(boolean_T));
            memset(&iC[0], 0, 96U * sizeof(int32_T));
            ColdReset = true;
          }
        } else {
          for (i = 0; i < nA; i++) {
            Rhs[i + 25] = b[iC[i] - 1];
            for (iSave = i + 1; iSave <= nA; iSave++) {
              U_tmp = (25 * i + iSave) - 1;
              rtDW.U[U_tmp] = 0.0;
              for (iC_0 = 0; iC_0 < nA; iC_0++) {
                rtDW.U[U_tmp] += rtDW.RLinv[(25 * iC_0 + iSave) - 1] *
                  rtDW.RLinv[25 * iC_0 + i];
              }

              rtDW.U[i + 25 * (iSave - 1)] = rtDW.U[U_tmp];
            }
          }

          for (i = 0; i < 25; i++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 25; iC_0++) {
              Xnorm0 += rtDW.b_H[25 * iC_0 + i] * Rhs[iC_0];
            }

            Opt[i] = Xnorm0;
            for (iSave = 0; iSave < nA; iSave++) {
              Opt[i] += rtDW.b_D[25 * iSave + i] * Rhs[iSave + 25];
            }
          }

          for (i = 0; i < nA; i++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 25; iC_0++) {
              Xnorm0 += rtDW.b_D[25 * i + iC_0] * Rhs[iC_0];
            }

            Opt[i + 25] = Xnorm0;
            for (iSave = 0; iSave < nA; iSave++) {
              Opt[i + 25] += rtDW.U[25 * iSave + i] * Rhs[iSave + 25];
            }
          }

          Xnorm0 = -1.0E-12;
          i = -1;
          for (iSave = 0; iSave < nA; iSave++) {
            cMin = Opt[iSave + 25];
            lambda[iC[iSave] - 1] = cMin;
            if ((cMin < Xnorm0) && (iSave + 1 <= nA)) {
              i = iSave;
              Xnorm0 = cMin;
            }
          }

          if (i + 1 <= 0) {
            DualFeasible = true;
            memcpy(&x[0], &Opt[0], 25U * sizeof(real_T));
          } else {
            (*status)++;
            if (tmp <= 5) {
              iC_0 = 5;
            } else {
              iC_0 = tmp;
            }

            if (*status > iC_0) {
              nA = 0;
              memset(&iA[0], 0, 96U * sizeof(boolean_T));
              memset(&iC[0], 0, 96U * sizeof(int32_T));
              ColdReset = true;
            } else {
              lambda[iC[i] - 1] = 0.0;
              DropConstraint(i + 1, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lambda[0], 0, 96U * sizeof(real_T));
          for (tmp = 0; tmp < 25; tmp++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 25; iC_0++) {
              Xnorm0 += -b_Hinv[25 * iC_0 + tmp] * f[iC_0];
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
    for (tmp = 0; tmp < 25; tmp++) {
      Xnorm0 = 0.0;
      for (iC_0 = 0; iC_0 < 25; iC_0++) {
        Xnorm0 += -b_Hinv[25 * iC_0 + tmp] * f[iC_0];
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
      for (i = 0; i < 96; i++) {
        if (!cTolComputed) {
          for (iSave = 0; iSave < 25; iSave++) {
            z[iSave] = fabs(b_Ac[96 * iSave + i] * x[iSave]);
          }

          cTol[i] = fmax(cTol[i], maximum(z));
        }

        if (!iA[i]) {
          t = 0.0;
          for (iC_0 = 0; iC_0 < 25; iC_0++) {
            t += b_Ac[96 * iC_0 + i] * x[iC_0];
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
              for (iC_0 = 0; iC_0 < 25; iC_0++) {
                cMin = 0.0;
                for (i = 0; i < 25; i++) {
                  cMin += b_Hinv[25 * i + iC_0] * b_Ac[96 * i + tmp];
                }

                z[iC_0] = cMin;
              }

              guard2 = true;
            } else {
              cMin = KWIKfactor(b_Ac, iC, nA, b_Linv, rtDW.RLinv, rtDW.b_D,
                                rtDW.b_H, degrees);
              if (cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (iC_0 = 0; iC_0 < 625; iC_0++) {
                  rtDW.U[iC_0] = -rtDW.b_H[iC_0];
                }

                for (iC_0 = 0; iC_0 < 25; iC_0++) {
                  cMin = 0.0;
                  for (i = 0; i < 25; i++) {
                    cMin += rtDW.U[25 * i + iC_0] * b_Ac[96 * i + tmp];
                  }

                  z[iC_0] = cMin;
                }

                for (i = 0; i < nA; i++) {
                  t = 0.0;
                  for (iC_0 = 0; iC_0 < 25; iC_0++) {
                    t += b_Ac[96 * iC_0 + tmp] * rtDW.b_D[25 * i + iC_0];
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
              for (iSave = 0; iSave < 25; iSave++) {
                cVal += b_Ac[96 * iSave + tmp] * z[iSave];
              }

              if (cVal <= 0.0) {
                cVal = 0.0;
                ColdReset = true;
              } else {
                t = 0.0;
                for (iC_0 = 0; iC_0 < 25; iC_0++) {
                  t += b_Ac[96 * iC_0 + tmp] * x[iC_0];
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
                  if ((iC_0 <= 96) && (lambda[iC_0 - 1] < 0.0)) {
                    lambda[iC_0 - 1] = 0.0;
                  }
                }

                lambda[tmp] += t;
                frexp(1.0, &exponent);
                if (fabs(t - cMin) < 2.2204460492503131E-16) {
                  DropConstraint(i, iA, &nA, iC);
                }

                if (!ColdReset) {
                  for (iC_0 = 0; iC_0 < 25; iC_0++) {
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
              for (tmp = 0; tmp < 96; tmp++) {
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
static void mpcblock_optimizer(const real_T rseq[75], const real_T vseq[26],
  const real_T x[3], const real_T old_u[4], const boolean_T iA[96], const real_T
  b_Mlim[96], real_T b_Mx[288], real_T b_Mu1[384], real_T b_Mv[2496], const
  real_T b_utarget[100], const real_T b_uoff[4], real_T b_H[625], real_T b_Ac
  [2400], const real_T b_Wy[3], const real_T b_Wdu[4], const real_T b_Jm[2400],
  const real_T b_Wu[4], const real_T b_I1[400], const real_T b_A[234], const
  real_T Bu[312], const real_T Bv[78], const real_T b_C[234], const real_T Dv[78],
  const int32_T b_Mrows[96], real_T u[4], real_T useq[104], real_T *status,
  boolean_T iAout[96])
{
  real_T c_Su1[300];
  real_T c_Sx[225];
  real_T AA[216];
  real_T b_I1_0[96];
  real_T c_Ku1[96];
  real_T c_Su1_0[96];
  real_T c_Kx[72];
  real_T varargin_1[25];
  real_T zopt[25];
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
  int8_T b[625];
  int8_T b_B_0[16];
  int8_T rows[3];
  int8_T b_B_1;
  int8_T c_A;
  int8_T rows_0;
  int8_T rows_1;
  int8_T rows_2;
  static const int8_T c_A_0[625] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  memset(&useq[0], 0, 104U * sizeof(real_T));
  memset(&iAout[0], 0, 96U * sizeof(boolean_T));
  normH = Bv[0];
  s = Bv[1];
  Bv_0 = Bv[2];
  for (i = 0; i < 3; i++) {
    rtDW.c_Hv[i] = (b_C[i + 9] * normH + b_C[i + 12] * s) + b_C[i + 15] * Bv_0;
    rtDW.c_Hv[i + 75] = Dv[i + 3];
  }

  for (i = 0; i < 24; i++) {
    c_Hv_tmp = (i + 2) * 75;
    rtDW.c_Hv[c_Hv_tmp] = 0.0;
    rtDW.c_Hv[c_Hv_tmp + 1] = 0.0;
    rtDW.c_Hv[c_Hv_tmp + 2] = 0.0;
  }

  for (i = 0; i < 26; i++) {
    memset(&rtDW.c_Hv[i * 75 + 3], 0, 72U * sizeof(real_T));
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
    c_Sx[75 * i] = b_C_0[3 * i];
    c_Sx[75 * i + 1] = b_C_0[3 * i + 1];
    c_Sx[75 * i + 2] = b_C_0[3 * i + 2];
  }

  for (i = 0; i < 72; i++) {
    c_Sx[i + 3] = 0.0;
    c_Sx[i + 78] = 0.0;
    c_Sx[i + 153] = 0.0;
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
    c_Su1[75 * i] = b_C_1[3 * i];
    c_Su1[75 * i + 1] = b_C_1[3 * i + 1];
    c_Su1[75 * i + 2] = b_C_1[3 * i + 2];
  }

  for (i = 0; i < 72; i++) {
    c_Su1[i + 3] = 0.0;
    c_Su1[i + 78] = 0.0;
    c_Su1[i + 153] = 0.0;
    c_Su1[i + 228] = 0.0;
  }

  for (i = 0; i < 4; i++) {
    rtDW.b_a[75 * i] = c_Su1[75 * i];
    b_a_tmp = 75 * i + 1;
    rtDW.b_a[b_a_tmp] = c_Su1[b_a_tmp];
    b_a_tmp = 75 * i + 2;
    rtDW.b_a[b_a_tmp] = c_Su1[b_a_tmp];
  }

  for (i = 0; i < 96; i++) {
    b_a_tmp = (i + 4) * 75;
    rtDW.b_a[b_a_tmp] = 0.0;
    rtDW.b_a[b_a_tmp + 1] = 0.0;
    rtDW.b_a[b_a_tmp + 2] = 0.0;
  }

  for (i = 0; i < 100; i++) {
    memset(&rtDW.b_a[i * 75 + 3], 0, 72U * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    AA[3 * i] = b_A[3 * i + 9];
    AA[3 * i + 1] = b_A[3 * i + 10];
    AA[3 * i + 2] = b_A[3 * i + 11];
  }

  for (i = 0; i < 69; i++) {
    AA_tmp = (i + 3) * 3;
    AA[AA_tmp] = 0.0;
    AA[AA_tmp + 1] = 0.0;
    AA[AA_tmp + 2] = 0.0;
  }

  for (kidx = 0; kidx < 24; kidx++) {
    kidx_tmp = (kidx + 1) * 3;
    for (i = 0; i < 3; i++) {
      rows_2 = (int8_T)((int8_T)(kidx_tmp + 1) + i);
      rows[i] = rows_2;
      b_a_tmp = (kidx + 2) * 9 + i;
      normH = b_C[b_a_tmp];
      s = b_C[b_a_tmp + 3];
      Bv_0 = b_C[b_a_tmp + 6];
      for (b_a_tmp = 0; b_a_tmp < 4; b_a_tmp++) {
        AA_tmp = (75 * b_a_tmp + rows_2) - 1;
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
      b_a_tmp = (i + Tries) * 75;
      rtDW.b_a[(rows_2 + b_a_tmp) - 1] = c_Su1[(75 * i + rows_2) - 1];
      rtDW.b_a[(rows_0 + b_a_tmp) - 1] = c_Su1[(75 * i + rows_0) - 1];
      rtDW.b_a[(rows_1 + b_a_tmp) - 1] = c_Su1[(75 * i + rows_1) - 1];
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
            Bu[AA_tmp + 2] * Bv_0) + c_Su1[(75 * b_a_tmp + rows[i]) - 1];
        }
      }

      rows_2 = rows[0];
      rows_0 = rows[1];
      rows_1 = rows[2];
      for (i = 0; i < 4; i++) {
        c_Su1[(rows_2 + 75 * i) - 1] = b_C_1[3 * i];
        c_Su1[(rows_0 + 75 * i) - 1] = b_C_1[3 * i + 1];
        c_Su1[(rows_1 + 75 * i) - 1] = b_C_1[3 * i + 2];
      }

      c_Hv_tmp <<= 2;
      for (i = 0; i < 4; i++) {
        b_a_tmp = (i + c_Hv_tmp) * 75;
        rtDW.b_a[(rows_2 + b_a_tmp) - 1] = c_Su1[(75 * i + rows_2) - 1];
        rtDW.b_a[(rows_0 + b_a_tmp) - 1] = c_Su1[(75 * i + rows_0) - 1];
        rtDW.b_a[(rows_1 + b_a_tmp) - 1] = c_Su1[(75 * i + rows_1) - 1];
      }
    }

    c_Hv_tmp = (kidx + 2) * 3;
    Tries = (kidx + 2) * 75;
    rtDW.c_Hv[(rows[0] + Tries) - 1] = Dv[c_Hv_tmp];
    rtDW.c_Hv[(rows[1] + Tries) - 1] = Dv[c_Hv_tmp + 1];
    rtDW.c_Hv[(rows[2] + Tries) - 1] = Dv[c_Hv_tmp + 2];
    normH = Bv[kidx_tmp];
    s = Bv[kidx_tmp + 1];
    Bv_0 = Bv[kidx_tmp + 2];
    for (i = 0; i < 3; i++) {
      c_Hv_tmp = ((kidx + 1) * 75 + rows[i]) - 1;
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
        c_Hv_tmp = (75 * Tries + rows[i]) - 1;
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
        Tries = (75 * b_a_tmp + rows_2) - 1;
        c_Sx[Tries] = 0.0;
        c_Sx[Tries] += b_A[3 * b_a_tmp] * normH;
        c_Sx[Tries] += b_A[3 * b_a_tmp + 1] * s;
        c_Sx[Tries] += b_A[3 * b_a_tmp + 2] * Bv_0;
      }
    }

    if (kidx + 2 < 25) {
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

  for (i = 0; i < 24; i++) {
    for (b_a_tmp = 0; b_a_tmp < 75; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 100; kidx++) {
        normH += rtDW.b_a[75 * kidx + b_a_tmp] * b_Jm[100 * i + kidx];
      }

      rtDW.c_SuJm[b_a_tmp + 75 * i] = normH;
    }
  }

  if (b_Mrows[0] > 0) {
    kidx = 0;
    exitg1 = false;
    while ((!exitg1) && (kidx < 96)) {
      if (b_Mrows[kidx] <= 75) {
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 24; i++) {
          b_Ac[kidx + 96 * i] = -rtDW.c_SuJm[(75 * i + kidx_tmp) - 1];
        }

        kidx_tmp = b_Mrows[kidx];
        b_Mx[kidx] = -c_Sx[kidx_tmp - 1];
        b_Mx[kidx + 96] = -c_Sx[kidx_tmp + 74];
        b_Mx[kidx + 192] = -c_Sx[kidx_tmp + 149];
        kidx_tmp = b_Mrows[kidx];
        b_Mu1[kidx] = -c_Su1[kidx_tmp - 1];
        b_Mu1[kidx + 96] = -c_Su1[kidx_tmp + 74];
        b_Mu1[kidx + 192] = -c_Su1[kidx_tmp + 149];
        b_Mu1[kidx + 288] = -c_Su1[kidx_tmp + 224];
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 26; i++) {
          b_Mv[kidx + 96 * i] = -rtDW.c_Hv[(75 * i + kidx_tmp) - 1];
        }

        kidx++;
      } else if (b_Mrows[kidx] <= 150) {
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 24; i++) {
          b_Ac[kidx + 96 * i] = rtDW.c_SuJm[(75 * i + kidx_tmp) - 76];
        }

        kidx_tmp = b_Mrows[kidx];
        b_Mx[kidx] = c_Sx[kidx_tmp - 76];
        b_Mx[kidx + 96] = c_Sx[kidx_tmp - 1];
        b_Mx[kidx + 192] = c_Sx[kidx_tmp + 74];
        kidx_tmp = b_Mrows[kidx];
        b_Mu1[kidx] = c_Su1[kidx_tmp - 76];
        b_Mu1[kidx + 96] = c_Su1[kidx_tmp - 1];
        b_Mu1[kidx + 192] = c_Su1[kidx_tmp + 74];
        b_Mu1[kidx + 288] = c_Su1[kidx_tmp + 149];
        kidx_tmp = b_Mrows[kidx];
        for (i = 0; i < 26; i++) {
          b_Mv[kidx + 96 * i] = rtDW.c_Hv[(75 * i + kidx_tmp) - 76];
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
  for (Tries = 0; Tries < 25; Tries++) {
    for (c_Hv_tmp = 0; c_Hv_tmp < 4; c_Hv_tmp++) {
      i = c_Hv_tmp << 2;
      rows_2 = b_B_0[i];
      rows_0 = b_B_0[i + 1];
      rows_1 = b_B_0[i + 2];
      b_B_1 = b_B_0[i + 3];
      for (i = 0; i < 25; i++) {
        c_A = c_A_0[25 * Tries + i];
        rtDW.a[kidx + 1] = (int8_T)(c_A * rows_2);
        rtDW.a[kidx + 2] = (int8_T)(c_A * rows_0);
        rtDW.a[kidx + 3] = (int8_T)(c_A * rows_1);
        rtDW.a[kidx + 4] = (int8_T)(c_A * b_B_1);
        kidx += 4;
      }
    }
  }

  for (i = 0; i < 24; i++) {
    for (b_a_tmp = 0; b_a_tmp < 100; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 100; kidx++) {
        normH += (real_T)rtDW.a[100 * kidx + b_a_tmp] * b_Jm[100 * i + kidx];
      }

      rtDW.I2Jm[b_a_tmp + 100 * i] = normH;
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 75; kidx++) {
    normH = b_Wy[ixw - 1];
    for (i = 0; i < 24; i++) {
      kidx_tmp = 75 * i + kidx;
      rtDW.WySuJm[kidx_tmp] = rtDW.c_SuJm[kidx_tmp] * normH;
    }

    ixw++;
    if (ixw > 3) {
      ixw = 1;
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 100; kidx++) {
    normH = b_Wu[ixw - 1];
    for (i = 0; i < 24; i++) {
      kidx_tmp = 100 * i + kidx;
      rtDW.WuI2Jm[kidx_tmp] = rtDW.I2Jm[kidx_tmp] * normH;
    }

    ixw++;
    if (ixw > 4) {
      ixw = 1;
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 100; kidx++) {
    normH = b_Wdu[ixw - 1];
    for (i = 0; i < 24; i++) {
      kidx_tmp = 100 * i + kidx;
      rtDW.WduJm[kidx_tmp] = b_Jm[kidx_tmp] * normH;
    }

    ixw++;
    if (ixw > 4) {
      ixw = 1;
    }
  }

  for (i = 0; i < 24; i++) {
    for (b_a_tmp = 0; b_a_tmp < 24; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 75; kidx++) {
        normH += rtDW.c_SuJm[75 * i + kidx] * rtDW.WySuJm[75 * b_a_tmp + kidx];
      }

      s = 0.0;
      Bv_0 = 0.0;
      for (kidx = 0; kidx < 100; kidx++) {
        kidx_tmp = 100 * i + kidx;
        Tries = 100 * b_a_tmp + kidx;
        Bv_0 += b_Jm[kidx_tmp] * rtDW.WduJm[Tries];
        s += rtDW.I2Jm[kidx_tmp] * rtDW.WuI2Jm[Tries];
      }

      b_H[i + 25 * b_a_tmp] = (normH + Bv_0) + s;
    }
  }

  for (i = 0; i < 4; i++) {
    for (b_a_tmp = 0; b_a_tmp < 24; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 75; kidx++) {
        normH += c_Su1[75 * i + kidx] * rtDW.WySuJm[75 * b_a_tmp + kidx];
      }

      AA_tmp = (b_a_tmp << 2) + i;
      c_Su1_0[AA_tmp] = normH;
      normH = 0.0;
      for (kidx = 0; kidx < 100; kidx++) {
        normH += b_I1[100 * i + kidx] * rtDW.WuI2Jm[100 * b_a_tmp + kidx];
      }

      b_I1_0[AA_tmp] = normH;
    }
  }

  for (i = 0; i < 96; i++) {
    c_Ku1[i] = c_Su1_0[i] + b_I1_0[i];
  }

  for (i = 0; i < 2400; i++) {
    rtDW.WuI2Jm[i] = -rtDW.WuI2Jm[i];
  }

  for (i = 0; i < 3; i++) {
    for (b_a_tmp = 0; b_a_tmp < 24; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 75; kidx++) {
        normH += c_Sx[75 * i + kidx] * rtDW.WySuJm[75 * b_a_tmp + kidx];
      }

      c_Kx[i + 3 * b_a_tmp] = normH;
    }
  }

  for (i = 0; i < 26; i++) {
    for (b_a_tmp = 0; b_a_tmp < 24; b_a_tmp++) {
      normH = 0.0;
      for (kidx = 0; kidx < 75; kidx++) {
        normH += rtDW.c_Hv[75 * i + kidx] * rtDW.WySuJm[75 * b_a_tmp + kidx];
      }

      rtDW.c_Kv[i + 26 * b_a_tmp] = normH;
    }
  }

  for (i = 0; i < 1800; i++) {
    rtDW.WySuJm[i] = -rtDW.WySuJm[i];
  }

  kidx = 0;
  memcpy(&rtDW.L[0], &b_H[0], 625U * sizeof(real_T));
  Tries = xpotrf(rtDW.L);
  guard1 = false;
  if (Tries == 0) {
    for (Tries = 0; Tries < 25; Tries++) {
      varargin_1[Tries] = rtDW.L[25 * Tries + Tries];
    }

    if (minimum(varargin_1) > 1.4901161193847656E-7) {
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
    while ((!exitg2) && (Tries < 25)) {
      s = 0.0;
      for (c_Hv_tmp = 0; c_Hv_tmp < 25; c_Hv_tmp++) {
        s += fabs(b_H[25 * c_Hv_tmp + Tries]);
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
        memset(&b[0], 0, 625U * sizeof(int8_T));
        for (kidx = 0; kidx < 25; kidx++) {
          b[kidx + 25 * kidx] = 1;
        }

        for (i = 0; i < 625; i++) {
          s = normH * (real_T)b[i] + b_H[i];
          b_H[i] = s;
          rtDW.L[i] = s;
        }

        kidx = xpotrf(rtDW.L);
        guard2 = false;
        if (kidx == 0) {
          for (kidx = 0; kidx < 25; kidx++) {
            varargin_1[kidx] = rtDW.L[25 * kidx + kidx];
          }

          if (minimum(varargin_1) > 1.4901161193847656E-7) {
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
    for (i = 0; i < 26; i++) {
      useq[i] = normH;
      useq[i + 26] = s;
      useq[i + 52] = Bv_0;
      useq[i + 78] = old_u_0;
    }

    *status = -2.0;
  } else {
    memset(&b[0], 0, 625U * sizeof(int8_T));
    for (kidx = 0; kidx < 25; kidx++) {
      b[kidx + 25 * kidx] = 1;
    }

    for (kidx = 0; kidx < 25; kidx++) {
      for (Tries = 0; Tries < 25; Tries++) {
        i = 25 * kidx + Tries;
        b_H[i] = b[i];
      }

      varargin_1[kidx] = 0.0;
    }

    trisolve(rtDW.L, b_H);
    for (kidx = 0; kidx < 24; kidx++) {
      s = 0.0;
      for (i = 0; i < 75; i++) {
        s += rtDW.WySuJm[75 * kidx + i] * rseq[i];
      }

      normH = 0.0;
      for (i = 0; i < 26; i++) {
        normH += rtDW.c_Kv[26 * kidx + i] * vseq[i];
      }

      Bv_0 = 0.0;
      for (i = 0; i < 100; i++) {
        Bv_0 += rtDW.WuI2Jm[100 * kidx + i] * b_utarget[i];
      }

      i = kidx << 2;
      varargin_1[kidx] = (((((c_Kx[3 * kidx + 1] * x[1] + c_Kx[3 * kidx] * x[0])
        + c_Kx[3 * kidx + 2] * x[2]) + s) + (((c_Ku1[i + 1] * old_u[1] + c_Ku1[i]
        * old_u[0]) + c_Ku1[i + 2] * old_u[2]) + c_Ku1[i + 3] * old_u[3])) +
                          normH) + Bv_0;
    }

    normH = x[1];
    s = x[0];
    Bv_0 = x[2];
    old_u_0 = old_u[1];
    old_u_1 = old_u[0];
    old_u_2 = old_u[2];
    old_u_3 = old_u[3];
    for (i = 0; i < 96; i++) {
      iAout[i] = iA[i];
      c_Ku1[i] = ((b_Mx[i + 96] * normH + b_Mx[i] * s) + b_Mx[i + 192] * Bv_0) +
        b_Mlim[i];
      b_I1_0[i] = ((b_Mu1[i + 96] * old_u_0 + b_Mu1[i] * old_u_1) + b_Mu1[i +
                   192] * old_u_2) + b_Mu1[i + 288] * old_u_3;
    }

    for (i = 0; i < 25; i++) {
      for (b_a_tmp = 0; b_a_tmp < 25; b_a_tmp++) {
        s = 0.0;
        for (kidx = 0; kidx < 25; kidx++) {
          s += b_H[25 * i + kidx] * b_H[25 * b_a_tmp + kidx];
        }

        rtDW.L[i + 25 * b_a_tmp] = s;
      }
    }

    for (i = 0; i < 96; i++) {
      s = 0.0;
      for (b_a_tmp = 0; b_a_tmp < 26; b_a_tmp++) {
        s += b_Mv[96 * b_a_tmp + i] * vseq[b_a_tmp];
      }

      c_Su1_0[i] = -((c_Ku1[i] + b_I1_0[i]) + s);
    }

    qpkwik(b_H, rtDW.L, varargin_1, b_Ac, c_Su1_0, iAout, 484, 1.0E-6, zopt,
           c_Ku1, &kidx);
    if ((kidx < 0) || (kidx == 0)) {
      memset(&zopt[0], 0, 25U * sizeof(real_T));
    }

    *status = kidx;
    u[0] = (old_u[0] + zopt[0]) + b_uoff[0];
    u[1] = (old_u[1] + zopt[1]) + b_uoff[1];
    u[2] = (old_u[2] + zopt[2]) + b_uoff[2];
    u[3] = (old_u[3] + zopt[3]) + b_uoff[3];
  }
}

/* Model step function */
void statespace_chaser_test0x2810x29_step(void)
{
  real_T Bu[312];
  real_T Cm[234];
  real_T b_A[234];
  real_T b_utarget[100];
  real_T b_Mlim[96];
  real_T Bv[78];
  real_T Dv[78];
  real_T Dvm[78];
  real_T rseq[75];
  real_T vseq[26];
  real_T Bd[12];
  real_T L[9];
  real_T Nk[9];
  real_T Qk[9];
  real_T Rk[9];
  real_T c_A[9];
  real_T theta_t;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int8_T Ad[9];
  int8_T b_I[9];
  static const real_T b_B_5[624] = { 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0012125000000000003, -0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, 0.0034642857142857145,
    0.0012125000000000003, 0.0012125000000000003, -0.0034642857142857145,
    0.0012125000000000003, -0.0012125000000000003, 0.0034642857142857145, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003,
    -0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, 0.0034642857142857145, 0.0012125000000000003,
    0.0012125000000000003, -0.0034642857142857145, 0.0012125000000000003,
    -0.0012125000000000003, 0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  real_T B_m[312];
  real_T tmp_2[288];
  real_T A[234];
  real_T C[234];
  real_T b_C_tmp[234];
  real_T U[104];
  real_T Cm_0[9];
  real_T c_A_tmp[9];
  real_T rtb_LastPcov[9];
  real_T rtb_LastPcov_0[9];
  real_T U_tmp[4];
  real_T rtb_last_mv_0[4];
  real_T tmp_1[4];
  real_T tmp_3[4];
  real_T rtb_last_x[3];
  real_T rtb_ref[3];
  real_T rtb_ref_0[3];
  real_T Clock;
  real_T Rk_tmp_0;
  real_T Rk_tmp_1;
  real_T Rk_tmp_2;
  real_T b_b;
  real_T c_tmp;
  real_T rtb_last_mv_idx_0;
  real_T rtb_last_mv_idx_1;
  real_T rtb_last_mv_idx_2;
  real_T rtb_last_mv_idx_3;
  real_T s_tmp;
  int32_T tmp[4];
  int32_T tmp_0[3];
  int32_T Bv_tmp;
  int32_T Rk_tmp;
  int32_T i;
  static const real_T b_b_0[12] = { 0.012125, -0.012125, -0.034642857142857142,
    0.012125, 0.012125, 0.034642857142857142, 0.012125, -0.012125,
    0.034642857142857142, 0.012125, 0.012125, -0.034642857142857142 };

  static const int8_T b[234] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0,
    0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0,
    0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1,
    0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0,
    1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0,
    0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0,
    0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1,
    0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1,
    1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0,
    1, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T c[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const real_T d[24] = { 0.0012125000000000003, -0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    0.0034642857142857145, 0.0012125000000000003, 0.0012125000000000003,
    -0.0034642857142857145, 0.0012125000000000003, -0.0012125000000000003,
    0.0034642857142857145, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  static const real_T e[24] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  static const int8_T f[96] = { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };

  static const int8_T b_D[624] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const real_T q[384] = { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T r[625] = { 4794.2895121173451, -4144.1904496173438,
    4144.1904496173438, -3494.3813871173425, 4512.2395408163247,
    -3900.4145408163236, 3900.4145408163236, -3288.8295408163235,
    4231.0972617346924, -3657.3887117346912, 3657.3887117346912,
    -3083.9101617346905, 3951.690367091835, -3415.8630420918344,
    3415.8630420918344, -2880.255717091833, 3674.8865491071419,
    -3176.5876116071408, 3176.5876116071408, -2678.4986741071393,
    3401.5534999999968, -2940.3124999999977, 2940.3124999999977,
    -2479.271499999998, 0.0, -4144.1904496173438, 4794.2895121173451,
    -3494.3813871173425, 4144.1904496173438, -3900.4145408163236,
    4512.2395408163247, -3288.8295408163235, 3900.4145408163236,
    -3657.3887117346912, 4231.0972617346924, -3083.9101617346905,
    3657.3887117346912, -3415.8630420918344, 3951.690367091835,
    -2880.255717091833, 3415.8630420918344, -3176.5876116071408,
    3674.8865491071419, -2678.4986741071393, 3176.5876116071408,
    -2940.3124999999977, 3401.5534999999968, -2479.271499999998,
    2940.3124999999977, 0.0, 4144.1904496173438, -3494.3813871173425,
    4794.2895121173451, -4144.1904496173438, 3900.4145408163236,
    -3288.8295408163235, 4512.2395408163247, -3900.4145408163236,
    3657.3887117346912, -3083.9101617346905, 4231.0972617346924,
    -3657.3887117346912, 3415.8630420918344, -2880.255717091833,
    3951.690367091835, -3415.8630420918344, 3176.5876116071408,
    -2678.4986741071393, 3674.8865491071419, -3176.5876116071408,
    2940.3124999999977, -2479.271499999998, 3401.5534999999968,
    -2940.3124999999977, 0.0, -3494.3813871173425, 4144.1904496173438,
    -4144.1904496173438, 4794.2895121173451, -3288.8295408163235,
    3900.4145408163236, -3900.4145408163236, 4512.2395408163247,
    -3083.9101617346905, 3657.3887117346912, -3657.3887117346912,
    4231.0972617346924, -2880.255717091833, 3415.8630420918344,
    -3415.8630420918344, 3951.690367091835, -2678.4986741071393,
    3176.5876116071408, -3176.5876116071408, 3674.8865491071419,
    -2479.271499999998, 2940.3124999999977, -2940.3124999999977,
    3401.5534999999968, 0.0, 4512.2395408163247, -3900.414540816324,
    3900.414540816324, -3288.8295408163235, 4251.9718749999984,
    -3675.3906249999973, 3675.3906249999973, -3099.0893749999959,
    3991.6142091836718, -3450.366709183671, 3450.366709183671,
    -2909.3492091836711, 3732.1642355867334, -3226.0928730867327,
    3226.0928730867327, -2720.2415105867321, 3474.44964642857,
    -3003.3191964285693, 3003.3191964285693, -2532.3987464285683,
    3219.33813392857, -2782.7957589285697, 2782.7957589285697,
    -2346.4533839285687, 0.0, -3900.414540816324, 4512.2395408163247,
    -3288.8295408163235, 3900.414540816324, -3675.3906249999973,
    4251.9718749999984, -3099.0893749999959, 3675.3906249999973,
    -3450.366709183671, 3991.6142091836718, -2909.3492091836711,
    3450.366709183671, -3226.0928730867327, 3732.1642355867334,
    -2720.2415105867321, 3226.0928730867327, -3003.3191964285693,
    3474.44964642857, -2532.3987464285683, 3003.3191964285693,
    -2782.7957589285697, 3219.33813392857, -2346.4533839285687,
    2782.7957589285697, 0.0, 3900.414540816324, -3288.8295408163235,
    4512.2395408163247, -3900.414540816324, 3675.3906249999973,
    -3099.0893749999959, 4251.9718749999984, -3675.3906249999973,
    3450.366709183671, -2909.3492091836711, 3991.6142091836718,
    -3450.366709183671, 3226.0928730867327, -2720.2415105867321,
    3732.1642355867334, -3226.0928730867327, 3003.3191964285693,
    -2532.3987464285683, 3474.44964642857, -3003.3191964285693,
    2782.7957589285697, -2346.4533839285687, 3219.33813392857,
    -2782.7957589285697, 0.0, -3288.8295408163235, 3900.414540816324,
    -3900.414540816324, 4512.2395408163247, -3099.0893749999959,
    3675.3906249999973, -3675.3906249999973, 4251.9718749999984,
    -2909.3492091836711, 3450.366709183671, -3450.366709183671,
    3991.6142091836718, -2720.2415105867321, 3226.0928730867327,
    -3226.0928730867327, 3732.1642355867334, -2532.3987464285683,
    3003.3191964285693, -3003.3191964285693, 3474.44964642857,
    -2346.4533839285687, 2782.7957589285697, -2782.7957589285697,
    3219.33813392857, 0.0, 4231.0972617346924, -3657.3887117346912,
    3657.3887117346912, -3083.9101617346905, 3991.6142091836718,
    -3450.3667091836714, 3450.3667091836714, -2909.3492091836711,
    3752.1711566326517, -3243.3447066326507, 3243.3447066326507,
    -2734.78825663265, 3512.6381040816314, -3036.3227040816305,
    3036.3227040816305, -2560.2273040816303, 3274.0127437499991,
    -2830.0507812499982, 2830.0507812499982, -2386.2988187499977,
    3037.1227678571413, -2625.2790178571413, 2625.2790178571413,
    -2213.6352678571407, 0.0, -3657.3887117346912, 4231.0972617346924,
    -3083.9101617346905, 3657.3887117346912, -3450.3667091836714,
    3991.6142091836718, -2909.3492091836711, 3450.3667091836714,
    -3243.3447066326507, 3752.1711566326517, -2734.78825663265,
    3243.3447066326507, -3036.3227040816305, 3512.6381040816314,
    -2560.2273040816303, 3036.3227040816305, -2830.0507812499982,
    3274.0127437499991, -2386.2988187499977, 2830.0507812499982,
    -2625.2790178571413, 3037.1227678571413, -2213.6352678571407,
    2625.2790178571413, 0.0, 3657.3887117346912, -3083.9101617346905,
    4231.0972617346924, -3657.3887117346912, 3450.3667091836714,
    -2909.3492091836711, 3991.6142091836718, -3450.3667091836714,
    3243.3447066326507, -2734.78825663265, 3752.1711566326517,
    -3243.3447066326507, 3036.3227040816305, -2560.2273040816303,
    3512.6381040816314, -3036.3227040816305, 2830.0507812499982,
    -2386.2988187499977, 3274.0127437499991, -2830.0507812499982,
    2625.2790178571413, -2213.6352678571407, 3037.1227678571413,
    -2625.2790178571413, 0.0, -3083.9101617346905, 3657.3887117346912,
    -3657.3887117346912, 4231.0972617346924, -2909.3492091836711,
    3450.3667091836714, -3450.3667091836714, 3991.6142091836718,
    -2734.78825663265, 3243.3447066326507, -3243.3447066326507,
    3752.1711566326517, -2560.2273040816303, 3036.3227040816305,
    -3036.3227040816305, 3512.6381040816314, -2386.2988187499977,
    2830.0507812499982, -2830.0507812499982, 3274.0127437499991,
    -2213.6352678571407, 2625.2790178571413, -2625.2790178571413,
    3037.1227678571413, 0.0, 3951.6903670918355, -3415.8630420918348,
    3415.8630420918348, -2880.2557170918335, 3732.1642355867334,
    -3226.0928730867327, 3226.0928730867327, -2720.2415105867321,
    3512.6381040816314, -3036.3227040816309, 3036.3227040816309,
    -2560.2273040816303, 3293.1519725765293, -2846.5525350765288,
    2846.5525350765288, -2400.2130975765281, 3073.5758410714279,
    -2656.7823660714266, 2656.7823660714266, -2240.1988910714263,
    2854.9074017857133, -2467.7622767857129, 2467.7622767857129,
    -2080.8171517857127, 0.0, -3415.8630420918348, 3951.6903670918355,
    -2880.2557170918335, 3415.8630420918348, -3226.0928730867327,
    3732.1642355867334, -2720.2415105867321, 3226.0928730867327,
    -3036.3227040816309, 3512.6381040816314, -2560.2273040816303,
    3036.3227040816309, -2846.5525350765288, 3293.1519725765293,
    -2400.2130975765281, 2846.5525350765288, -2656.7823660714266,
    3073.5758410714279, -2240.1988910714263, 2656.7823660714266,
    -2467.7622767857129, 2854.9074017857133, -2080.8171517857127,
    2467.7622767857129, 0.0, 3415.8630420918348, -2880.2557170918335,
    3951.6903670918355, -3415.8630420918348, 3226.0928730867327,
    -2720.2415105867321, 3732.1642355867334, -3226.0928730867327,
    3036.3227040816309, -2560.2273040816303, 3512.6381040816314,
    -3036.3227040816309, 2846.5525350765288, -2400.2130975765281,
    3293.1519725765293, -2846.5525350765288, 2656.7823660714266,
    -2240.1988910714263, 3073.5758410714279, -2656.7823660714266,
    2467.7622767857129, -2080.8171517857127, 2854.9074017857133,
    -2467.7622767857129, 0.0, -2880.2557170918335, 3415.8630420918348,
    -3415.8630420918348, 3951.6903670918355, -2720.2415105867321,
    3226.0928730867327, -3226.0928730867327, 3732.1642355867334,
    -2560.2273040816303, 3036.3227040816309, -3036.3227040816309,
    3512.6381040816314, -2400.2130975765281, 2846.5525350765288,
    -2846.5525350765288, 3293.1519725765293, -2240.1988910714263,
    2656.7823660714266, -2656.7823660714266, 3073.5758410714279,
    -2080.8171517857127, 2467.7622767857129, -2467.7622767857129,
    2854.9074017857133, 0.0, 3674.8865491071419, -3176.5876116071408,
    3176.5876116071408, -2678.4986741071393, 3474.4496464285703,
    -3003.3191964285697, 3003.3191964285697, -2532.3987464285688,
    3274.0127437499991, -2830.0507812499982, 2830.0507812499982,
    -2386.2988187499977, 3073.5758410714279, -2656.7823660714271,
    2656.7823660714271, -2240.1988910714263, 2873.1789383928558,
    -2483.5139508928555, 2483.5139508928555, -2094.0989633928552,
    2672.6920357142849, -2310.245535714284, 2310.245535714284,
    -1947.9990357142838, 0.0, -3176.5876116071408, 3674.8865491071419,
    -2678.4986741071393, 3176.5876116071408, -3003.3191964285697,
    3474.4496464285703, -2532.3987464285688, 3003.3191964285697,
    -2830.0507812499982, 3274.0127437499991, -2386.2988187499977,
    2830.0507812499982, -2656.7823660714271, 3073.5758410714279,
    -2240.1988910714263, 2656.7823660714271, -2483.5139508928555,
    2873.1789383928558, -2094.0989633928552, 2483.5139508928555,
    -2310.245535714284, 2672.6920357142849, -1947.9990357142838,
    2310.245535714284, 0.0, 3176.5876116071408, -2678.4986741071393,
    3674.8865491071419, -3176.5876116071408, 3003.3191964285697,
    -2532.3987464285688, 3474.4496464285703, -3003.3191964285697,
    2830.0507812499982, -2386.2988187499977, 3274.0127437499991,
    -2830.0507812499982, 2656.7823660714271, -2240.1988910714263,
    3073.5758410714279, -2656.7823660714271, 2483.5139508928555,
    -2094.0989633928552, 2873.1789383928558, -2483.5139508928555,
    2310.245535714284, -1947.9990357142838, 2672.6920357142849,
    -2310.245535714284, 0.0, -2678.4986741071393, 3176.5876116071408,
    -3176.5876116071408, 3674.8865491071419, -2532.3987464285688,
    3003.3191964285697, -3003.3191964285697, 3474.4496464285703,
    -2386.2988187499977, 2830.0507812499982, -2830.0507812499982,
    3274.0127437499991, -2240.1988910714263, 2656.7823660714271,
    -2656.7823660714271, 3073.5758410714279, -2094.0989633928552,
    2483.5139508928555, -2483.5139508928555, 2873.1789383928558,
    -1947.9990357142838, 2310.245535714284, -2310.245535714284,
    2672.6920357142849, 0.0, 3401.5534999999968, -2940.3124999999977,
    2940.3124999999977, -2479.271499999998, 3219.33813392857,
    -2782.7957589285697, 2782.7957589285697, -2346.4533839285687,
    3037.1227678571413, -2625.2790178571413, 2625.2790178571413,
    -2213.6352678571407, 2854.9074017857133, -2467.7622767857129,
    2467.7622767857129, -2080.8171517857127, 2672.6920357142849,
    -2310.2455357142844, 2310.2455357142844, -1947.9990357142838,
    2490.516669642856, -2152.728794642856, 2152.728794642856,
    -1815.1809196428558, 0.0, -2940.3124999999977, 3401.5534999999968,
    -2479.271499999998, 2940.3124999999977, -2782.7957589285697,
    3219.33813392857, -2346.4533839285687, 2782.7957589285697,
    -2625.2790178571413, 3037.1227678571413, -2213.6352678571407,
    2625.2790178571413, -2467.7622767857129, 2854.9074017857133,
    -2080.8171517857127, 2467.7622767857129, -2310.2455357142844,
    2672.6920357142849, -1947.9990357142838, 2310.2455357142844,
    -2152.728794642856, 2490.516669642856, -1815.1809196428558,
    2152.728794642856, 0.0, 2940.3124999999977, -2479.271499999998,
    3401.5534999999968, -2940.3124999999977, 2782.7957589285697,
    -2346.4533839285687, 3219.33813392857, -2782.7957589285697,
    2625.2790178571413, -2213.6352678571407, 3037.1227678571413,
    -2625.2790178571413, 2467.7622767857129, -2080.8171517857127,
    2854.9074017857133, -2467.7622767857129, 2310.2455357142844,
    -1947.9990357142838, 2672.6920357142849, -2310.2455357142844,
    2152.728794642856, -1815.1809196428558, 2490.516669642856,
    -2152.728794642856, 0.0, -2479.271499999998, 2940.3124999999977,
    -2940.3124999999977, 3401.5534999999968, -2346.4533839285687,
    2782.7957589285697, -2782.7957589285697, 3219.33813392857,
    -2213.6352678571407, 2625.2790178571413, -2625.2790178571413,
    3037.1227678571413, -2080.8171517857127, 2467.7622767857129,
    -2467.7622767857129, 2854.9074017857133, -1947.9990357142838,
    2310.2455357142844, -2310.2455357142844, 2672.6920357142849,
    -1815.1809196428558, 2152.728794642856, -2152.728794642856,
    2490.516669642856, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0 };

  static const real_T t[2400] = { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T u[3] = { 40000.0, 40000.0, 62500.0 };

  static const real_T v[2400] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T ab[400] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0 };

  static const int32_T b_Mrows[96] = { 151, 152, 153, 154, 155, 156, 157, 158,
    159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173,
    174, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264,
    265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 351, 352, 353, 354, 355,
    356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370,
    371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385,
    386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398 };

  real_T q_0[384];
  real_T b_B_6[24];
  real_T d_0[24];
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Integrator: '<S4>/Integrator'
   *  MATLAB Function: '<S4>/MATLAB Function'
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
    b_b = b_b_0[3 * r2 + 1];
    Clock = b_b_0[3 * r2];
    theta_t = b_b_0[3 * r2 + 2];
    for (r1 = 0; r1 < 3; r1++) {
      Bd[r1 + 3 * r2] = (Qk[r1 + 3] * b_b + Clock * Qk[r1]) + Qk[r1 + 6] *
        theta_t;
    }
  }

  for (r1 = 0; r1 < 26; r1++) {
    for (r2 = 0; r2 < 3; r2++) {
      i = 3 * r2 + 9 * r1;
      A[i] = Ad[3 * r2];
      A[i + 1] = Ad[3 * r2 + 1];
      A[i + 2] = Ad[3 * r2 + 2];
    }

    for (r2 = 0; r2 < 4; r2++) {
      i = 3 * r2 + 12 * r1;
      B_m[i] = Bd[3 * r2];
      B_m[i + 1] = Bd[3 * r2 + 1];
      B_m[i + 2] = Bd[3 * r2 + 2];
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

  /* Clock: '<Root>/Clock' */
  Clock = rtM->Timing.t[0];
  if (rtmIsMajorTimeStep(rtM)) {
    /* Memory: '<S5>/LastPcov' */
    memcpy(&rtb_LastPcov[0], &rtDW.LastPcov_PreviousInput[0], 9U * sizeof(real_T));

    /* Memory: '<S5>/last_x' */
    rtb_last_x[0] = rtDW.last_x_PreviousInput[0];
    rtb_last_x[1] = rtDW.last_x_PreviousInput[1];
    rtb_last_x[2] = rtDW.last_x_PreviousInput[2];

    /* MATLAB Function: '<Root>/MATLAB Function1' */
    /* PATCHED for real deployment (see rtP_TargetTumbleAngle below): the
     * original generated code assumed a target tumbling at a FIXED, KNOWN
     * rate of 0.1 rad/s from the model's own simulated clock
     * (theta_t = 0.1 * Clock, where Clock = rtM->Timing.t[0]). A real
     * target's tumble rate/phase is unknown ahead of time and must come
     * from the CV pipeline's measured target orientation instead. */
    theta_t = rtP_TargetTumbleAngle;

    /* Approach-distance schedule: unchanged in structure (hold, then ramp
     * down, then hold at final standoff), but the hardcoded times/radii
     * are now parameters (rtP_Approach*) instead of literals, so the
     * approach profile can be tuned without recompiling. Clock here still
     * means elapsed time (rtM->Timing.t[0], set just above), NOT the
     * tumble angle -- this variable is reused/renamed by the generated
     * code's optimizer, unrelated to rtP_TargetTumbleAngle. */
    if (Clock < rtP_ApproachRampStartTime) {
      Clock = rtP_ApproachHoldRadius;
    } else if ((Clock >= rtP_ApproachRampStartTime) &&
               (Clock <= rtP_ApproachRampEndTime)) {
      Clock = rtP_ApproachHoldRadius - (Clock - rtP_ApproachRampStartTime) *
        ((rtP_ApproachHoldRadius - rtP_ApproachFinalRadius) /
         (rtP_ApproachRampEndTime - rtP_ApproachRampStartTime));
    } else {
      Clock = rtP_ApproachFinalRadius;
    }

    /* MATLAB Function: '<S33>/FixedHorizonOptimizer' incorporates:
     *  Integrator: '<S4>/Integrator'
     *  MATLAB Function: '<Root>/MATLAB Function1'
     *  Memory: '<S5>/LastPcov'
     *  Memory: '<S5>/Memory'
     *  UnitDelay: '<S5>/last_mv'
     */
    for (r2 = 0; r2 < 234; r2++) {
      r1 = b[r2];
      b_C_tmp[r2] = r1;
      b_A[r2] = r1;
    }

    memset(&Bu[0], 0, 312U * sizeof(real_T));
    memset(&Bv[0], 0, 78U * sizeof(real_T));
    memset(&Dv[0], 0, 78U * sizeof(real_T));
    memset(&Dvm[0], 0, 78U * sizeof(real_T));
    memset(&Cm[0], 0, 234U * sizeof(real_T));
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
    rtb_ref[0] = 1.0;
    rtb_ref[1] = 1.0;
    rtb_ref[2] = 1.0;
    memcpy(&Bd[0], &(*(real_T (*)[12])&B_m[0])[0], 12U * sizeof(real_T));
    memcpy(&L[0], &(*(real_T (*)[9])&C[0])[0], 9U * sizeof(real_T));
    memcpy(&d_0[0], &d[0], 24U * sizeof(real_T));
    mpc_plantupdate(&A[0], Bd, L, &b_A[0], d_0, &b_C_tmp[0], e, tmp, tmp_0,
                    tmp_1, rtb_ref, &Bu[0], &Bv[0], &Cm[0], &Dv[0], &Dvm[0], Qk,
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
    rtb_ref[0] = 1.0;
    rtb_ref[1] = 1.0;
    rtb_ref[2] = 1.0;
    for (i = 0; i < 25; i++) {
      for (r2 = 0; r2 < 8; r2++) {
        r1 = (i + 1) * 24 + 3 * r2;
        d_0[3 * r2] = b_D[r1];
        d_0[3 * r2 + 1] = b_D[r1 + 1];
        d_0[3 * r2 + 2] = b_D[r1 + 2];
      }

      r2 = (i + 1) * 9;
      r1 = (i + 1) * 12;
      r3 = (i + 1) * 3;
      memcpy(&Bd[0], &(*(real_T (*)[12])&B_m[r1])[0], 12U * sizeof(real_T));
      memcpy(&L[0], &(*(real_T (*)[9])&C[r2])[0], 9U * sizeof(real_T));
      memcpy(&b_B_6[0], &(*(real_T (*)[24])&b_B_5[24 * (i + 1)])[0], 24U *
             sizeof(real_T));
      mpc_plantupdate(&A[r2], Bd, L, &b_A[r2], b_B_6, &b_C_tmp[r2], d_0, tmp,
                      tmp_0, tmp_1, rtb_ref, &Bu[r1], &Bv[r3], &Cm[r2], &Dv[r3],
                      &Dvm[r3], Qk, Rk, Nk);
    }

    for (i = 0; i < 96; i++) {
      b_Mlim[i] = f[i];
    }

    U_tmp[0] = 0.0;
    U_tmp[1] = 0.0;
    U_tmp[2] = 0.0;
    U_tmp[3] = 0.0;
    memset(&b_utarget[0], 0, 100U * sizeof(real_T));
    Bv[0] = 0.0;
    Bv[1] = 0.0;
    Bv[2] = 0.0;
    for (r1 = 0; r1 < 25; r1++) {
      for (r2 = 0; r2 < 3; r2++) {
        i = (r1 + 1) * 12 + r2;
        r3 = (r1 + 1) * 9 + r2;
        Bv_tmp = (r1 + 1) * 3 + r2;
        Bv[Bv_tmp] = (((Bu[i + 3] * 0.0 + Bu[i] * 0.0) + Bu[i + 6] * 0.0) + Bu[i
                      + 9] * 0.0) + ((b_A[r3 + 3] * 0.0 + b_A[r3] * 0.0) +
          b_A[r3 + 6] * 0.0);
        Dv[Bv_tmp] = (b_C_tmp[r3 + 3] * 0.0 + b_C_tmp[r3] * 0.0) + b_C_tmp[r3 +
          6] * 0.0;
      }
    }

    for (i = 0; i < 26; i++) {
      vseq[i] = 1.0;
    }

    /* PATCHED: geometric constants (dock center, arm offset) are now
     * parameters (rtP_Dock*) instead of literals -- same formula
     * structure as generated, tunable without recompiling. theta_t here
     * is the live CV-measured tumble angle (see above), so this docking
     * point now rotates with the REAL target instead of a simulated one. */
    b_b = rtP_DockCenterX - (Clock + rtP_DockArmOffset) * cos(theta_t);
    Clock = rtP_DockCenterY - (Clock + rtP_DockArmOffset) * sin(theta_t);
    for (i = 0; i < 25; i++) {
      rseq[i * ny] = b_b;
      rseq[i * ny + 1] = Clock;
      rseq[i * ny + 2] = theta_t + 3.1415926535897931;
    }

    memset(&L[0], 0, 9U * sizeof(real_T));
    for (i = 0; i < 3; i++) {
      L[i + 3 * i] = 1.0;
    }

    for (r2 = 0; r2 < 3; r2++) {
      b_b = rtb_LastPcov[3 * r2 + 1];
      Clock = rtb_LastPcov[3 * r2];
      theta_t = rtb_LastPcov[3 * r2 + 2];
      for (i = 0; i < 3; i++) {
        r1 = 3 * r2 + i;
        c_A_tmp[r2 + 3 * i] = Cm[r1];
        Cm_0[r1] = (Cm[i + 3] * b_b + Clock * Cm[i]) + Cm[i + 6] * theta_t;
      }
    }

    for (r2 = 0; r2 < 3; r2++) {
      b_b = Cm_0[r2 + 3];
      Clock = Cm_0[r2];
      theta_t = Cm_0[r2 + 6];
      for (r1 = 0; r1 < 3; r1++) {
        i = 3 * r1 + r2;
        c_A[i] = ((c_A_tmp[3 * r1 + 1] * b_b + c_A_tmp[3 * r1] * Clock) +
                  c_A_tmp[3 * r1 + 2] * theta_t) + Rk[i];
      }
    }

    r1 = 0;
    r2 = 1;
    r3 = 2;
    theta_t = fabs(c_A[0]);
    Clock = fabs(c_A[1]);
    if (Clock > theta_t) {
      theta_t = Clock;
      r1 = 1;
      r2 = 0;
    }

    if (fabs(c_A[2]) > theta_t) {
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
    b_b = c_A[r1 + 3];
    Rk[3 * r2] = L[3] - Rk[3 * r1] * b_b;
    Clock = c_A[r1 + 6];
    Rk[3 * r3] = L[6] - Rk[3 * r1] * Clock;
    theta_t = c_A[r2 + 3];
    Rk[3 * r2] /= theta_t;
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
    Rk[Bv_tmp] = L[4] - Rk[i] * b_b;
    Rk_tmp = 3 * r3 + 1;
    Rk[Rk_tmp] = L[7] - Rk[i] * Clock;
    Rk[Bv_tmp] /= theta_t;
    Rk[Rk_tmp] -= Rk[Bv_tmp] * Rk_tmp_0;
    Rk[Rk_tmp] /= Rk_tmp_1;
    Rk[Bv_tmp] -= Rk[Rk_tmp] * Rk_tmp_2;
    Rk[i] -= Rk[Rk_tmp] * c_A[r3];
    Rk[i] -= Rk[Bv_tmp] * c_A[r2];
    i = 3 * r1 + 2;
    Rk[i] = L[2] / c_A[r1];
    Bv_tmp = 3 * r2 + 2;
    Rk[Bv_tmp] = L[5] - Rk[i] * b_b;
    Rk_tmp = 3 * r3 + 2;
    Rk[Rk_tmp] = L[8] - Rk[i] * Clock;
    Rk[Bv_tmp] /= theta_t;
    Rk[Rk_tmp] -= Rk[Bv_tmp] * Rk_tmp_0;
    Rk[Rk_tmp] /= Rk_tmp_1;
    Rk[Bv_tmp] -= Rk[Rk_tmp] * Rk_tmp_2;
    Rk[i] -= Rk[Rk_tmp] * c_A[r3];
    Rk[i] -= Rk[Bv_tmp] * c_A[r2];
    for (r2 = 0; r2 < 3; r2++) {
      b_b = b_A[r2 + 3];
      Clock = b_A[r2];
      theta_t = b_A[r2 + 6];
      for (r1 = 0; r1 < 3; r1++) {
        c_A[r2 + 3 * r1] = (rtb_LastPcov[3 * r1 + 1] * b_b + rtb_LastPcov[3 * r1]
                            * Clock) + rtb_LastPcov[3 * r1 + 2] * theta_t;
      }

      b_b = c_A[r2 + 3];
      Clock = c_A[r2];
      theta_t = c_A[r2 + 6];
      for (r1 = 0; r1 < 3; r1++) {
        i = 3 * r1 + r2;
        Cm_0[i] = ((c_A_tmp[3 * r1 + 1] * b_b + c_A_tmp[3 * r1] * Clock) +
                   c_A_tmp[3 * r1 + 2] * theta_t) + Nk[i];
      }

      Rk_tmp_0 = Cm_0[r2 + 3];
      Rk_tmp_1 = Cm_0[r2];
      Rk_tmp_2 = Cm_0[r2 + 6];
      for (r1 = 0; r1 < 3; r1++) {
        L[r2 + 3 * r1] = (Rk[3 * r1 + 1] * Rk_tmp_0 + Rk[3 * r1] * Rk_tmp_1) +
          Rk[3 * r1 + 2] * Rk_tmp_2;
      }

      rtb_ref[r2] = (((Bu[r2 + 3] * 0.0 + Bu[r2] * 0.0) + Bu[r2 + 6] * 0.0) +
                     Bu[r2 + 9] * 0.0) + rtb_last_x[r2];
    }

    for (r2 = 0; r2 < 3; r2++) {
      Rk_tmp_0 = 0.0;
      b_b = rtb_LastPcov[r2 + 3];
      Clock = rtb_LastPcov[r2];
      theta_t = rtb_LastPcov[r2 + 6];
      for (r1 = 0; r1 < 3; r1++) {
        r3 = 3 * r1 + r2;
        Rk_tmp_0 += Cm[r3] * rtb_ref[r1];
        Nk[r3] = (c_A_tmp[3 * r1 + 1] * b_b + c_A_tmp[3 * r1] * Clock) +
          c_A_tmp[3 * r1 + 2] * theta_t;
      }

      rtb_last_x[r2] = rtX.Integrator_CSTATE[r2] - (Rk_tmp_0 + Dvm[r2]);
      b_b = Nk[r2 + 3];
      Clock = Nk[r2];
      theta_t = Nk[r2 + 6];
      for (r1 = 0; r1 < 3; r1++) {
        rtb_LastPcov_0[r2 + 3 * r1] = (Rk[3 * r1 + 1] * b_b + Rk[3 * r1] * Clock)
          + Rk[3 * r1 + 2] * theta_t;
      }
    }

    theta_t = rtb_last_x[1];
    Rk_tmp_0 = rtb_last_x[0];
    Rk_tmp_1 = rtb_last_x[2];
    for (r2 = 0; r2 < 3; r2++) {
      rtb_ref_0[r2] = ((rtb_LastPcov_0[r2 + 3] * theta_t + rtb_LastPcov_0[r2] *
                        Rk_tmp_0) + rtb_LastPcov_0[r2 + 6] * Rk_tmp_1) +
        rtb_ref[r2];
    }

    rtb_last_mv_0[0] = rtDW.last_mv_DSTATE[0];
    rtb_last_mv_0[1] = rtDW.last_mv_DSTATE[1];
    rtb_last_mv_0[2] = rtDW.last_mv_DSTATE[2];
    rtb_last_mv_0[3] = rtDW.last_mv_DSTATE[3];
    memset(&tmp_2[0], 0, 288U * sizeof(real_T));
    memset(&rtDW.dv[0], 0, 2496U * sizeof(real_T));
    tmp_1[0] = 0.040000000000000008;
    tmp_1[1] = 0.040000000000000008;
    tmp_1[2] = 0.040000000000000008;
    tmp_1[3] = 0.040000000000000008;
    tmp_3[0] = 0.010000000000000002;
    tmp_3[1] = 0.010000000000000002;
    tmp_3[2] = 0.010000000000000002;
    tmp_3[3] = 0.010000000000000002;
    memcpy(&q_0[0], &q[0], 384U * sizeof(real_T));
    memcpy(&rtDW.r[0], &r[0], 625U * sizeof(real_T));
    memcpy(&rtDW.t[0], &t[0], 2400U * sizeof(real_T));
    mpcblock_optimizer(rseq, vseq, rtb_ref_0, rtb_last_mv_0,
                       &rtDW.Memory_PreviousInput[0], b_Mlim, tmp_2, q_0,
                       rtDW.dv, b_utarget, U_tmp, rtDW.r, rtDW.t, u, tmp_1, v,
                       tmp_3, ab, b_A, Bu, Bv, b_C_tmp, Dv, b_Mrows, rtDW.u, U,
                       &theta_t, rtDW.iAout);
    for (r2 = 0; r2 < 3; r2++) {
      Rk_tmp_0 = Cm_0[r2 + 3];
      Rk_tmp_1 = Cm_0[r2];
      Rk_tmp_2 = Cm_0[r2 + 6];
      b_b = c_A[r2 + 3];
      Clock = c_A[r2];
      theta_t = c_A[r2 + 6];
      for (r1 = 0; r1 < 3; r1++) {
        i = 3 * r1 + r2;
        c_A_tmp[i] = (L[r1 + 3] * Rk_tmp_0 + Rk_tmp_1 * L[r1]) + L[r1 + 6] *
          Rk_tmp_2;
        rtb_LastPcov[i] = (b_A[r1 + 3] * b_b + Clock * b_A[r1]) + b_A[r1 + 6] *
          theta_t;
      }
    }

    for (r2 = 0; r2 < 9; r2++) {
      Rk[r2] = (rtb_LastPcov[r2] - c_A_tmp[r2]) + Qk[r2];
    }

    rtb_last_mv_idx_0 = rtDW.u[0];
    rtb_last_mv_idx_1 = rtDW.u[1];
    rtb_last_mv_idx_2 = rtDW.u[2];
    rtb_last_mv_idx_3 = rtDW.u[3];
    b_b = rtb_ref[1];
    Clock = rtb_ref[0];
    Rk_tmp_2 = rtb_ref[2];
    theta_t = rtb_last_x[1];
    Rk_tmp_0 = rtb_last_x[0];
    Rk_tmp_1 = rtb_last_x[2];
    for (r2 = 0; r2 < 3; r2++) {
      rtDW.Pk1[3 * r2] = (Rk[3 * r2] + Rk[r2]) * 0.5;
      r1 = 3 * r2 + 1;
      rtDW.Pk1[r1] = (Rk[r2 + 3] + Rk[r1]) * 0.5;
      r1 = 3 * r2 + 2;
      rtDW.Pk1[r1] = (Rk[r2 + 6] + Rk[r1]) * 0.5;
      rtDW.xk1[r2] = (((((Bu[r2 + 3] * rtb_last_mv_idx_1 + Bu[r2] *
                          rtb_last_mv_idx_0) + Bu[r2 + 6] * rtb_last_mv_idx_2) +
                        Bu[r2 + 9] * rtb_last_mv_idx_3) + ((b_A[r2 + 3] * b_b +
        b_A[r2] * Clock) + b_A[r2 + 6] * Rk_tmp_2)) + Bv[r2]) + ((L[r2 + 3] *
        theta_t + L[r2] * Rk_tmp_0) + L[r2 + 6] * Rk_tmp_1);
    }

    /* End of MATLAB Function: '<S33>/FixedHorizonOptimizer' */
  }

  /* MATLAB Function: '<S4>/MATLAB Function' */
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
    rtb_last_mv_idx_0 = Qk[r2];
    i = (int32_T)Qk[r2 + 6];
    for (r1 = 0; r1 < 4; r1++) {
      c_tmp += ((b_b_0[3 * r1 + 1] * s_tmp + b_b_0[3 * r1] * rtb_last_mv_idx_0)
                + b_b_0[3 * r1 + 2] * (real_T)i) * rtDW.u[r1];
    }

    rtDW.x_dot[r2] = c_tmp;
  }

  if (rtmIsMajorTimeStep(rtM)) {
    if (rtmIsMajorTimeStep(rtM)) {
      /* Update for Memory: '<S5>/LastPcov' */
      memcpy(&rtDW.LastPcov_PreviousInput[0], &rtDW.Pk1[0], 9U * sizeof(real_T));

      /* Update for Memory: '<S5>/Memory' */
      memcpy(&rtDW.Memory_PreviousInput[0], &rtDW.iAout[0], 96U * sizeof
             (boolean_T));

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
void statespace_chaser_test0x2810x29_derivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<S4>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = rtDW.x_dot[0];
  _rtXdot->Integrator_CSTATE[1] = rtDW.x_dot[1];
  _rtXdot->Integrator_CSTATE[2] = rtDW.x_dot[2];
}

/* Model initialize function */
void statespace_chaser_test0x2810x29_initialize(void)
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
  rtM->contStates = ((X *) &rtX);
  rtM->contStateDisabled = ((XDis *) &rtXDis);
  rtM->Timing.tStart = (0.0);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetSolverName(&rtM->solverInfo,"ode3");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.1;

  /* InitializeConditions for Integrator: '<S4>/Integrator' */
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
