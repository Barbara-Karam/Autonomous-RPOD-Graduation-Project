/*
 * File: statespace_chaser_test0x2810x29.h
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

#ifndef statespace_chaser_test0x2810x29_h_
#define statespace_chaser_test0x2810x29_h_
#ifndef statespace_chaser_test0x2810x29_COMMON_INCLUDES_
#define statespace_chaser_test0x2810x29_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                    /* statespace_chaser_test0x2810x29_COMMON_INCLUDES_ */

#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T x_dot[3];                     /* '<S4>/MATLAB Function' */
  real_T xk1[3];                       /* '<S33>/FixedHorizonOptimizer' */
  real_T u[4];                         /* '<S33>/FixedHorizonOptimizer' */
  real_T Pk1[9];                       /* '<S33>/FixedHorizonOptimizer' */
  real_T last_mv_DSTATE[4];            /* '<S5>/last_mv' */
  real_T LastPcov_PreviousInput[9];    /* '<S5>/LastPcov' */
  real_T last_x_PreviousInput[3];      /* '<S5>/last_x' */
  real_T dv[2496];
  real_T r[625];
  real_T t[2400];
  real_T c_SuJm[1800];
  real_T c_Hv[1950];
  real_T c_Kv[624];
  real_T b_a[7500];
  real_T I2Jm[2400];
  real_T WySuJm[1800];
  real_T WuI2Jm[2400];
  real_T WduJm[2400];
  real_T L[625];
  real_T RLinv[625];
  real_T b_D[625];
  real_T b_H[625];
  real_T U[625];
  real_T TL[625];
  real_T R[625];
  real_T b_A[625];
  int8_T a[10000];
  boolean_T iAout[96];                 /* '<S33>/FixedHorizonOptimizer' */
  boolean_T Memory_PreviousInput[96];  /* '<S5>/Memory' */
} DW;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE[3];         /* '<S4>/Integrator' */
} X;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE[3];         /* '<S4>/Integrator' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE[3];      /* '<S4>/Integrator' */
} XDis;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: lastPcov
   * Referenced by: '<S5>/LastPcov'
   */
  real_T LastPcov_InitialCondition[9];
} ConstP;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[3];
  real_T odeF[3][3];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Continuous states (default storage) */
extern X rtX;

/* Disabled states (default storage) */
extern XDis rtXDis;

/* Block signals and states (default storage) */
extern DW rtDW;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void statespace_chaser_test0x2810x29_initialize(void);
extern void statespace_chaser_test0x2810x29_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S5>/Constant' : Unused code path elimination
 * Block '<S5>/Floor' : Unused code path elimination
 * Block '<S5>/Floor1' : Unused code path elimination
 * Block '<S6>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S7>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S8>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S9>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S10>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S11>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S12>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S13>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S14>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S15>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S16>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S17>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S18>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S19>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S20>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S21>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S22>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S23>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S24>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S25>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S26>/Vector Dimension Check' : Unused code path elimination
 * Block '<S27>/Vector Dimension Check' : Unused code path elimination
 * Block '<S28>/Vector Dimension Check' : Unused code path elimination
 * Block '<S29>/Vector Dimension Check' : Unused code path elimination
 * Block '<S30>/Vector Dimension Check' : Unused code path elimination
 * Block '<S31>/Vector Dimension Check' : Unused code path elimination
 * Block '<S5>/Min' : Unused code path elimination
 * Block '<S32>/Vector Dimension Check' : Unused code path elimination
 * Block '<S5>/useq_scale' : Unused code path elimination
 * Block '<S5>/useq_scale1' : Unused code path elimination
 * Block '<S5>/ym_zero' : Unused code path elimination
 * Block '<S1>/m_zero' : Unused code path elimination
 * Block '<S1>/p_zero' : Unused code path elimination
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<Root>/Display1' : Unused code path elimination
 * Block '<S4>/Display' : Unused code path elimination
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<S5>/Reshape' : Reshape block reduction
 * Block '<S5>/Reshape1' : Reshape block reduction
 * Block '<S5>/Reshape2' : Reshape block reduction
 * Block '<S5>/Reshape3' : Reshape block reduction
 * Block '<S5>/Reshape4' : Reshape block reduction
 * Block '<S5>/Reshape5' : Reshape block reduction
 * Block '<S5>/ext.mv_scale' : Eliminated nontunable gain of 1
 * Block '<S5>/u_scale' : Eliminated nontunable gain of 1
 * Block '<S5>/umin_scale4' : Eliminated nontunable gain of 1
 * Block '<S5>/uref_scale' : Eliminated nontunable gain of 1
 * Block '<S5>/ymin_scale1' : Eliminated nontunable gain of 1
 * Block '<S5>/ymin_scale2' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'statespace_chaser_test0x2810x29'
 * '<S1>'   : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller'
 * '<S2>'   : 'statespace_chaser_test0x2810x29/MATLAB Function'
 * '<S3>'   : 'statespace_chaser_test0x2810x29/MATLAB Function1'
 * '<S4>'   : 'statespace_chaser_test0x2810x29/Plant'
 * '<S5>'   : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC'
 * '<S6>'   : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S7>'   : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check A'
 * '<S8>'   : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check B'
 * '<S9>'   : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check C'
 * '<S10>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check D'
 * '<S11>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check DX'
 * '<S12>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check U'
 * '<S13>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check X'
 * '<S14>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check Y'
 * '<S15>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S16>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S17>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check'
 * '<S18>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S19>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S20>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S21>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S22>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S23>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S24>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S25>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S26>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S27>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S28>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S29>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Vector Signal Check'
 * '<S30>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S31>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S32>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/moorx'
 * '<S33>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/optimizer'
 * '<S34>'  : 'statespace_chaser_test0x2810x29/Adaptive MPC Controller/MPC/optimizer/FixedHorizonOptimizer'
 * '<S35>'  : 'statespace_chaser_test0x2810x29/Plant/MATLAB Function'
 */
#endif                                 /* statespace_chaser_test0x2810x29_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
