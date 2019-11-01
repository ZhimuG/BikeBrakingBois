/*
 * sldemo_absbrake.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "sldemo_absbrake".
 *
 * Model version              : 1.73
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Oct 30 23:00:35 2019
 *
 * Target selection: rsim.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_sldemo_absbrake_h_
#define RTW_HEADER_sldemo_absbrake_h_
#include <stddef.h>
#include <string.h>
#ifndef sldemo_absbrake_COMMON_INCLUDES_
# define sldemo_absbrake_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rsim.h"
#include "rt_logging.h"
#include "dt_info.h"
#endif                                 /* sldemo_absbrake_COMMON_INCLUDES_ */

#include "sldemo_absbrake_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "model_reference_types.h"

/* Child system includes */
#include "sldemo_wheelspeed_absbrake.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME                     sldemo_absbrake
#define NSAMPLE_TIMES                  (2)                       /* Number of sample times */
#define NINPUTS                        (0)                       /* Number of model inputs */
#define NOUTPUTS                       (0)                       /* Number of model outputs */
#define NBLOCKIO                       (7)                       /* Number of data output port signals */
#define NUM_ZC_EVENTS                  (0)                       /* Number of zero-crossing events */
#ifndef NCSTATES
# define NCSTATES                      (5)                       /* Number of continuous states */
#elif NCSTATES != 5
# error Invalid specification of NCSTATES defined in compiler command
#endif

#ifndef rtmGetDataMapInfo
# define rtmGetDataMapInfo(rtm)        (NULL)
#endif

#ifndef rtmSetDataMapInfo
# define rtmSetDataMapInfo(rtm, val)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Vehiclespeed_o1;              /* '<Root>/Vehicle speed' */
  real_T Vehiclespeed_o2;              /* '<Root>/Vehicle speed' */
  real_T Ww;                           /* '<Root>/Wheel Speed' */
  real_T slp;                          /* '<Root>/Relative Slip' */
  real_T um;                           /* '<Root>/-1//m' */
  real_T tiretorque;                   /* '<Root>/Rr' */
  real_T Sum1;                         /* '<Root>/Sum1' */
} B_sldemo_absbrake_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  int_T Vehiclespeed_MODE;             /* '<Root>/Vehicle speed' */
  MdlrefDW_sldemo_wheelspeed_ab_T WheelSpeed_InstanceData;/* '<Root>/Wheel Speed' */
} DW_sldemo_absbrake_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Vehiclespeed_CSTATE;          /* '<Root>/Vehicle speed' */
  X_sldemo_wheelspeed_absbrak_n_T WheelSpeed_CSTATE;/* '<Root>/Wheel Speed' */
  real_T Stoppingdistance_CSTATE;      /* '<Root>/Stopping distance' */
} X_sldemo_absbrake_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Vehiclespeed_CSTATE;          /* '<Root>/Vehicle speed' */
  XDot_sldemo_wheelspeed_absb_n_T WheelSpeed_CSTATE;/* '<Root>/Wheel Speed' */
  real_T Stoppingdistance_CSTATE;      /* '<Root>/Stopping distance' */
} XDot_sldemo_absbrake_T;

/* State disabled  */
typedef struct {
  boolean_T Vehiclespeed_CSTATE;       /* '<Root>/Vehicle speed' */
  XDis_sldemo_wheelspeed_absb_n_T WheelSpeed_CSTATE;/* '<Root>/Wheel Speed' */
  boolean_T Stoppingdistance_CSTATE;   /* '<Root>/Stopping distance' */
} XDis_sldemo_absbrake_T;

/* Continuous State Absolute Tolerance  */
typedef struct {
  real_T Vehiclespeed_CSTATE;          /* '<Root>/Vehicle speed' */
  XAbsTol_sldemo_wheelspeed_abs_T WheelSpeed_CSTATE;/* '<Root>/Wheel Speed' */
  real_T Stoppingdistance_CSTATE;      /* '<Root>/Stopping distance' */
} CStateAbsTol_sldemo_absbrake_T;

/* Zero-crossing (trigger) state */
typedef struct {
  real_T Vehiclespeed_IntgUpLimit_ZC;  /* '<Root>/Vehicle speed' */
  real_T Vehiclespeed_IntgLoLimit_ZC;  /* '<Root>/Vehicle speed' */
  real_T Vehiclespeed_LeaveSaturate_ZC;/* '<Root>/Vehicle speed' */
  ZCV_sldemo_wheelspeed_absbr_g_T WheelSpeed_IntgUpLimit_ZC;/* '<Root>/Wheel Speed' */
} ZCV_sldemo_absbrake_T;

/* External data declarations for dependent source files */
extern const char *RT_MEMORY_ALLOCATION_ERROR;
extern B_sldemo_absbrake_T sldemo_absbrake_B;/* block i/o */
extern X_sldemo_absbrake_T sldemo_absbrake_X;/* states (continuous) */
extern DW_sldemo_absbrake_T sldemo_absbrake_DW;/* states (dwork) */

/* Simulation Structure */
extern SimStruct *const rtS;

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
 * '<Root>' : 'sldemo_absbrake'
 */

/* user code (bottom of header file) */
extern const int_T gblNumToFiles;
extern const int_T gblNumFrFiles;
extern const int_T gblNumFrWksBlocks;
extern rtInportTUtable *gblInportTUtables;
extern const char *gblInportFileName;
extern const int_T gblNumRootInportBlks;
extern const int_T gblNumModelInputs;
extern const int_T gblInportDataTypeIdx[];
extern const int_T gblInportDims[];
extern const int_T gblInportComplex[];
extern const int_T gblInportInterpoFlag[];
extern const int_T gblInportContinuous[];

#endif                                 /* RTW_HEADER_sldemo_absbrake_h_ */
