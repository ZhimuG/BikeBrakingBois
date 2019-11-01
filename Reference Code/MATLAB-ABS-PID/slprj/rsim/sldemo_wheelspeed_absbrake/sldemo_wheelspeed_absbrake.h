/*
 * Code generation for system model 'sldemo_wheelspeed_absbrake'
 * For more details, see corresponding source file sldemo_wheelspeed_absbrake.c
 *
 */

#ifndef RTW_HEADER_sldemo_wheelspeed_absbrake_h_
#define RTW_HEADER_sldemo_wheelspeed_absbrake_h_
#include <string.h>
#include <stddef.h>
#ifndef sldemo_wheelspeed_absbrake_COMMON_INCLUDES_
# define sldemo_wheelspeed_absbrake_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rsim.h"
#endif                         /* sldemo_wheelspeed_absbrake_COMMON_INCLUDES_ */

#include "sldemo_wheelspeed_absbrake_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "model_reference_types.h"

/* Block signals for model 'sldemo_wheelspeed_absbrake' */
typedef struct {
  real_T Brakepressure;                /* '<Root>/Brake pressure' */
  real_T HydraulicLag;                 /* '<Root>/Hydraulic Lag ' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T uI;                           /* '<Root>/1//I' */
  real_T DataTypeConversion1;          /* '<S1>/Data Type Conversion1' */
  real_T DataTypeConversion2;          /* '<S1>/Data Type Conversion2' */
  real_T Sum_o;                        /* '<S1>/Sum' */
  boolean_T RelationalOperator;        /* '<S1>/Relational Operator' */
  boolean_T RelationalOperator1;       /* '<S1>/Relational Operator1' */
} B_sldemo_wheelspeed_absbrak_c_T;

/* Block states (default storage) for model 'sldemo_wheelspeed_absbrake' */
typedef struct {
  int_T Brakepressure_MODE;            /* '<Root>/Brake pressure' */
  int_T WheelSpeed_MODE;               /* '<Root>/Wheel Speed' */
  boolean_T RelationalOperator_Mode;   /* '<S1>/Relational Operator' */
  boolean_T RelationalOperator1_Mode;  /* '<S1>/Relational Operator1' */
} DW_sldemo_wheelspeed_absbra_f_T;

/* Continuous states for model 'sldemo_wheelspeed_absbrake' */
typedef struct {
  real_T Brakepressure_CSTATE;         /* '<Root>/Brake pressure' */
  real_T HydraulicLag_CSTATE;          /* '<Root>/Hydraulic Lag ' */
  real_T WheelSpeed_CSTATE;            /* '<Root>/Wheel Speed' */
} X_sldemo_wheelspeed_absbrak_n_T;

/* State derivatives for model 'sldemo_wheelspeed_absbrake' */
typedef struct {
  real_T Brakepressure_CSTATE;         /* '<Root>/Brake pressure' */
  real_T HydraulicLag_CSTATE;          /* '<Root>/Hydraulic Lag ' */
  real_T WheelSpeed_CSTATE;            /* '<Root>/Wheel Speed' */
} XDot_sldemo_wheelspeed_absb_n_T;

/* State Disabled for model 'sldemo_wheelspeed_absbrake' */
typedef struct {
  boolean_T Brakepressure_CSTATE;      /* '<Root>/Brake pressure' */
  boolean_T HydraulicLag_CSTATE;       /* '<Root>/Hydraulic Lag ' */
  boolean_T WheelSpeed_CSTATE;         /* '<Root>/Wheel Speed' */
} XDis_sldemo_wheelspeed_absb_n_T;

/* Continuous State Absolute Tolerance for model 'sldemo_wheelspeed_absbrake' */
typedef struct {
  real_T Brakepressure_CSTATE;         /* '<Root>/Brake pressure' */
  real_T HydraulicLag_CSTATE;          /* '<Root>/Hydraulic Lag ' */
  real_T WheelSpeed_CSTATE;            /* '<Root>/Wheel Speed' */
} XAbsTol_sldemo_wheelspeed_abs_T;

/* Zero-crossing (trigger) state for model 'sldemo_wheelspeed_absbrake' */
typedef struct {
  real_T Brakepressure_IntgUpLimit_ZC; /* '<Root>/Brake pressure' */
  real_T Brakepressure_IntgLoLimit_ZC; /* '<Root>/Brake pressure' */
  real_T Brakepressure_LeaveSaturate_ZC;/* '<Root>/Brake pressure' */
  real_T WheelSpeed_IntgUpLimit_ZC;    /* '<Root>/Wheel Speed' */
  real_T WheelSpeed_IntgLoLimit_ZC;    /* '<Root>/Wheel Speed' */
  real_T WheelSpeed_LeaveSaturate_ZC;  /* '<Root>/Wheel Speed' */
  real_T RelationalOperator_RelopInput_Z;/* '<S1>/Relational Operator' */
  real_T RelationalOperator1_RelopInput_;/* '<S1>/Relational Operator1' */
} ZCV_sldemo_wheelspeed_absbr_g_T;

/* Real-time Model Data Structure */
struct tag_RTM_sldemo_wheelspeed_abs_T {
  struct SimStruct_tag *rtS;

  /*
   * The following structure contains memory needed to
   * track noncontinuous signals feeding derivative ports.
   */
  struct {
    real_T mr_nonContSig0[1];
  } NonContDerivMemory;

  ssNonContDerivSigInfo nonContDerivSignal[1];
  const rtTimingBridge *timingBridge;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    int_T mdlref_GlobalTID[2];
  } Timing;
};

typedef struct {
  B_sldemo_wheelspeed_absbrak_c_T rtb;
  DW_sldemo_wheelspeed_absbra_f_T rtdw;
  RT_MODEL_sldemo_wheelspeed_ab_T rtm;
} MdlrefDW_sldemo_wheelspeed_ab_T;

/* Model reference registration function */
extern void sldemo_wheelspeed_ab_initialize(SimStruct *const rtS, int_T
  mdlref_TID0, int_T mdlref_TID1, RT_MODEL_sldemo_wheelspeed_ab_T *const
  sldemo_wheelspeed_absbrake_M, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW);
extern void sldemo_wheelspeed_absbrake_Init(X_sldemo_wheelspeed_absbrak_n_T
  *localX);
extern void sldemo_wheelspeed_absbrak_Reset(X_sldemo_wheelspeed_absbrak_n_T
  *localX);
extern void sldemo_wheelspeed_absbrak_Deriv(RT_MODEL_sldemo_wheelspeed_ab_T *
  const sldemo_wheelspeed_absbrake_M, const real_T *rtu_Input, const real_T
  *rtu_TireTorque, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW, X_sldemo_wheelspeed_absbrak_n_T
  *localX, XDis_sldemo_wheelspeed_absb_n_T *localXdis,
  XDot_sldemo_wheelspeed_absb_n_T *localXdot);
extern void sldemo_wheelspeed_absbrake_ZC(RT_MODEL_sldemo_wheelspeed_ab_T *
  const sldemo_wheelspeed_absbrake_M, const real_T *rtu_Input, const real_T
  *rtu_TireTorque, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW, X_sldemo_wheelspeed_absbrak_n_T
  *localX, ZCV_sldemo_wheelspeed_absbr_g_T *localZCSV);
extern void sldemo_wheelspeed_absbra_Update(RT_MODEL_sldemo_wheelspeed_ab_T *
  const sldemo_wheelspeed_absbrake_M, const real_T *rtu_Input, const real_T
  *rtu_TireTorque, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW, X_sldemo_wheelspeed_absbrak_n_T
  *localX);
extern void sldemo_wheelspeed_absbrake(RT_MODEL_sldemo_wheelspeed_ab_T * const
  sldemo_wheelspeed_absbrake_M, real_T *rty_WheelSpeed,
  B_sldemo_wheelspeed_absbrak_c_T *localB, X_sldemo_wheelspeed_absbrak_n_T
  *localX);

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
 * '<Root>' : 'sldemo_wheelspeed_absbrake'
 * '<S1>'   : 'sldemo_wheelspeed_absbrake/Bang-bang controller'
 */
#endif                            /* RTW_HEADER_sldemo_wheelspeed_absbrake_h_ */
