/*
 * Code generation for system model 'sldemo_wheelspeed_absbrake'
 *
 * Model                      : sldemo_wheelspeed_absbrake
 * Model version              : 1.69
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Oct 30 23:00:32 2019
 *
 * Note that the functions contained in this file are part of a Simulink
 * model, and are not self-contained algorithms.
 */

#include "sldemo_wheelspeed_absbrake.h"
#include "sldemo_wheelspeed_absbrake_private.h"

/* System initialize for referenced model: 'sldemo_wheelspeed_absbrake' */
void sldemo_wheelspeed_absbrake_Init(X_sldemo_wheelspeed_absbrak_n_T *localX)
{
  /* InitializeConditions for Integrator: '<Root>/Brake pressure' */
  localX->Brakepressure_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<Root>/Hydraulic Lag ' */
  localX->HydraulicLag_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<Root>/Wheel Speed' */
  localX->WheelSpeed_CSTATE = 70.4;
}

/* System reset for referenced model: 'sldemo_wheelspeed_absbrake' */
void sldemo_wheelspeed_absbrak_Reset(X_sldemo_wheelspeed_absbrak_n_T *localX)
{
  /* InitializeConditions for Integrator: '<Root>/Brake pressure' */
  localX->Brakepressure_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<Root>/Hydraulic Lag ' */
  localX->HydraulicLag_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<Root>/Wheel Speed' */
  localX->WheelSpeed_CSTATE = 70.4;
}

/* Outputs for referenced model: 'sldemo_wheelspeed_absbrake' */
void sldemo_wheelspeed_absbrake(RT_MODEL_sldemo_wheelspeed_ab_T * const
  sldemo_wheelspeed_absbrake_M, real_T *rty_WheelSpeed,
  B_sldemo_wheelspeed_absbrak_c_T *localB, X_sldemo_wheelspeed_absbrak_n_T
  *localX)
{
  /* Integrator: '<Root>/Brake pressure' incorporates:
   *  Integrator: '<Root>/Wheel Speed'
   */
  /* Limited  Integrator  */
  if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M)) {
    if (localX->Brakepressure_CSTATE >= 1500.0) {
      localX->Brakepressure_CSTATE = 1500.0;
    } else {
      if (localX->Brakepressure_CSTATE <= 0.0) {
        localX->Brakepressure_CSTATE = 0.0;
      }
    }

    if (localX->WheelSpeed_CSTATE >= 1000.0) {
      localX->WheelSpeed_CSTATE = 1000.0;
    } else {
      if (localX->WheelSpeed_CSTATE <= 0.0) {
        localX->WheelSpeed_CSTATE = 0.0;
      }
    }
  }

  localB->Brakepressure = localX->Brakepressure_CSTATE;

  /* End of Integrator: '<Root>/Brake pressure' */

  /* TransferFcn: '<Root>/Hydraulic Lag ' */
  localB->HydraulicLag = 0.0;
  localB->HydraulicLag += 10000.0 * localX->HydraulicLag_CSTATE;

  /* Integrator: '<Root>/Wheel Speed' */
  /* Limited  Integrator  */
  *rty_WheelSpeed = localX->WheelSpeed_CSTATE;
}

/* Update for referenced model: 'sldemo_wheelspeed_absbrake' */
void sldemo_wheelspeed_absbra_Update(RT_MODEL_sldemo_wheelspeed_ab_T * const
  sldemo_wheelspeed_absbrake_M, const real_T *rtu_Input, const real_T
  *rtu_TireTorque, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW, X_sldemo_wheelspeed_absbrak_n_T
  *localX)
{
  if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M)) {
    if (memcmp(sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].pCurrVal,
               sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].pPrevVal,
               sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].sizeInBytes)
        != 0) {
      (void) memcpy(sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].pPrevVal,
                    sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].pCurrVal,
                    sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].
                    sizeInBytes);
      ssSetSolverNeedsReset(sldemo_wheelspeed_absbrake_M->rtS);
    }
  }

  /* Sum: '<Root>/Sum' */
  localB->Sum = *rtu_TireTorque - localB->Brakepressure;

  /* Gain: '<Root>/1//I' */
  localB->uI = 0.2 * localB->Sum;
  if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M) && rtmIsSampleHit
      (sldemo_wheelspeed_absbrake_M, 1, 0)) {
    /* RelationalOperator: '<S1>/Relational Operator' incorporates:
     *  Constant: '<S1>/Constant'
     *  RelationalOperator: '<S1>/Relational Operator1'
     */
    if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M)) {
      localDW->RelationalOperator_Mode = (*rtu_Input > 0.0);
      localDW->RelationalOperator1_Mode = (*rtu_Input < 0.0);
    }

    localB->RelationalOperator = localDW->RelationalOperator_Mode;

    /* End of RelationalOperator: '<S1>/Relational Operator' */

    /* DataTypeConversion: '<S1>/Data Type Conversion1' */
    localB->DataTypeConversion1 = localB->RelationalOperator;

    /* RelationalOperator: '<S1>/Relational Operator1' */
    localB->RelationalOperator1 = localDW->RelationalOperator1_Mode;

    /* DataTypeConversion: '<S1>/Data Type Conversion2' */
    localB->DataTypeConversion2 = localB->RelationalOperator1;

    /* Sum: '<S1>/Sum' */
    localB->Sum_o = localB->DataTypeConversion1 - localB->DataTypeConversion2;
  }

  /* Update for Integrator: '<Root>/Brake pressure' */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if (localX->Brakepressure_CSTATE == 1500.0) {
    switch (localDW->Brakepressure_MODE) {
     case 3:
      if (localB->HydraulicLag < 0.0) {
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
        localDW->Brakepressure_MODE = 1;
      }
      break;

     case 1:
      if (localB->HydraulicLag >= 0.0) {
        localDW->Brakepressure_MODE = 3;
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
      }
      break;

     default:
      ssSetBlockStateForSolverChangedAtMajorStep
        (sldemo_wheelspeed_absbrake_M->rtS);
      if (localB->HydraulicLag < 0.0) {
        localDW->Brakepressure_MODE = 1;
      } else {
        localDW->Brakepressure_MODE = 3;
      }
      break;
    }
  } else if (localX->Brakepressure_CSTATE == 0.0) {
    switch (localDW->Brakepressure_MODE) {
     case 4:
      if (localB->HydraulicLag > 0.0) {
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
        localDW->Brakepressure_MODE = 2;
      }
      break;

     case 2:
      if (localB->HydraulicLag <= 0.0) {
        localDW->Brakepressure_MODE = 4;
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
      }
      break;

     default:
      ssSetBlockStateForSolverChangedAtMajorStep
        (sldemo_wheelspeed_absbrake_M->rtS);
      if (localB->HydraulicLag > 0.0) {
        localDW->Brakepressure_MODE = 2;
      } else {
        localDW->Brakepressure_MODE = 4;
      }
      break;
    }
  } else {
    localDW->Brakepressure_MODE = 0;
  }

  /* End of Update for Integrator: '<Root>/Brake pressure' */

  /* Update for Integrator: '<Root>/Wheel Speed' */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if (localX->WheelSpeed_CSTATE == 1000.0) {
    switch (localDW->WheelSpeed_MODE) {
     case 3:
      if (localB->uI < 0.0) {
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
        localDW->WheelSpeed_MODE = 1;
      }
      break;

     case 1:
      if (localB->uI >= 0.0) {
        localDW->WheelSpeed_MODE = 3;
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
      }
      break;

     default:
      ssSetBlockStateForSolverChangedAtMajorStep
        (sldemo_wheelspeed_absbrake_M->rtS);
      if (localB->uI < 0.0) {
        localDW->WheelSpeed_MODE = 1;
      } else {
        localDW->WheelSpeed_MODE = 3;
      }
      break;
    }
  } else if (localX->WheelSpeed_CSTATE == 0.0) {
    switch (localDW->WheelSpeed_MODE) {
     case 4:
      if (localB->uI > 0.0) {
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
        localDW->WheelSpeed_MODE = 2;
      }
      break;

     case 2:
      if (localB->uI <= 0.0) {
        localDW->WheelSpeed_MODE = 4;
        ssSetBlockStateForSolverChangedAtMajorStep
          (sldemo_wheelspeed_absbrake_M->rtS);
      }
      break;

     default:
      ssSetBlockStateForSolverChangedAtMajorStep
        (sldemo_wheelspeed_absbrake_M->rtS);
      if (localB->uI > 0.0) {
        localDW->WheelSpeed_MODE = 2;
      } else {
        localDW->WheelSpeed_MODE = 4;
      }
      break;
    }
  } else {
    localDW->WheelSpeed_MODE = 0;
  }

  /* End of Update for Integrator: '<Root>/Wheel Speed' */
}

/* Derivatives for referenced model: 'sldemo_wheelspeed_absbrake' */
void sldemo_wheelspeed_absbrak_Deriv(RT_MODEL_sldemo_wheelspeed_ab_T * const
  sldemo_wheelspeed_absbrake_M, const real_T *rtu_Input, const real_T
  *rtu_TireTorque, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW, X_sldemo_wheelspeed_absbrak_n_T
  *localX, XDis_sldemo_wheelspeed_absb_n_T *localXdis,
  XDot_sldemo_wheelspeed_absb_n_T *localXdot)
{
  /* Sum: '<Root>/Sum' */
  localB->Sum = *rtu_TireTorque - localB->Brakepressure;

  /* Gain: '<Root>/1//I' */
  localB->uI = 0.2 * localB->Sum;
  if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M) && rtmIsSampleHit
      (sldemo_wheelspeed_absbrake_M, 1, 0)) {
    /* RelationalOperator: '<S1>/Relational Operator' incorporates:
     *  Constant: '<S1>/Constant'
     *  RelationalOperator: '<S1>/Relational Operator1'
     */
    if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M)) {
      localDW->RelationalOperator_Mode = (*rtu_Input > 0.0);
      localDW->RelationalOperator1_Mode = (*rtu_Input < 0.0);
    }

    localB->RelationalOperator = localDW->RelationalOperator_Mode;

    /* End of RelationalOperator: '<S1>/Relational Operator' */

    /* DataTypeConversion: '<S1>/Data Type Conversion1' */
    localB->DataTypeConversion1 = localB->RelationalOperator;

    /* RelationalOperator: '<S1>/Relational Operator1' */
    localB->RelationalOperator1 = localDW->RelationalOperator1_Mode;

    /* DataTypeConversion: '<S1>/Data Type Conversion2' */
    localB->DataTypeConversion2 = localB->RelationalOperator1;

    /* Sum: '<S1>/Sum' */
    localB->Sum_o = localB->DataTypeConversion1 - localB->DataTypeConversion2;
  }

  /* Derivatives for Integrator: '<Root>/Brake pressure' */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if ((localDW->Brakepressure_MODE != 3) && (localDW->Brakepressure_MODE != 4))
  {
    localXdot->Brakepressure_CSTATE = localB->HydraulicLag;
    localXdis->Brakepressure_CSTATE = false;
  } else {
    /* in saturation */
    localXdot->Brakepressure_CSTATE = 0.0;
    localXdis->Brakepressure_CSTATE = ((localDW->Brakepressure_MODE == 3) ||
      (localDW->Brakepressure_MODE == 4) || localXdis->Brakepressure_CSTATE);
  }

  /* End of Derivatives for Integrator: '<Root>/Brake pressure' */

  /* Derivatives for TransferFcn: '<Root>/Hydraulic Lag ' */
  localXdot->HydraulicLag_CSTATE = 0.0;
  localXdot->HydraulicLag_CSTATE += -100.0 * localX->HydraulicLag_CSTATE;
  localXdot->HydraulicLag_CSTATE += localB->Sum_o;

  /* Derivatives for Integrator: '<Root>/Wheel Speed' */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if ((localDW->WheelSpeed_MODE != 3) && (localDW->WheelSpeed_MODE != 4)) {
    localXdot->WheelSpeed_CSTATE = localB->uI;
    localXdis->WheelSpeed_CSTATE = false;
  } else {
    /* in saturation */
    localXdot->WheelSpeed_CSTATE = 0.0;
    localXdis->WheelSpeed_CSTATE = ((localDW->WheelSpeed_MODE == 3) ||
      (localDW->WheelSpeed_MODE == 4) || localXdis->WheelSpeed_CSTATE);
  }

  /* End of Derivatives for Integrator: '<Root>/Wheel Speed' */
}

/* ZeroCrossings for referenced model: 'sldemo_wheelspeed_absbrake' */
void sldemo_wheelspeed_absbrake_ZC(RT_MODEL_sldemo_wheelspeed_ab_T * const
  sldemo_wheelspeed_absbrake_M, const real_T *rtu_Input, const real_T
  *rtu_TireTorque, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW, X_sldemo_wheelspeed_absbrak_n_T
  *localX, ZCV_sldemo_wheelspeed_absbr_g_T *localZCSV)
{
  /* Sum: '<Root>/Sum' */
  localB->Sum = *rtu_TireTorque - localB->Brakepressure;

  /* Gain: '<Root>/1//I' */
  localB->uI = 0.2 * localB->Sum;
  if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M) && rtmIsSampleHit
      (sldemo_wheelspeed_absbrake_M, 1, 0)) {
    /* RelationalOperator: '<S1>/Relational Operator' incorporates:
     *  Constant: '<S1>/Constant'
     *  RelationalOperator: '<S1>/Relational Operator1'
     */
    if (rtmIsMajorTimeStep(sldemo_wheelspeed_absbrake_M)) {
      localDW->RelationalOperator_Mode = (*rtu_Input > 0.0);
      localDW->RelationalOperator1_Mode = (*rtu_Input < 0.0);
    }

    localB->RelationalOperator = localDW->RelationalOperator_Mode;

    /* End of RelationalOperator: '<S1>/Relational Operator' */

    /* DataTypeConversion: '<S1>/Data Type Conversion1' */
    localB->DataTypeConversion1 = localB->RelationalOperator;

    /* RelationalOperator: '<S1>/Relational Operator1' */
    localB->RelationalOperator1 = localDW->RelationalOperator1_Mode;

    /* DataTypeConversion: '<S1>/Data Type Conversion2' */
    localB->DataTypeConversion2 = localB->RelationalOperator1;

    /* Sum: '<S1>/Sum' */
    localB->Sum_o = localB->DataTypeConversion1 - localB->DataTypeConversion2;
  }

  /* ZeroCrossings for RelationalOperator: '<S1>/Relational Operator' */
  localZCSV->RelationalOperator_RelopInput_Z = *rtu_Input;

  /* ZeroCrossings for RelationalOperator: '<S1>/Relational Operator1' */
  localZCSV->RelationalOperator1_RelopInput_ = *rtu_Input;

  /* ZeroCrossings for Integrator: '<Root>/Brake pressure' */
  /* zero crossings for entering into limited region */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if ((localDW->Brakepressure_MODE == 1) && (localX->Brakepressure_CSTATE >=
       1500.0)) {
    localZCSV->Brakepressure_IntgUpLimit_ZC = 0.0;
  } else {
    localZCSV->Brakepressure_IntgUpLimit_ZC = localX->Brakepressure_CSTATE -
      1500.0;
  }

  if ((localDW->Brakepressure_MODE == 2) && (localX->Brakepressure_CSTATE <= 0.0))
  {
    localZCSV->Brakepressure_IntgLoLimit_ZC = 0.0;
  } else {
    localZCSV->Brakepressure_IntgLoLimit_ZC = localX->Brakepressure_CSTATE;
  }

  /* zero crossings for leaving limited region */
  if ((localDW->Brakepressure_MODE == 3) || (localDW->Brakepressure_MODE == 4))
  {
    localZCSV->Brakepressure_LeaveSaturate_ZC = localB->HydraulicLag;
  } else {
    localZCSV->Brakepressure_LeaveSaturate_ZC = 0.0;
  }

  /* End of ZeroCrossings for Integrator: '<Root>/Brake pressure' */

  /* ZeroCrossings for Integrator: '<Root>/Wheel Speed' */
  /* zero crossings for entering into limited region */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if ((localDW->WheelSpeed_MODE == 1) && (localX->WheelSpeed_CSTATE >= 1000.0))
  {
    localZCSV->WheelSpeed_IntgUpLimit_ZC = 0.0;
  } else {
    localZCSV->WheelSpeed_IntgUpLimit_ZC = localX->WheelSpeed_CSTATE - 1000.0;
  }

  if ((localDW->WheelSpeed_MODE == 2) && (localX->WheelSpeed_CSTATE <= 0.0)) {
    localZCSV->WheelSpeed_IntgLoLimit_ZC = 0.0;
  } else {
    localZCSV->WheelSpeed_IntgLoLimit_ZC = localX->WheelSpeed_CSTATE;
  }

  /* zero crossings for leaving limited region */
  if ((localDW->WheelSpeed_MODE == 3) || (localDW->WheelSpeed_MODE == 4)) {
    localZCSV->WheelSpeed_LeaveSaturate_ZC = localB->uI;
  } else {
    localZCSV->WheelSpeed_LeaveSaturate_ZC = 0.0;
  }

  /* End of ZeroCrossings for Integrator: '<Root>/Wheel Speed' */
}

/* Model initialize function */
void sldemo_wheelspeed_ab_initialize(SimStruct *const rtS, int_T mdlref_TID0,
  int_T mdlref_TID1, RT_MODEL_sldemo_wheelspeed_ab_T *const
  sldemo_wheelspeed_absbrake_M, B_sldemo_wheelspeed_absbrak_c_T *localB,
  DW_sldemo_wheelspeed_absbra_f_T *localDW)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)sldemo_wheelspeed_absbrake_M, 0,
                sizeof(RT_MODEL_sldemo_wheelspeed_ab_T));

  /* setup the global timing engine */
  sldemo_wheelspeed_absbrake_M->Timing.mdlref_GlobalTID[0] = mdlref_TID0;
  sldemo_wheelspeed_absbrake_M->Timing.mdlref_GlobalTID[1] = mdlref_TID1;
  sldemo_wheelspeed_absbrake_M->rtS = (rtS);

  /* block I/O */
  (void) memset(((void *) localB), 0,
                sizeof(B_sldemo_wheelspeed_absbrak_c_T));

  {
    localB->Brakepressure = 0.0;
    localB->HydraulicLag = 0.0;
    localB->Sum = 0.0;
    localB->uI = 0.0;
    localB->DataTypeConversion1 = 0.0;
    localB->DataTypeConversion2 = 0.0;
    localB->Sum_o = 0.0;
  }

  /* states (dwork) */
  (void) memset((void *)localDW, 0,
                sizeof(DW_sldemo_wheelspeed_absbra_f_T));
  sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].pPrevVal = (char_T *)
    sldemo_wheelspeed_absbrake_M->NonContDerivMemory.mr_nonContSig0;
  sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].sizeInBytes = (1*sizeof
    (real_T));
  sldemo_wheelspeed_absbrake_M->nonContDerivSignal[0].pCurrVal = (char_T *)
    (&localB->Sum_o);
  ;
}
