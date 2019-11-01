/*
 * sldemo_absbrake.c
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

#include <math.h>
#include "sldemo_absbrake.h"
#include "sldemo_absbrake_private.h"
#include "sldemo_absbrake_dt.h"

/* user code (top of parameter file) */
const int_T gblNumToFiles = 0;
const int_T gblNumFrFiles = 0;
const int_T gblNumFrWksBlocks = 0;
const char *gblSlvrJacPatternFileName =
  "sldemo_absbrake_rsim_rtw\\sldemo_absbrake_Jpattern.mat";

/* Root inports information  */
const int_T gblNumRootInportBlks = 0;
const int_T gblNumModelInputs = 0;
extern rtInportTUtable *gblInportTUtables;
extern const char *gblInportFileName;
const int_T gblInportDataTypeIdx[] = { -1 };

const int_T gblInportDims[] = { -1 } ;

const int_T gblInportComplex[] = { -1 };

const int_T gblInportInterpoFlag[] = { -1 };

const int_T gblInportContinuous[] = { -1 };

#include "simstruc.h"
#include "fixedpoint.h"
#include "look1_binlxpw.h"

/* Block signals (default storage) */
B_sldemo_absbrake_T sldemo_absbrake_B;

/* Continuous states */
X_sldemo_absbrake_T sldemo_absbrake_X;

/* Block states (default storage) */
DW_sldemo_absbrake_T sldemo_absbrake_DW;

/* Parent Simstruct */
static SimStruct model_S;
SimStruct *const rtS = &model_S;

/* System initialize for root system: '<Root>' */
void MdlInitialize(void)
{
  /* InitializeConditions for Integrator: '<Root>/Vehicle speed' */
  sldemo_absbrake_X.Vehiclespeed_CSTATE = 88.0;

  /* InitializeConditions for Integrator: '<Root>/Stopping distance' */
  sldemo_absbrake_X.Stoppingdistance_CSTATE = 0.0;

  /* SystemInitialize for ModelReference: '<Root>/Wheel Speed' */
  sldemo_wheelspeed_absbrake_Init(&(sldemo_absbrake_X.WheelSpeed_CSTATE));
}

/* Start for root system: '<Root>' */
void MdlStart(void)
{
  MdlInitialize();
}

/* Outputs for root system: '<Root>' */
void MdlOutputs(int_T tid)
{
  real_T rtb_Vs;

  /* Integrator: '<Root>/Vehicle speed' */
  /* Limited  Integrator  (w/ Saturation Port) */
  if (ssIsMajorTimeStep(rtS)) {
    if (sldemo_absbrake_X.Vehiclespeed_CSTATE >= 1000.0) {
      sldemo_absbrake_X.Vehiclespeed_CSTATE = 1000.0;
      sldemo_absbrake_B.Vehiclespeed_o2 = 1.0;
    } else if (sldemo_absbrake_X.Vehiclespeed_CSTATE <= 0.0) {
      sldemo_absbrake_X.Vehiclespeed_CSTATE = 0.0;
      sldemo_absbrake_B.Vehiclespeed_o2 = -1.0;
    } else {
      sldemo_absbrake_B.Vehiclespeed_o2 = 0.0;
    }
  }

  sldemo_absbrake_B.Vehiclespeed_o1 = sldemo_absbrake_X.Vehiclespeed_CSTATE;

  /* End of Integrator: '<Root>/Vehicle speed' */

  /* Stop: '<Root>/Stop Simulation2' */
  if (ssIsSampleHit(rtS, 1, 0) && (sldemo_absbrake_B.Vehiclespeed_o2 != 0.0)) {
    ssSetStopRequested(rtS, 1);
  }

  /* End of Stop: '<Root>/Stop Simulation2' */

  /* ModelReference: '<Root>/Wheel Speed' */
  sldemo_wheelspeed_absbrake(&(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtm),
    &sldemo_absbrake_B.Ww, &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtb),
    &(sldemo_absbrake_X.WheelSpeed_CSTATE));

  /* Gain: '<Root>/Vehicle speed (angular)' */
  rtb_Vs = 0.8 * sldemo_absbrake_B.Vehiclespeed_o1;

  /* Fcn: '<Root>/Relative Slip' */
  sldemo_absbrake_B.slp = 1.0 - sldemo_absbrake_B.Ww / ((real_T)(rtb_Vs == 0.0) *
    2.2204460492503131e-16 + rtb_Vs);

  /* Gain: '<Root>/Weight' incorporates:
   *  Lookup_n-D: '<Root>/mu-slip friction curve'
   */
  rtb_Vs = 402.25 * look1_binlxpw(sldemo_absbrake_B.slp,
    rtCP_muslipfrictioncurve_bp01Da, rtCP_muslipfrictioncurve_tableD, 20U);

  /* Gain: '<Root>/-1//m' */
  sldemo_absbrake_B.um = -0.02 * rtb_Vs;

  /* Gain: '<Root>/Rr' */
  sldemo_absbrake_B.tiretorque = 1.25 * rtb_Vs;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Constant: '<Root>/Desired relative slip'
   */
  sldemo_absbrake_B.Sum1 = 0.2 - sldemo_absbrake_B.slp;
  UNUSED_PARAMETER(tid);
}

/* Update for root system: '<Root>' */
void MdlUpdate(int_T tid)
{
  /* Update for Integrator: '<Root>/Vehicle speed' */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if (sldemo_absbrake_X.Vehiclespeed_CSTATE == 1000.0) {
    switch (sldemo_absbrake_DW.Vehiclespeed_MODE) {
     case 3:
      if (sldemo_absbrake_B.um < 0.0) {
        ssSetBlockStateForSolverChangedAtMajorStep(rtS);
        sldemo_absbrake_DW.Vehiclespeed_MODE = 1;
      }
      break;

     case 1:
      if (sldemo_absbrake_B.um >= 0.0) {
        sldemo_absbrake_DW.Vehiclespeed_MODE = 3;
        ssSetBlockStateForSolverChangedAtMajorStep(rtS);
      }
      break;

     default:
      ssSetBlockStateForSolverChangedAtMajorStep(rtS);
      if (sldemo_absbrake_B.um < 0.0) {
        sldemo_absbrake_DW.Vehiclespeed_MODE = 1;
      } else {
        sldemo_absbrake_DW.Vehiclespeed_MODE = 3;
      }
      break;
    }
  } else if (sldemo_absbrake_X.Vehiclespeed_CSTATE == 0.0) {
    switch (sldemo_absbrake_DW.Vehiclespeed_MODE) {
     case 4:
      if (sldemo_absbrake_B.um > 0.0) {
        ssSetBlockStateForSolverChangedAtMajorStep(rtS);
        sldemo_absbrake_DW.Vehiclespeed_MODE = 2;
      }
      break;

     case 2:
      if (sldemo_absbrake_B.um <= 0.0) {
        sldemo_absbrake_DW.Vehiclespeed_MODE = 4;
        ssSetBlockStateForSolverChangedAtMajorStep(rtS);
      }
      break;

     default:
      ssSetBlockStateForSolverChangedAtMajorStep(rtS);
      if (sldemo_absbrake_B.um > 0.0) {
        sldemo_absbrake_DW.Vehiclespeed_MODE = 2;
      } else {
        sldemo_absbrake_DW.Vehiclespeed_MODE = 4;
      }
      break;
    }
  } else {
    sldemo_absbrake_DW.Vehiclespeed_MODE = 0;
  }

  /* End of Update for Integrator: '<Root>/Vehicle speed' */

  /* Update for ModelReference: '<Root>/Wheel Speed' */
  sldemo_wheelspeed_absbra_Update
    (&(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtm), &sldemo_absbrake_B.Sum1,
     &sldemo_absbrake_B.tiretorque,
     &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtb),
     &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtdw),
     &(sldemo_absbrake_X.WheelSpeed_CSTATE));
  UNUSED_PARAMETER(tid);
}

/* Derivatives for root system: '<Root>' */
void MdlDerivatives(void)
{
  XDot_sldemo_absbrake_T *_rtXdot;
  _rtXdot = ((XDot_sldemo_absbrake_T *) ssGetdX(rtS));

  /* Derivatives for Integrator: '<Root>/Vehicle speed' */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if ((sldemo_absbrake_DW.Vehiclespeed_MODE != 3) &&
      (sldemo_absbrake_DW.Vehiclespeed_MODE != 4)) {
    _rtXdot->Vehiclespeed_CSTATE = sldemo_absbrake_B.um;
    ((XDis_sldemo_absbrake_T *) ssGetContStateDisabled(rtS))
      ->Vehiclespeed_CSTATE = false;
  } else {
    /* in saturation */
    _rtXdot->Vehiclespeed_CSTATE = 0.0;
    ((XDis_sldemo_absbrake_T *) ssGetContStateDisabled(rtS))
      ->Vehiclespeed_CSTATE = ((sldemo_absbrake_DW.Vehiclespeed_MODE == 3) ||
      (sldemo_absbrake_DW.Vehiclespeed_MODE == 4) || ((XDis_sldemo_absbrake_T *)
      ssGetContStateDisabled(rtS))->Vehiclespeed_CSTATE);
  }

  /* End of Derivatives for Integrator: '<Root>/Vehicle speed' */

  /* Derivatives for ModelReference: '<Root>/Wheel Speed' */
  sldemo_wheelspeed_absbrak_Deriv
    (&(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtm), &sldemo_absbrake_B.Sum1,
     &sldemo_absbrake_B.tiretorque,
     &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtb),
     &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtdw),
     &(sldemo_absbrake_X.WheelSpeed_CSTATE), &(((XDis_sldemo_absbrake_T *)
       ssGetContStateDisabled(rtS))->WheelSpeed_CSTATE),
     &(((XDot_sldemo_absbrake_T *) ssGetdX(rtS))->WheelSpeed_CSTATE));

  /* Derivatives for Integrator: '<Root>/Stopping distance' */
  _rtXdot->Stoppingdistance_CSTATE = sldemo_absbrake_B.Vehiclespeed_o1;
}

/* Projection for root system: '<Root>' */
void MdlProjection(void)
{
}

/* ZeroCrossings for root system: '<Root>' */
void MdlZeroCrossings(void)
{
  /* ZeroCrossings for Integrator: '<Root>/Vehicle speed' */
  /* zero crossings for entering into limited region */
  /* 0: INTG_NORMAL     1: INTG_LEAVING_UPPER_SAT  2: INTG_LEAVING_LOWER_SAT */
  /* 3: INTG_UPPER_SAT  4: INTG_LOWER_SAT */
  if ((sldemo_absbrake_DW.Vehiclespeed_MODE == 1) &&
      (sldemo_absbrake_X.Vehiclespeed_CSTATE >= 1000.0)) {
    ((ZCV_sldemo_absbrake_T *) ssGetSolverZcSignalVector(rtS))
      ->Vehiclespeed_IntgUpLimit_ZC = 0.0;
  } else {
    ((ZCV_sldemo_absbrake_T *) ssGetSolverZcSignalVector(rtS))
      ->Vehiclespeed_IntgUpLimit_ZC = sldemo_absbrake_X.Vehiclespeed_CSTATE -
      1000.0;
  }

  if ((sldemo_absbrake_DW.Vehiclespeed_MODE == 2) &&
      (sldemo_absbrake_X.Vehiclespeed_CSTATE <= 0.0)) {
    ((ZCV_sldemo_absbrake_T *) ssGetSolverZcSignalVector(rtS))
      ->Vehiclespeed_IntgLoLimit_ZC = 0.0;
  } else {
    ((ZCV_sldemo_absbrake_T *) ssGetSolverZcSignalVector(rtS))
      ->Vehiclespeed_IntgLoLimit_ZC = sldemo_absbrake_X.Vehiclespeed_CSTATE;
  }

  /* zero crossings for leaving limited region */
  if ((sldemo_absbrake_DW.Vehiclespeed_MODE == 3) ||
      (sldemo_absbrake_DW.Vehiclespeed_MODE == 4)) {
    ((ZCV_sldemo_absbrake_T *) ssGetSolverZcSignalVector(rtS))
      ->Vehiclespeed_LeaveSaturate_ZC = sldemo_absbrake_B.um;
  } else {
    ((ZCV_sldemo_absbrake_T *) ssGetSolverZcSignalVector(rtS))
      ->Vehiclespeed_LeaveSaturate_ZC = 0.0;
  }

  /* End of ZeroCrossings for Integrator: '<Root>/Vehicle speed' */

  /* ZeroCrossings for ModelReference: '<Root>/Wheel Speed' */
  sldemo_wheelspeed_absbrake_ZC(&(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtm),
    &sldemo_absbrake_B.Sum1, &sldemo_absbrake_B.tiretorque,
    &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtb),
    &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtdw),
    &(sldemo_absbrake_X.WheelSpeed_CSTATE), &(((ZCV_sldemo_absbrake_T *)
    ssGetSolverZcSignalVector(rtS))->WheelSpeed_IntgUpLimit_ZC));
}

/* Termination for root system: '<Root>' */
void MdlTerminate(void)
{
}

/* Function to initialize sizes */
void MdlInitializeSizes(void)
{
  ssSetNumContStates(rtS, 5);          /* Number of continuous states */
  ssSetNumPeriodicContStates(rtS, 0); /* Number of periodic continuous states */
  ssSetNumY(rtS, 0);                   /* Number of model outputs */
  ssSetNumU(rtS, 0);                   /* Number of model inputs */
  ssSetDirectFeedThrough(rtS, 0);      /* The model is not direct feedthrough */
  ssSetNumSampleTimes(rtS, 2);         /* Number of sample times */
  ssSetNumBlocks(rtS, 12);             /* Number of blocks */
  ssSetNumBlockIO(rtS, 7);             /* Number of block outputs */
}

/* Function to initialize sample times. */
void MdlInitializeSampleTimes(void)
{
  /* task periods */
  ssSetSampleTime(rtS, 0, 0.0);
  ssSetSampleTime(rtS, 1, 0.0);

  /* task offsets */
  ssSetOffsetTime(rtS, 0, 0.0);
  ssSetOffsetTime(rtS, 1, 1.0);
}

/* Function to register the model */
/* Turns off all optimizations on Windows because of issues with VC 2015 compiler.
   This function is not performance-critical, hence this is not a problem.
 */
#if defined(_MSC_VER)

#pragma optimize( "", off )

#endif

SimStruct * sldemo_absbrake(void)
{
  static struct _ssMdlInfo mdlInfo;
  (void) memset((char *)rtS, 0,
                sizeof(SimStruct));
  (void) memset((char *)&mdlInfo, 0,
                sizeof(struct _ssMdlInfo));
  ssSetMdlInfoPtr(rtS, &mdlInfo);

  /* timing info */
  {
    static time_T mdlPeriod[NSAMPLE_TIMES];
    static time_T mdlOffset[NSAMPLE_TIMES];
    static time_T mdlTaskTimes[NSAMPLE_TIMES];
    static int_T mdlTsMap[NSAMPLE_TIMES];
    static int_T mdlSampleHits[NSAMPLE_TIMES];
    static boolean_T mdlTNextWasAdjustedPtr[NSAMPLE_TIMES];
    static int_T mdlPerTaskSampleHits[NSAMPLE_TIMES * NSAMPLE_TIMES];
    static time_T mdlTimeOfNextSampleHit[NSAMPLE_TIMES];

    {
      int_T i;
      for (i = 0; i < NSAMPLE_TIMES; i++) {
        mdlPeriod[i] = 0.0;
        mdlOffset[i] = 0.0;
        mdlTaskTimes[i] = 0.0;
        mdlTsMap[i] = i;
        mdlSampleHits[i] = 1;
      }
    }

    ssSetSampleTimePtr(rtS, &mdlPeriod[0]);
    ssSetOffsetTimePtr(rtS, &mdlOffset[0]);
    ssSetSampleTimeTaskIDPtr(rtS, &mdlTsMap[0]);
    ssSetTPtr(rtS, &mdlTaskTimes[0]);
    ssSetSampleHitPtr(rtS, &mdlSampleHits[0]);
    ssSetTNextWasAdjustedPtr(rtS, &mdlTNextWasAdjustedPtr[0]);
    ssSetPerTaskSampleHitsPtr(rtS, &mdlPerTaskSampleHits[0]);
    ssSetTimeOfNextSampleHitPtr(rtS, &mdlTimeOfNextSampleHit[0]);
  }

  ssSetSolverMode(rtS, SOLVER_MODE_SINGLETASKING);

  /*
   * initialize model vectors and cache them in SimStruct
   */

  /* block I/O */
  {
    ssSetBlockIO(rtS, ((void *) &sldemo_absbrake_B));

    {
      sldemo_absbrake_B.Vehiclespeed_o1 = 0.0;
      sldemo_absbrake_B.Vehiclespeed_o2 = 0.0;
      sldemo_absbrake_B.Ww = 0.0;
      sldemo_absbrake_B.slp = 0.0;
      sldemo_absbrake_B.um = 0.0;
      sldemo_absbrake_B.tiretorque = 0.0;
      sldemo_absbrake_B.Sum1 = 0.0;
    }
  }

  /* states (continuous)*/
  {
    real_T *x = (real_T *) &sldemo_absbrake_X;
    ssSetContStates(rtS, x);
    (void) memset((void *)x, 0,
                  sizeof(X_sldemo_absbrake_T));
  }

  /* states (dwork) */
  {
    void *dwork = (void *) &sldemo_absbrake_DW;
    ssSetRootDWork(rtS, dwork);
    (void) memset(dwork, 0,
                  sizeof(DW_sldemo_absbrake_T));
  }

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    ssSetModelMappingInfo(rtS, &dtInfo);
    dtInfo.numDataTypes = 15;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;
  }

  /* Model specific registration */
  ssSetRootSS(rtS, rtS);
  ssSetVersion(rtS, SIMSTRUCT_VERSION_LEVEL2);
  ssSetModelName(rtS, "sldemo_absbrake");
  ssSetPath(rtS, "sldemo_absbrake");
  ssSetTStart(rtS, 0.0);
  ssSetTFinal(rtS, 25.0);

  /* Model Initialize function for ModelReference Block: '<Root>/Wheel Speed' */
  sldemo_wheelspeed_ab_initialize(rtS, 0, 1,
    &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtm),
    &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtb),
    &(sldemo_absbrake_DW.WheelSpeed_InstanceData.rtdw));

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    ssSetRTWLogInfo(rtS, &rt_DataLoggingInfo);
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogXSignalPtrs(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogT(ssGetRTWLogInfo(rtS), "");
    rtliSetLogX(ssGetRTWLogInfo(rtS), "");
    rtliSetLogXFinal(ssGetRTWLogInfo(rtS), "");
    rtliSetLogVarNameModifier(ssGetRTWLogInfo(rtS), "rt_");
    rtliSetLogFormat(ssGetRTWLogInfo(rtS), 0);
    rtliSetLogMaxRows(ssGetRTWLogInfo(rtS), 0);
    rtliSetLogDecimation(ssGetRTWLogInfo(rtS), 1);
    rtliSetLogY(ssGetRTWLogInfo(rtS), "");
    rtliSetLogYSignalInfo(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogYSignalPtrs(ssGetRTWLogInfo(rtS), (NULL));
  }

  {
    static struct _ssStatesInfo2 statesInfo2;
    ssSetStatesInfo2(rtS, &statesInfo2);
  }

  {
    static ssPeriodicStatesInfo periodicStatesInfo;
    ssSetPeriodicStatesInfo(rtS, &periodicStatesInfo);
  }

  {
    static ssSolverInfo slvrInfo;
    static boolean_T contStatesDisabled[5];
    static real_T absTol[5] = { 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6 };

    static uint8_T absTolControl[5] = { 2U, 2U, 2U, 2U, 2U };

    static uint8_T zcAttributes[11] = { (ZC_EVENT_ALL), (ZC_EVENT_ALL),
      (ZC_EVENT_ALL), (ZC_EVENT_ALL_UP), (ZC_EVENT_ALL_DN), (ZC_EVENT_ALL),
      (ZC_EVENT_ALL_UP), (ZC_EVENT_ALL_DN), (ZC_EVENT_ALL), (ZC_EVENT_ALL),
      (ZC_EVENT_ALL) };

    ssSetSolverRelTol(rtS, 0.001);
    ssSetStepSize(rtS, 0.0);
    ssSetMinStepSize(rtS, 0.0);
    ssSetMaxNumMinSteps(rtS, -1);
    ssSetMinStepViolatedError(rtS, 0);
    ssSetMaxStepSize(rtS, 0.01);
    ssSetSolverMaxOrder(rtS, -1);
    ssSetSolverRefineFactor(rtS, 1);
    ssSetOutputTimes(rtS, (NULL));
    ssSetNumOutputTimes(rtS, 0);
    ssSetOutputTimesOnly(rtS, 0);
    ssSetOutputTimesIndex(rtS, 0);
    ssSetZCCacheNeedsReset(rtS, 1);
    ssSetDerivCacheNeedsReset(rtS, 0);
    ssSetNumNonContDerivSigInfos(rtS, 0);
    ssSetNonContDerivSigInfos(rtS, (NULL));
    ssSetSolverInfo(rtS, &slvrInfo);
    ssSetSolverName(rtS, "ode45");
    ssSetVariableStepSolver(rtS, 1);
    ssSetSolverConsistencyChecking(rtS, 0);
    ssSetSolverAdaptiveZcDetection(rtS, 0);
    ssSetSolverRobustResetMethod(rtS, 0);
    ssSetAbsTolVector(rtS, absTol);
    ssSetAbsTolControlVector(rtS, absTolControl);
    ssSetSolverAbsTol_Obsolete(rtS, absTol);
    ssSetSolverAbsTolControl_Obsolete(rtS, absTolControl);
    ssSetSolverStateProjection(rtS, 0);
    ssSetSolverMassMatrixType(rtS, (ssMatrixType)0);
    ssSetSolverMassMatrixNzMax(rtS, 0);
    ssSetModelOutputs(rtS, MdlOutputs);
    ssSetModelLogData(rtS, rt_UpdateTXYLogVars);
    ssSetModelLogDataIfInInterval(rtS, rt_UpdateTXXFYLogVars);
    ssSetModelUpdate(rtS, MdlUpdate);
    ssSetModelDerivatives(rtS, MdlDerivatives);
    ssSetSolverZcSignalAttrib(rtS, zcAttributes);
    ssSetSolverNumZcSignals(rtS, 11);
    ssSetModelZeroCrossings(rtS, MdlZeroCrossings);
    ssSetSolverConsecutiveZCsStepRelTol(rtS, 2.8421709430404007E-13);
    ssSetSolverMaxConsecutiveZCs(rtS, 1000);
    ssSetSolverConsecutiveZCsError(rtS, 2);
    ssSetSolverMaskedZcDiagnostic(rtS, 1);
    ssSetSolverIgnoredZcDiagnostic(rtS, 1);
    ssSetSolverMaxConsecutiveMinStep(rtS, 1);
    ssSetSolverShapePreserveControl(rtS, 2);
    ssSetTNextTid(rtS, INT_MIN);
    ssSetTNext(rtS, rtMinusInf);
    ssSetSolverNeedsReset(rtS);
    ssSetNumNonsampledZCs(rtS, 11);
    ssSetContStateDisabled(rtS, contStatesDisabled);
    ssSetSolverMaxConsecutiveMinStep(rtS, 1);
  }

  ssSetChecksumVal(rtS, 0, 1193497065U);
  ssSetChecksumVal(rtS, 1, 1152612431U);
  ssSetChecksumVal(rtS, 2, 2363878747U);
  ssSetChecksumVal(rtS, 3, 620816562U);
  return rtS;
}

/* When you use the on parameter, it resets the optimizations to those that you
   specified with the /O compiler option. */
#if defined(_MSC_VER)

#pragma optimize( "", on )

#endif
