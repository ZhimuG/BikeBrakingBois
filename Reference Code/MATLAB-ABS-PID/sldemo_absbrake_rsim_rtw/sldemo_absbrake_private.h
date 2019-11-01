/*
 * sldemo_absbrake_private.h
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

#ifndef RTW_HEADER_sldemo_absbrake_private_h_
#define RTW_HEADER_sldemo_absbrake_private_h_
#include "rtwtypes.h"
#include "model_reference_types.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(!(ptr)) {\
 ssSetErrorStatus(rtS, RT_MEMORY_ALLOCATION_ERROR);\
 }
#endif

#if !defined(rt_FREE)
#if !defined(_WIN32)
#define rt_FREE(ptr)                   if((ptr) != (NULL)) {\
 free((ptr));\
 (ptr) = (NULL);\
 }
#else

/* Visual and other windows compilers declare free without const */
#define rt_FREE(ptr)                   if((ptr) != (NULL)) {\
 free((void *)(ptr));\
 (ptr) = (NULL);\
 }
#endif
#endif

extern const real_T rtCP_pooled_blQyLhhC4XgN[21];
extern const real_T rtCP_pooled_JTgxNesIBIjv[21];

#define rtCP_muslipfrictioncurve_tableD rtCP_pooled_blQyLhhC4XgN /* Expression: mu
                                                                  * Referenced by: '<Root>/mu-slip friction curve'
                                                                  */
#define rtCP_muslipfrictioncurve_bp01Da rtCP_pooled_JTgxNesIBIjv /* Expression: slip
                                                                  * Referenced by: '<Root>/mu-slip friction curve'
                                                                  */
#if defined(MULTITASKING)
#  error Models using the variable step solvers cannot define MULTITASKING
#endif
#endif                               /* RTW_HEADER_sldemo_absbrake_private_h_ */
