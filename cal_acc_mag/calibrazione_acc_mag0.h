/*
 * File: calibrazione_acc_mag0.h
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

#ifndef RTW_HEADER_calibrazione_acc_mag0_h_
#define RTW_HEADER_calibrazione_acc_mag0_h_
#ifndef calibrazione_acc_mag0_COMMON_INCLUDES_
# define calibrazione_acc_mag0_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_assert.h"
#include "rt_defines.h"
#endif                                 /* calibrazione_acc_mag0_COMMON_INCLUDES_ */

#include "calibrazione_acc_mag0_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T sp2;                          /* '<S17>/sp[2]' */
  real_T cp2;                          /* '<S17>/cp[2]' */
  real_T cp1;                          /* '<S17>/cp[1]' */
  real_T sp1;                          /* '<S17>/sp[1]' */
  real_T Assignment[11];               /* '<S64>/Assignment' */
  real_T Assignment1[11];              /* '<S64>/Assignment1' */
  real_T sqrt_a;                       /* '<S16>/sqrt' */
  real_T Product11;                    /* '<S56>/Product11' */
  real_T Product4;                     /* '<S57>/Product4' */
  real_T Product12;                    /* '<S62>/Product12' */
  real_T sqrt_n;                       /* '<S63>/sqrt' */
  real_T Merge[2];                     /* '<S29>/Merge' */
  real_T Sum1;                         /* '<S24>/Sum1' */
  real_T Sum2;                         /* '<S24>/Sum2' */
  real_T Sum3;                         /* '<S24>/Sum3' */
  real_T Sum5;                         /* '<S24>/Sum5' */
  real_T Sum2_a[169];                  /* '<S26>/Sum2' */
  real_T Merge_h[2];                   /* '<S25>/Merge' */
  real_T Assignment_n[169];            /* '<S25>/Assignment' */
  real_T Assignment_snorm[169];        /* '<S25>/Assignment_snorm' */
  real_T Product2;                     /* '<S27>/Product2' */
  real_T Assignment2_h[13];            /* '<S32>/Assignment2' */
  real_T Assignment2_h1[13];           /* '<S31>/Assignment2' */
} BlockIO_calibrazione_acc_mag0;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay1_DSTATE[2];         /* '<S64>/Unit Delay1' */
  real_T UnitDelay1_DSTATE_h;          /* '<S24>/Unit Delay1' */
  real_T UnitDelay3_DSTATE;            /* '<S24>/Unit Delay3' */
  real_T UnitDelay2_DSTATE_d;          /* '<S24>/Unit Delay2' */
  real_T UnitDelay4_DSTATE;            /* '<S24>/Unit Delay4' */
  real_T UnitDelay_DSTATE_j[169];      /* '<S26>/Unit Delay' */
  real_T UnitDelay_DSTATE_g[169];      /* '<S53>/Unit Delay' */
  real_T UnitDelay_DSTATE_k[169];      /* '<S25>/Unit Delay' */
  real_T UnitDelay1_DSTATE_g[169];     /* '<S25>/Unit Delay1' */
  real_T UnitDelay1_DSTATE_i[13];      /* '<S27>/Unit Delay1' */
  real_T otime_PreviousInput;          /* '<S21>/otime' */
  real_T olon_PreviousInput;           /* '<S20>/olon' */
  real_T olat_PreviousInput;           /* '<S19>/olat' */
  real_T oalt_PreviousInput;           /* '<S19>/oalt' */
  uint8_T ForIterator_IterationMarker[7];/* '<S23>/For Iterator' */
  boolean_T SpecialcaseNorthSouthGeographic;/* '<S24>/Special case - North//South Geographic Pole' */
} D_Work_calibrazione_acc_mag0;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T position[3];                  /* '<Root>/position' */
  real_T w1[4];                        /* '<Root>/w1' */
  real_T w2[4];                        /* '<Root>/w2' */
  real_T w3[4];                        /* '<Root>/w3' */
  real_T w4[4];                        /* '<Root>/w4' */
  real_T w5[4];                        /* '<Root>/w5' */
  real_T w6[4];                        /* '<Root>/w6' */
  real_T mag_x[168];                   /* '<Root>/mag_x' */
  real_T mag_y[168];                   /* '<Root>/mag_y' */
  real_T mag_z[168];                   /* '<Root>/mag_z' */
} ExternalInputs_calibrazione_acc;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T G_acc[9];                     /* '<Root>/G_acc' */
  real_T Bias_acc[3];                  /* '<Root>/Bias_acc' */
  real_T G_mag[9];                     /* '<Root>/G_mag' */
  real_T Bias_mag[3];                  /* '<Root>/Bias_mag' */
} ExternalOutputs_calibrazione_ac;

/* Parameters (auto storage) */
struct Parameters_calibrazione_acc_mag_ {
  real_T Constant_Value;               /* Expression: 1
                                        * Referenced by: '<S31>/Constant'
                                        */
  real_T k1313_Value[169];             /* Expression: k
                                        * Referenced by: '<S32>/k[13][13]'
                                        */
  real_T bpp_Y0;                       /* Expression: 0
                                        * Referenced by: '<S27>/bpp'
                                        */
  real_T Constant_Value_b;             /* Expression: fm(2)
                                        * Referenced by: '<S27>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 1
                                        * Referenced by: '<S27>/Constant1'
                                        */
  real_T UnitDelay1_X0[13];            /* Expression: ones(1,maxdef+1)
                                        * Referenced by: '<S27>/Unit Delay1'
                                        */
  real_T Constant_Value_g;             /* Expression: 1
                                        * Referenced by: '<S35>/Constant'
                                        */
  real_T Constant_Value_o;             /* Expression: 1
                                        * Referenced by: '<S37>/Constant'
                                        */
  real_T Constant_Value_oi;            /* Expression: 1
                                        * Referenced by: '<S38>/Constant'
                                        */
  real_T Constant1_Value_b;            /* Expression: 0
                                        * Referenced by: '<S41>/Constant1'
                                        */
  real_T Constant_Value_bn;            /* Expression: 0
                                        * Referenced by: '<S41>/Constant'
                                        */
  real_T Switch_Threshold;             /* Expression: 0.5
                                        * Referenced by: '<S41>/Switch'
                                        */
  real_T k1313_Value_b[169];           /* Expression: k
                                        * Referenced by: '<S41>/k[13][13]'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S41>/Switch1'
                                        */
  real_T UnitDelay_X0[169];            /* Expression: zeros(maxdef+1,maxdef+1)
                                        * Referenced by: '<S25>/Unit Delay'
                                        */
  real_T UnitDelay1_X0_d[169];         /* Expression: snorm
                                        * Referenced by: '<S25>/Unit Delay1'
                                        */
  real_T zerosmaxdef1maxdef1_Value[169];/* Expression: zeros(maxdef+1,maxdef+1)
                                         * Referenced by: '<S53>/zeros(maxdef+1,maxdef+1)'
                                         */
  real_T Gain_Gain;                    /* Expression: 1
                                        * Referenced by: '<S54>/Gain'
                                        */
  real_T UnitDelay_X0_g[169];          /* Expression: zeros(maxdef+1,maxdef+1)
                                        * Referenced by: '<S26>/Unit Delay'
                                        */
  real_T cmaxdefmaxdef_Value[169];     /* Expression: c
                                        * Referenced by: '<S26>/c[maxdef][maxdef]'
                                        */
  real_T cdmaxdefmaxdef_Value[169];    /* Expression: cd
                                        * Referenced by: '<S26>/cd[maxdef][maxdef]'
                                        */
  real_T UnitDelay_X0_o[169];          /* Expression: zeros(maxdef+1,maxdef+1)
                                        * Referenced by: '<S53>/Unit Delay'
                                        */
  real_T Constant1_Value_i;            /* Expression: 1
                                        * Referenced by: '<S29>/Constant1'
                                        */
  real_T fm_Value[13];                 /* Expression: fm
                                        * Referenced by: '<S24>/fm'
                                        */
  real_T fn_Value[13];                 /* Expression: fn
                                        * Referenced by: '<S24>/fn'
                                        */
  real_T Constant1_Value_f;            /* Expression: 0
                                        * Referenced by: '<S30>/Constant1'
                                        */
  real_T UnitDelay1_X0_h;              /* Expression: 0
                                        * Referenced by: '<S24>/Unit Delay1'
                                        */
  real_T UnitDelay3_X0;                /* Expression: 0
                                        * Referenced by: '<S24>/Unit Delay3'
                                        */
  real_T UnitDelay2_X0;                /* Expression: 0
                                        * Referenced by: '<S24>/Unit Delay2'
                                        */
  real_T UnitDelay4_X0;                /* Expression: 0
                                        * Referenced by: '<S24>/Unit Delay4'
                                        */
  real_T UnitDelay_X0_b;               /* Expression: 0
                                        * Referenced by: '<S15>/Unit Delay'
                                        */
  real_T UnitDelay2_X0_a[4];           /* Expression: [0 0 0 0]
                                        * Referenced by: '<S15>/Unit Delay2'
                                        */
  real_T b_Value;                      /* Expression: 6356.7523142
                                        * Referenced by: '<S16>/b'
                                        */
  real_T a_Value;                      /* Expression: 6378.137
                                        * Referenced by: '<S16>/a'
                                        */
  real_T Gain_Gain_a;                  /* Expression: 2
                                        * Referenced by: '<S61>/Gain'
                                        */
  real_T Constant_Value_a;             /* Expression: 1
                                        * Referenced by: '<S63>/Constant'
                                        */
  real_T Constant_Value_j[11];         /* Expression: [1:maxdef-1]
                                        * Referenced by: '<S64>/Constant'
                                        */
  real_T UnitDelay1_X0_k;              /* Expression: 0
                                        * Referenced by: '<S64>/Unit Delay1'
                                        */
  real_T ForIterator_IterationLimit;   /* Expression: maxdef-1
                                        * Referenced by: '<S64>/For Iterator'
                                        */
  real_T cpm1spm1_Threshold;           /* Expression: 1
                                        * Referenced by: '<S64>/cp[m-1] sp[m-1]'
                                        */
  real_T Constant1_Value_fr[11];       /* Expression: [1:maxdef-1]
                                        * Referenced by: '<S64>/Constant1'
                                        */
  real_T sp13_Y0[13];                  /* Expression: [ zeros(1,13) ]
                                        * Referenced by: '<S17>/sp[13]'
                                        */
  real_T cp13_Y0[13];                  /* Expression: [ zeros(1,13) ]
                                        * Referenced by: '<S17>/cp[13]'
                                        */
  real_T cp1_Value;                    /* Expression: 1
                                        * Referenced by: '<S17>/cp[1]'
                                        */
  real_T sp1_Value;                    /* Expression: 0
                                        * Referenced by: '<S17>/sp[1]'
                                        */
  real_T GravityatEquator_Value;       /* Expression: 9.7803267714
                                        * Referenced by: '<S2>/Gravity at Equator'
                                        */
  real_T One_Value;                    /* Expression: 1
                                        * Referenced by: '<S2>/One'
                                        */
  real_T Gravityformulaconst_Value;    /* Expression: 0.00193185138639
                                        * Referenced by: '<S2>/Gravity formula const'
                                        */
  real_T FirstEccentricity_Value;      /* Expression: 0.0818191908426
                                        * Referenced by: '<S2>/First Eccentricity'
                                        */
  real_T u_Value;                      /* Expression: 1/2
                                        * Referenced by: '<S2>/1//2'
                                        */
  real_T epoch_Value;                  /* Expression: epoch
                                        * Referenced by: '<S13>/epoch'
                                        */
  real_T Date_Value[3];                /* Expression: simdate
                                        * Referenced by: '<S3>/Date'
                                        */
  real_T DayToYear_Gain;               /* Expression: 1/365
                                        * Referenced by: '<S3>/DayToYear'
                                        */
  real_T MonthToYear_Gain;             /* Expression: 1/12
                                        * Referenced by: '<S3>/MonthToYear'
                                        */
  real_T otime_X0;                     /* Expression: -1000
                                        * Referenced by: '<S21>/otime'
                                        */
  real_T u80deg_UpperSat;              /* Expression: 180.0
                                        * Referenced by: '<S6>/+//- 180 deg'
                                        */
  real_T u80deg_LowerSat;              /* Expression: -180.0
                                        * Referenced by: '<S6>/+//- 180 deg'
                                        */
  real_T u0deg_UpperSat;               /* Expression: 90.0
                                        * Referenced by: '<S6>/+//- 90 deg'
                                        */
  real_T u0deg_LowerSat;               /* Expression: -90.0
                                        * Referenced by: '<S6>/+//- 90 deg'
                                        */
  real_T UnitConversion_Gain;          /* Expression: slope
                                        * Referenced by: '<S65>/Unit Conversion'
                                        */
  real_T olon_X0;                      /* Expression: -1000
                                        * Referenced by: '<S20>/olon'
                                        */
  real_T olat_X0;                      /* Expression: -1000
                                        * Referenced by: '<S19>/olat'
                                        */
  real_T UnitConversion_Gain_a;        /* Expression: slope
                                        * Referenced by: '<S12>/Unit Conversion'
                                        */
  real_T to1000000m_UpperSat;          /* Expression: 1000000.0
                                        * Referenced by: '<S6>/0 to 1,000,000 m'
                                        */
  real_T to1000000m_LowerSat;          /* Expression: 0
                                        * Referenced by: '<S6>/0 to 1,000,000 m'
                                        */
  real_T Gain_Gain_e;                  /* Expression: 0.001
                                        * Referenced by: '<S6>/Gain'
                                        */
  real_T oalt_X0;                      /* Expression: -1000
                                        * Referenced by: '<S19>/oalt'
                                        */
  real_T re_Value;                     /* Expression: 6371.2
                                        * Referenced by: '<S13>/re'
                                        */
  real_T UnitConversion_Gain_k;        /* Expression: slope
                                        * Referenced by: '<S70>/Unit Conversion'
                                        */
  real_T UnitConversion_Gain_kp;       /* Expression: slope
                                        * Referenced by: '<S71>/Unit Conversion'
                                        */
  real_T UnitConversion_Gain_d;        /* Expression: slope
                                        * Referenced by: '<S14>/Unit Conversion'
                                        */
  real_T PowerConversion_Gain;         /* Expression: 1.0
                                        * Referenced by: '<S6>/Power Conversion'
                                        */
  real_T Gain1_Gain;                   /* Expression: 0.00001
                                        * Referenced by: '<S3>/Gain1'
                                        */
  real_T Gain1_Gain_o;                 /* Expression: 0.00001
                                        * Referenced by: '<S1>/Gain1'
                                        */
  real_T min_val_Value;                /* Expression: min
                                        * Referenced by: '<S7>/min_val'
                                        */
  real_T max_val_Value;                /* Expression: max
                                        * Referenced by: '<S7>/max_val'
                                        */
  real_T min_val_Value_e;              /* Expression: min
                                        * Referenced by: '<S8>/min_val'
                                        */
  real_T max_val_Value_g;              /* Expression: max
                                        * Referenced by: '<S8>/max_val'
                                        */
  real_T min_val_Value_eh;             /* Expression: min
                                        * Referenced by: '<S9>/min_val'
                                        */
  real_T max_val_Value_k;              /* Expression: max
                                        * Referenced by: '<S9>/max_val'
                                        */
  real_T min_val_Value_c;              /* Expression: min
                                        * Referenced by: '<S11>/min_val'
                                        */
  real_T max_val_Value_gm;             /* Expression: max
                                        * Referenced by: '<S11>/max_val'
                                        */
  int32_T Constant1_Value_b2;          /* Computed Parameter: Constant1_Value_b2
                                        * Referenced by: '<S33>/Constant1'
                                        */
  int32_T Constant_Value_b2;           /* Computed Parameter: Constant_Value_b2
                                        * Referenced by: '<S34>/Constant'
                                        */
  int32_T Constant_Value_b2j;          /* Computed Parameter: Constant_Value_b2j
                                        * Referenced by: '<S32>/Constant'
                                        */
  int32_T Constant_Value_bn4;          /* Computed Parameter: Constant_Value_bn4
                                        * Referenced by: '<S43>/Constant'
                                        */
  int32_T Gain_Gain_b;                 /* Computed Parameter: Gain_Gain_b
                                        * Referenced by: '<S43>/Gain'
                                        */
  int32_T Constant_Value_bn44;         /* Computed Parameter: Constant_Value_bn44
                                        * Referenced by: '<S46>/Constant'
                                        */
  int32_T Gain_Gain_bn;                /* Computed Parameter: Gain_Gain_bn
                                        * Referenced by: '<S45>/Gain'
                                        */
  int32_T Constant_Value_bn44a;        /* Computed Parameter: Constant_Value_bn44a
                                        * Referenced by: '<S49>/Constant'
                                        */
  int32_T Constant1_Value_bn;          /* Computed Parameter: Constant1_Value_bn
                                        * Referenced by: '<S49>/Constant1'
                                        */
  int32_T Constant1_Value_bn4;         /* Computed Parameter: Constant1_Value_bn4
                                        * Referenced by: '<S50>/Constant1'
                                        */
  int32_T Constant_Value_bn44aw;       /* Computed Parameter: Constant_Value_bn44aw
                                        * Referenced by: '<S48>/Constant'
                                        */
  int32_T Constant1_Value_bn44;        /* Computed Parameter: Constant1_Value_bn44
                                        * Referenced by: '<S47>/Constant1'
                                        */
  int32_T Gain_Gain_bn4;               /* Computed Parameter: Gain_Gain_bn4
                                        * Referenced by: '<S47>/Gain'
                                        */
  int32_T Constant1_Value_bn44a;       /* Computed Parameter: Constant1_Value_bn44a
                                        * Referenced by: '<S51>/Constant1'
                                        */
  int32_T Constant_Value_bd;           /* Computed Parameter: Constant_Value_bd
                                        * Referenced by: '<S25>/Constant'
                                        */
  int32_T Constant_Value_e;            /* Computed Parameter: Constant_Value_e
                                        * Referenced by: '<S42>/Constant'
                                        */
  int32_T Gain_Gain_eu;                /* Computed Parameter: Gain_Gain_eu
                                        * Referenced by: '<S42>/Gain'
                                        */
  int32_T Constant_Value_n;            /* Computed Parameter: Constant_Value_n
                                        * Referenced by: '<S52>/Constant'
                                        */
  int32_T Constant1_Value_n;           /* Computed Parameter: Constant1_Value_n
                                        * Referenced by: '<S52>/Constant1'
                                        */
  int32_T Constant_Value_bj;           /* Computed Parameter: Constant_Value_bj
                                        * Referenced by: '<S54>/Constant'
                                        */
  int32_T tc_old_Threshold;            /* Computed Parameter: tc_old_Threshold
                                        * Referenced by: '<S53>/tc_old'
                                        */
  int32_T Constant_Value_i;            /* Computed Parameter: Constant_Value_i
                                        * Referenced by: '<S24>/Constant'
                                        */
  int32_T Constant1_Value_k;           /* Computed Parameter: Constant1_Value_k
                                        * Referenced by: '<S24>/Constant1'
                                        */
  int32_T Constant_Value_p;            /* Computed Parameter: Constant_Value_p
                                        * Referenced by: '<S23>/Constant'
                                        */
  int32_T Constant_Value_h;            /* Computed Parameter: Constant_Value_h
                                        * Referenced by: '<S28>/Constant'
                                        */
  int32_T Gain_Gain_a2;                /* Computed Parameter: Gain_Gain_a2
                                        * Referenced by: '<S28>/Gain'
                                        */
  int32_T Constant_Value_ji;           /* Computed Parameter: Constant_Value_ji
                                        * Referenced by: '<S30>/Constant'
                                        */
  int32_T Constant_Value_jd;           /* Computed Parameter: Constant_Value_jd
                                        * Referenced by: '<S15>/Constant'
                                        */
  int32_T ForIterator_IterationLimit_l;/* Computed Parameter: ForIterator_IterationLimit_l
                                        * Referenced by: '<S15>/For Iterator'
                                        */
  int32_T arn_Threshold;               /* Computed Parameter: arn_Threshold
                                        * Referenced by: '<S15>/ar(n)'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_calibrazione_acc_mag0 {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_calibrazione_acc_mag calibrazione_acc_mag0_P;

/* Block signals (auto storage) */
extern BlockIO_calibrazione_acc_mag0 calibrazione_acc_mag0_B;

/* Block states (auto storage) */
extern D_Work_calibrazione_acc_mag0 calibrazione_acc_mag0_DWork;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_calibrazione_acc calibrazione_acc_mag0_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_calibrazione_ac calibrazione_acc_mag0_Y;

/* Model entry point functions */
extern void calibrazione_acc_mag0_initialize(void);
extern void calibrazione_acc_mag0_step(void);
extern void calibrazione_acc_mag0_terminate(void);

/* Real-time Model object */
extern struct RT_MODEL_calibrazione_acc_mag0 *const calibrazione_acc_mag0_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('calibrazione_acc_mag/calibrazione_acc_mag')    - opens subsystem calibrazione_acc_mag/calibrazione_acc_mag
 * hilite_system('calibrazione_acc_mag/calibrazione_acc_mag/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : calibrazione_acc_mag
 * '<S1>'   : calibrazione_acc_mag/calibrazione_acc_mag
 * '<S2>'   : calibrazione_acc_mag/calibrazione_acc_mag/WGS-84
 * '<S3>'   : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010
 * '<S4>'   : calibrazione_acc_mag/calibrazione_acc_mag/acc_calibration
 * '<S5>'   : calibrazione_acc_mag/calibrazione_acc_mag/mag_calibration
 * '<S6>'   : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010
 * '<S7>'   : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/Check Altitude
 * '<S8>'   : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/Check Latitude
 * '<S9>'   : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/Check Longitude
 * '<S10>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/Compute x,y,z, and h components of magnetic field
 * '<S11>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/Is time within model limits
 * '<S12>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/Length Conversion
 * '<S13>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag
 * '<S14>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/Compute x,y,z, and h components of magnetic field/Angle Conversion2
 * '<S15>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates
 * '<S16>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates
 * '<S17>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates
 * '<S18>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Get Cosine and Sine  of Latitude and Longitude
 * '<S19>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Has altitude or latitude changed
 * '<S20>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Has longitude changed
 * '<S21>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Has time changed
 * '<S22>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity
 * '<S23>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem
 * '<S24>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion
 * '<S25>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations
 * '<S26>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients
 * '<S27>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole
 * '<S28>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate  index
 * '<S29>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values
 * '<S30>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/special case
 * '<S31>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem1
 * '<S32>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem2
 * '<S33>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem2/calculate  indices
 * '<S34>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/Special case - North//South Geographic Pole/If Action Subsystem2/calculate  row and column
 * '<S35>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem
 * '<S36>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem1
 * '<S37>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem1/m,n
 * '<S38>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Accumulate terms of the  spherical harmonic expansion/calculate temp values/If Action Subsystem1/n,m-1
 * '<S39>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem
 * '<S40>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem1
 * '<S41>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2
 * '<S42>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/calculate  index
 * '<S43>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem/calculate  index
 * '<S44>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem/calculate  row and column
 * '<S45>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem1/calculate  index
 * '<S46>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem1/calculate  row and column
 * '<S47>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/calculate  indices
 * '<S48>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/calculate  row and column1
 * '<S49>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/calculate  row and column2
 * '<S50>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/m<n-2
 * '<S51>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations/If Action Subsystem2/m<n-2
 * '<S52>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/If Action Subsystem
 * '<S53>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/if (m~=0)
 * '<S54>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/if (m~=0)/If Action Subsystem1
 * '<S55>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Compute magnetic vector in spherical coordinates/For Iterator Subsystem/Time adjust the gauss coefficients/if (m~=0)/If Action Subsystem2
 * '<S56>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate ca
 * '<S57>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate ct
 * '<S58>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate d
 * '<S59>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate q
 * '<S60>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate q2
 * '<S61>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate r2
 * '<S62>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate sa
 * '<S63>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates/calculate st
 * '<S64>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Convert from geodetic to  spherical coordinates /For Iterator Subsystem
 * '<S65>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Get Cosine and Sine  of Latitude and Longitude/Angle Conversion
 * '<S66>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Calculate bx
 * '<S67>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Calculate by
 * '<S68>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Calculate bz
 * '<S69>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Compute declination, inclination,  and total intensity
 * '<S70>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Compute declination, inclination,  and total intensity/Angle Conversion1
 * '<S71>'  : calibrazione_acc_mag/calibrazione_acc_mag/WMM-2010/World Magnetic Model 2010/geomag/Rotate magnetic vector components  to geodetic from spherical and  compute declination, inclination  and total intensity/Compute declination, inclination,  and total intensity/Angle Conversion2
 */
#endif                                 /* RTW_HEADER_calibrazione_acc_mag0_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
