#ifndef RTW_HEADER_pathPlanningTest_h_
#define RTW_HEADER_pathPlanningTest_h_
#ifndef pathPlanningTest_COMMON_INCLUDES_
#define pathPlanningTest_COMMON_INCLUDES_
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "pathPlanningTest_types.h"
#include "mwmathutil.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#define MODEL_NAME pathPlanningTest
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (4) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (4)   
#elif NCSTATES != 4
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T ombmpdkbgd ; real_T lhrp3afpef ; real_T anf3fqcamm [
4 ] ; real_T cmiugdp1qd [ 4 ] ; } B ; typedef struct { pxmbdpwwlm pvlsfgblh4
; real_T l5p0ekiltu ; struct { void * TimePtr ; void * DataPtr ; void *
RSimInfoPtr ; } fslrnbblve ; struct { void * LoggedData ; } bqx3ghvx5v ;
struct { void * AQHandles ; } agh1p1wjbr ; struct { void * TimePtr ; void *
DataPtr ; void * RSimInfoPtr ; } oc52imyk4j ; struct { int_T PrevIndex ; }
czyahcbvuz ; struct { int_T PrevIndex ; } am00lbhjdq ; boolean_T jsnnhatzfm ;
} DW ; typedef struct { real_T iij3drc5sy [ 4 ] ; } X ; typedef struct {
real_T iij3drc5sy [ 4 ] ; } XDot ; typedef struct { boolean_T iij3drc5sy [ 4
] ; } XDis ; typedef struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ;
struct P_ { real_T iteractions ; real_T wheelbase ; real_T
AckermannKinematicModel_VehicleSpeedRange [ 2 ] ; real_T FromWorkspace_Time0
; real_T FromWorkspace_Data0 [ 500 ] ; real_T Memory_InitialCondition ;
real_T Integrator_IC [ 4 ] ; real_T Integrator_UpperSat [ 4 ] ; real_T
Integrator_LowerSat [ 4 ] ; real_T FromWorkspace1_Time0 ; real_T
FromWorkspace1_Data0 [ 500 ] ; real_T linearVel3_Value ; } ; extern const
char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ; extern DW
rtDW ; extern P rtP ; extern mxArray * mr_pathPlanningTest_GetDWork ( ) ;
extern void mr_pathPlanningTest_SetDWork ( const mxArray * ssDW ) ; extern
mxArray * mr_pathPlanningTest_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * pathPlanningTest_GetCAPIStaticMap ( void ) ;
extern SimStruct * const rtS ; extern const int_T gblNumToFiles ; extern
const int_T gblNumFrFiles ; extern const int_T gblNumFrWksBlocks ; extern
rtInportTUtable * gblInportTUtables ; extern const char * gblInportFileName ;
extern const int_T gblNumRootInportBlks ; extern const int_T
gblNumModelInputs ; extern const int_T gblInportDataTypeIdx [ ] ; extern
const int_T gblInportDims [ ] ; extern const int_T gblInportComplex [ ] ;
extern const int_T gblInportInterpoFlag [ ] ; extern const int_T
gblInportContinuous [ ] ; extern const int_T gblParameterTuningTid ; extern
DataMapInfo * rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo *
rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid ) ; void
MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ;
void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void
MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model (
ssExecutionInfo * executionInfo ) ;
#endif
