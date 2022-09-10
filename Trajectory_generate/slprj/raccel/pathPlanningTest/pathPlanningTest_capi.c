#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "pathPlanningTest_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 18
#endif
#ifndef SS_INT64
#define SS_INT64 19
#endif
#else
#include "builtin_typeid_types.h"
#include "pathPlanningTest.h"
#include "pathPlanningTest_capi.h"
#include "pathPlanningTest_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING (
"pathPlanningTest/Selector1" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } ,
{ 1 , 0 , TARGET_STRING ( "pathPlanningTest/Add" ) , TARGET_STRING ( "" ) , 0
, 0 , 0 , 0 , 1 } , { 2 , 0 , TARGET_STRING (
"pathPlanningTest/Ackermann Kinematic Model/Integrator" ) , TARGET_STRING (
"" ) , 0 , 0 , 1 , 0 , 0 } , { 3 , 1 , TARGET_STRING (
"pathPlanningTest/Ackermann Kinematic Model/MATLAB System" ) , TARGET_STRING
( "" ) , 0 , 0 , 2 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0
, 0 } } ; static const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 4
, TARGET_STRING ( "pathPlanningTest/Ackermann Kinematic Model" ) ,
TARGET_STRING ( "VehicleSpeedRange" ) , 0 , 3 , 0 } , { 5 , TARGET_STRING (
"pathPlanningTest/linearVel3" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , {
6 , TARGET_STRING ( "pathPlanningTest/From Workspace" ) , TARGET_STRING (
"Time0" ) , 0 , 0 , 0 } , { 7 , TARGET_STRING (
"pathPlanningTest/From Workspace" ) , TARGET_STRING ( "Data0" ) , 0 , 4 , 0 }
, { 8 , TARGET_STRING ( "pathPlanningTest/From Workspace1" ) , TARGET_STRING
( "Time0" ) , 0 , 0 , 0 } , { 9 , TARGET_STRING (
"pathPlanningTest/From Workspace1" ) , TARGET_STRING ( "Data0" ) , 0 , 4 , 0
} , { 10 , TARGET_STRING ( "pathPlanningTest/Memory" ) , TARGET_STRING (
"InitialCondition" ) , 0 , 0 , 0 } , { 11 , TARGET_STRING (
"pathPlanningTest/Ackermann Kinematic Model/Integrator" ) , TARGET_STRING (
"InitialCondition" ) , 0 , 1 , 0 } , { 12 , TARGET_STRING (
"pathPlanningTest/Ackermann Kinematic Model/Integrator" ) , TARGET_STRING (
"UpperSaturationLimit" ) , 0 , 1 , 0 } , { 13 , TARGET_STRING (
"pathPlanningTest/Ackermann Kinematic Model/Integrator" ) , TARGET_STRING (
"LowerSaturationLimit" ) , 0 , 1 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 ,
0 } } ; static int_T rt_LoggedStateIdxList [ ] = { - 1 } ; static const
rtwCAPI_Signals rtRootInputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 ,
0 , 0 , 0 } } ; static const rtwCAPI_Signals rtRootOutputs [ ] = { { 0 , 0 ,
( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const
rtwCAPI_ModelParameters rtModelParameters [ ] = { { 14 , TARGET_STRING (
"iteractions" ) , 0 , 0 , 0 } , { 15 , TARGET_STRING ( "wheelbase" ) , 0 , 0
, 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . lhrp3afpef , & rtB . ombmpdkbgd ,
& rtB . anf3fqcamm [ 0 ] , & rtB . cmiugdp1qd [ 0 ] , & rtP .
AckermannKinematicModel_VehicleSpeedRange [ 0 ] , & rtP . linearVel3_Value ,
& rtP . FromWorkspace_Time0 , & rtP . FromWorkspace_Data0 [ 0 ] , & rtP .
FromWorkspace1_Time0 , & rtP . FromWorkspace1_Data0 [ 0 ] , & rtP .
Memory_InitialCondition , & rtP . Integrator_IC [ 0 ] , & rtP .
Integrator_UpperSat [ 0 ] , & rtP . Integrator_LowerSat [ 0 ] , & rtP .
iteractions , & rtP . wheelbase , } ; static int32_T * rtVarDimsAddrMap [ ] =
{ ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } }
;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 2 , 2 , 0 } , { rtwCAPI_VECTOR , 4 , 2 , 0 } , {
rtwCAPI_VECTOR , 6 , 2 , 0 } } ; static const uint_T rtDimensionArray [ ] = {
1 , 1 , 4 , 1 , 1 , 2 , 500 , 1 } ; static const real_T rtcapiStoredFloats [
] = { 0.0 , 0.01 } ; static const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { (
NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ;
static const rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * )
& rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , (
int8_T ) 0 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 1 ]
, ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 1 , ( uint8_T ) 0
} } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals ,
4 , rtRootInputs , 0 , rtRootOutputs , 0 } , { rtBlockParameters , 10 ,
rtModelParameters , 2 } , { ( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap
, rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float"
, { 1052907474U , 658466419U , 2613212608U , 2291559574U } , ( NULL ) , 0 , (
boolean_T ) 0 , rt_LoggedStateIdxList } ; const
rtwCAPI_ModelMappingStaticInfo * pathPlanningTest_GetCAPIStaticMap ( void ) {
return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void pathPlanningTest_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( (
* rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void pathPlanningTest_host_InitializeDataMapInfo (
pathPlanningTest_host_DataMapInfo_T * dataMap , const char * path ) {
rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap ->
mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetPath ( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap ->
mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
