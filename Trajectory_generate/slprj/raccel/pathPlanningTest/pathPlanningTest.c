#include "pathPlanningTest.h"
#include "pathPlanningTest_types.h"
#include "rtwtypes.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include "pathPlanningTest_private.h"
#include "rt_logging_mmi.h"
#include "pathPlanningTest_capi.h"
#include "pathPlanningTest_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; void
raccelForceExtModeShutdown ( boolean_T extModeStartPktReceived ) { if ( !
extModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 2 , & stopRequested ) ; }
rtExtModeShutdown ( 2 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
#include "slsa_sim_engine.h"
const int_T gblNumToFiles = 0 ; const int_T gblNumFrFiles = 0 ; const int_T
gblNumFrWksBlocks = 2 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 1 ; int_T gbl_raccel_NumST = 3 ; const char_T
* gbl_raccel_Version = "9.7 (R2022a) 13-Nov-2021" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const int_T gblNumRootInportBlks = 0 ; const int_T
gblNumModelInputs = 0 ; extern const char * gblInportFileName ; extern
rtInportTUtable * gblInportTUtables ; const int_T gblInportDataTypeIdx [ ] =
{ - 1 } ; const int_T gblInportDims [ ] = { - 1 } ; const int_T
gblInportComplex [ ] = { - 1 } ; const int_T gblInportInterpoFlag [ ] = { - 1
} ; const int_T gblInportContinuous [ ] = { - 1 } ; int_T enableFcnCallFlag [
] = { 1 , 1 , 1 } ; const char * raccelLoadInputsAndAperiodicHitTimes (
SimStruct * S , const char * inportFileName , int * matFileFormat ) { return
rt_RAccelReadInportsMatFile ( S , inportFileName , matFileFormat ) ; }
#include "simstruc.h"
#include "fixedpoint.h"
#include "slsa_sim_engine.h"
#include "simtarget/slSimTgtSLExecSimBridge.h"
B rtB ; X rtX ; DW rtDW ; static SimStruct model_S ; SimStruct * const rtS =
& model_S ; static void drjwi24wqz ( pxmbdpwwlm * obj ) ; static void
drjwi24wqz ( pxmbdpwwlm * obj ) { obj -> isInitialized = 1 ; obj -> KinModel
. WheelBase = 1.0 ; obj -> KinModel . VehicleSpeedRange [ 0 ] = ( rtMinusInf
) ; obj -> KinModel . VehicleSpeedRange [ 0 ] = obj -> VehicleSpeedRange [ 0
] ; obj -> KinModel . VehicleSpeedRange [ 1 ] = ( rtInf ) ; obj -> KinModel .
VehicleSpeedRange [ 1 ] = obj -> VehicleSpeedRange [ 1 ] ; obj -> KinModel .
WheelBase = obj -> WheelBase ; obj -> TunablePropsChanged = false ; } void
MdlInitialize ( void ) { rtDW . l5p0ekiltu = rtP . Memory_InitialCondition ;
rtX . iij3drc5sy [ 0 ] = rtP . Integrator_IC [ 0 ] ; rtX . iij3drc5sy [ 1 ] =
rtP . Integrator_IC [ 1 ] ; rtX . iij3drc5sy [ 2 ] = rtP . Integrator_IC [ 2
] ; rtX . iij3drc5sy [ 3 ] = rtP . Integrator_IC [ 3 ] ; rtDW . pvlsfgblh4 .
KinModel . VehicleSpeedRange [ 0 ] = rtDW . pvlsfgblh4 . VehicleSpeedRange [
0 ] ; rtDW . pvlsfgblh4 . KinModel . VehicleSpeedRange [ 1 ] = rtDW .
pvlsfgblh4 . VehicleSpeedRange [ 1 ] ; rtDW . pvlsfgblh4 . KinModel .
WheelBase = rtDW . pvlsfgblh4 . WheelBase ; } void MdlStart ( void ) { { bool
externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( rtS ) ;
rtwISigstreamManagerGetInputIsInDatasetFormat ( pISigstreamManager , &
externalInputIsInDatasetFormat ) ; if ( externalInputIsInDatasetFormat ) { }
} { { { bool isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU
srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars ( "Mux1" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Mux1" ) ; sdiLabelU blockPath = sdiGetLabelFromChars
( "pathPlanningTest/To Workspace2" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Mux1" ) ;
sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 1 ] = { 3 } ; sigDims . nDims = 1 ; sigDims . dimensions = sigDimsArray ;
srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU
) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ; srcInfo .
subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo . signalName =
sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . agh1p1wjbr . AQHandles =
sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo . mmi .
InstanceMap . fullPath , "7cb7d7ee-0db6-44ce-8cc3-306f03282c60" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ;
sdiCompleteAsyncioQueueCreation ( rtDW . agh1p1wjbr . AQHandles , hDT , &
srcInfo ) ; if ( rtDW . agh1p1wjbr . AQHandles ) {
sdiSetSignalSampleTimeString ( rtDW . agh1p1wjbr . AQHandles , "0.01" , 0.01
, ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW . agh1p1wjbr . AQHandles
, 0.0 ) ; sdiSetRunStartTime ( rtDW . agh1p1wjbr . AQHandles , ssGetTaskTime
( rtS , 1 ) ) ; sdiAsyncRepoSetSignalExportSettings ( rtDW . agh1p1wjbr .
AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName ( rtDW . agh1p1wjbr .
AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . agh1p1wjbr . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"CarPose" ) ; sdiRegisterWksVariable ( rtDW . agh1p1wjbr . AQHandles ,
varName , "array" ) ; sdiFreeLabel ( varName ) ; } } } } } { FWksInfo *
fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo * ) calloc ( 1 , sizeof (
FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus ( rtS ,
"from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "v" ; fromwksInfo -> origDataTypeId = 0
; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 500 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWorkspace_Data0 ; fromwksInfo -> nDataPoints = 1 ; fromwksInfo
-> time = ( double * ) & rtP . FromWorkspace_Time0 ; rtDW . fslrnbblve .
TimePtr = fromwksInfo -> time ; rtDW . fslrnbblve . DataPtr = fromwksInfo ->
data ; rtDW . fslrnbblve . RSimInfoPtr = fromwksInfo ; } rtDW . czyahcbvuz .
PrevIndex = 0 ; } { FWksInfo * fromwksInfo ; if ( ( fromwksInfo = ( FWksInfo
* ) calloc ( 1 , sizeof ( FWksInfo ) ) ) == ( NULL ) ) { ssSetErrorStatus (
rtS , "from workspace STRING(Name) memory allocation error" ) ; } else {
fromwksInfo -> origWorkspaceVarName = "dphi" ; fromwksInfo -> origDataTypeId
= 0 ; fromwksInfo -> origIsComplex = 0 ; fromwksInfo -> origWidth = 500 ;
fromwksInfo -> origElSize = sizeof ( real_T ) ; fromwksInfo -> data = ( void
* ) rtP . FromWorkspace1_Data0 ; fromwksInfo -> nDataPoints = 1 ; fromwksInfo
-> time = ( double * ) & rtP . FromWorkspace1_Time0 ; rtDW . oc52imyk4j .
TimePtr = fromwksInfo -> time ; rtDW . oc52imyk4j . DataPtr = fromwksInfo ->
data ; rtDW . oc52imyk4j . RSimInfoPtr = fromwksInfo ; } rtDW . am00lbhjdq .
PrevIndex = 0 ; } rtDW . pvlsfgblh4 . isInitialized = 0 ; rtDW . pvlsfgblh4 .
tunablePropertyChanged [ 0 ] = false ; rtDW . pvlsfgblh4 .
tunablePropertyChanged [ 1 ] = false ; rtDW . jsnnhatzfm = true ; if ( rtDW .
pvlsfgblh4 . isInitialized == 1 ) { rtDW . pvlsfgblh4 . TunablePropsChanged =
true ; rtDW . pvlsfgblh4 . tunablePropertyChanged [ 0 ] = true ; } rtDW .
pvlsfgblh4 . WheelBase = rtP . wheelbase ; if ( rtDW . pvlsfgblh4 .
isInitialized == 1 ) { rtDW . pvlsfgblh4 . TunablePropsChanged = true ; rtDW
. pvlsfgblh4 . tunablePropertyChanged [ 1 ] = true ; } rtDW . pvlsfgblh4 .
VehicleSpeedRange [ 0 ] = rtP . AckermannKinematicModel_VehicleSpeedRange [ 0
] ; rtDW . pvlsfgblh4 . VehicleSpeedRange [ 1 ] = rtP .
AckermannKinematicModel_VehicleSpeedRange [ 1 ] ; drjwi24wqz ( & rtDW .
pvlsfgblh4 ) ; MdlInitialize ( ) ; } void MdlOutputs ( int_T tid ) { real_T
ksq1cq4mtl [ 500 ] ; __m128d tmp_e ; __m128d tmp_i ; real_T tmp_p [ 8 ] ;
real_T cmds_idx_1 ; real_T tmp ; int32_T i ; boolean_T exitg1 ; boolean_T
lhkt0dexha ; boolean_T p ; { real_T * pDataValues = ( real_T * ) rtDW .
fslrnbblve . DataPtr ; { int_T elIdx ; for ( elIdx = 0 ; elIdx < 500 ; ++
elIdx ) { ( & ksq1cq4mtl [ 0 ] ) [ elIdx ] = pDataValues [ 0 ] ; pDataValues
+= 1 ; } } } if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtB . ombmpdkbgd = rtP .
linearVel3_Value + rtDW . l5p0ekiltu ; } rtB . lhrp3afpef = ksq1cq4mtl [ (
int32_T ) rtB . ombmpdkbgd - 1 ] ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { if
( ! ( rtB . ombmpdkbgd < rtP . iteractions - 1.0 ) ) { ssSetStopRequested (
rtS , 1 ) ; } } if ( ssIsModeUpdateTimeStep ( rtS ) ) { if ( rtX . iij3drc5sy
[ 0 ] >= rtP . Integrator_UpperSat [ 0 ] ) { if ( rtX . iij3drc5sy [ 0 ] !=
rtP . Integrator_UpperSat [ 0 ] ) { rtX . iij3drc5sy [ 0 ] = rtP .
Integrator_UpperSat [ 0 ] ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; } } else if ( ( rtX . iij3drc5sy [ 0 ] <= rtP . Integrator_LowerSat [ 0 ]
) && ( rtX . iij3drc5sy [ 0 ] != rtP . Integrator_LowerSat [ 0 ] ) ) { rtX .
iij3drc5sy [ 0 ] = rtP . Integrator_LowerSat [ 0 ] ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtX . iij3drc5sy
[ 1 ] >= rtP . Integrator_UpperSat [ 1 ] ) { if ( rtX . iij3drc5sy [ 1 ] !=
rtP . Integrator_UpperSat [ 1 ] ) { rtX . iij3drc5sy [ 1 ] = rtP .
Integrator_UpperSat [ 1 ] ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; } } else if ( ( rtX . iij3drc5sy [ 1 ] <= rtP . Integrator_LowerSat [ 1 ]
) && ( rtX . iij3drc5sy [ 1 ] != rtP . Integrator_LowerSat [ 1 ] ) ) { rtX .
iij3drc5sy [ 1 ] = rtP . Integrator_LowerSat [ 1 ] ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtX . iij3drc5sy
[ 2 ] >= rtP . Integrator_UpperSat [ 2 ] ) { if ( rtX . iij3drc5sy [ 2 ] !=
rtP . Integrator_UpperSat [ 2 ] ) { rtX . iij3drc5sy [ 2 ] = rtP .
Integrator_UpperSat [ 2 ] ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; } } else if ( ( rtX . iij3drc5sy [ 2 ] <= rtP . Integrator_LowerSat [ 2 ]
) && ( rtX . iij3drc5sy [ 2 ] != rtP . Integrator_LowerSat [ 2 ] ) ) { rtX .
iij3drc5sy [ 2 ] = rtP . Integrator_LowerSat [ 2 ] ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } if ( rtX . iij3drc5sy
[ 3 ] >= rtP . Integrator_UpperSat [ 3 ] ) { if ( rtX . iij3drc5sy [ 3 ] !=
rtP . Integrator_UpperSat [ 3 ] ) { rtX . iij3drc5sy [ 3 ] = rtP .
Integrator_UpperSat [ 3 ] ; ssSetBlockStateForSolverChangedAtMajorStep ( rtS
) ; } } else if ( ( rtX . iij3drc5sy [ 3 ] <= rtP . Integrator_LowerSat [ 3 ]
) && ( rtX . iij3drc5sy [ 3 ] != rtP . Integrator_LowerSat [ 3 ] ) ) { rtX .
iij3drc5sy [ 3 ] = rtP . Integrator_LowerSat [ 3 ] ;
ssSetBlockStateForSolverChangedAtMajorStep ( rtS ) ; } rtB . anf3fqcamm [ 0 ]
= rtX . iij3drc5sy [ 0 ] ; rtB . anf3fqcamm [ 1 ] = rtX . iij3drc5sy [ 1 ] ;
rtB . anf3fqcamm [ 2 ] = rtX . iij3drc5sy [ 2 ] ; rtB . anf3fqcamm [ 3 ] =
rtX . iij3drc5sy [ 3 ] ; } else { if ( rtX . iij3drc5sy [ 0 ] >= rtP .
Integrator_UpperSat [ 0 ] ) { rtB . anf3fqcamm [ 0 ] = rtP .
Integrator_UpperSat [ 0 ] ; } else if ( rtX . iij3drc5sy [ 0 ] <= rtP .
Integrator_LowerSat [ 0 ] ) { rtB . anf3fqcamm [ 0 ] = rtP .
Integrator_LowerSat [ 0 ] ; } else { rtB . anf3fqcamm [ 0 ] = rtX .
iij3drc5sy [ 0 ] ; } if ( rtX . iij3drc5sy [ 1 ] >= rtP . Integrator_UpperSat
[ 1 ] ) { rtB . anf3fqcamm [ 1 ] = rtP . Integrator_UpperSat [ 1 ] ; } else
if ( rtX . iij3drc5sy [ 1 ] <= rtP . Integrator_LowerSat [ 1 ] ) { rtB .
anf3fqcamm [ 1 ] = rtP . Integrator_LowerSat [ 1 ] ; } else { rtB .
anf3fqcamm [ 1 ] = rtX . iij3drc5sy [ 1 ] ; } if ( rtX . iij3drc5sy [ 2 ] >=
rtP . Integrator_UpperSat [ 2 ] ) { rtB . anf3fqcamm [ 2 ] = rtP .
Integrator_UpperSat [ 2 ] ; } else if ( rtX . iij3drc5sy [ 2 ] <= rtP .
Integrator_LowerSat [ 2 ] ) { rtB . anf3fqcamm [ 2 ] = rtP .
Integrator_LowerSat [ 2 ] ; } else { rtB . anf3fqcamm [ 2 ] = rtX .
iij3drc5sy [ 2 ] ; } if ( rtX . iij3drc5sy [ 3 ] >= rtP . Integrator_UpperSat
[ 3 ] ) { rtB . anf3fqcamm [ 3 ] = rtP . Integrator_UpperSat [ 3 ] ; } else
if ( rtX . iij3drc5sy [ 3 ] <= rtP . Integrator_LowerSat [ 3 ] ) { rtB .
anf3fqcamm [ 3 ] = rtP . Integrator_LowerSat [ 3 ] ; } else { rtB .
anf3fqcamm [ 3 ] = rtX . iij3drc5sy [ 3 ] ; } } if ( ssIsSampleHit ( rtS , 1
, 0 ) ) { { if ( rtDW . agh1p1wjbr . AQHandles && ssGetLogOutput ( rtS ) ) {
sdiWriteSignal ( rtDW . agh1p1wjbr . AQHandles , ssGetTaskTime ( rtS , 1 ) ,
( char * ) & rtB . anf3fqcamm [ 0 ] + 0 ) ; } } } { real_T * pDataValues = (
real_T * ) rtDW . oc52imyk4j . DataPtr ; { int_T elIdx ; for ( elIdx = 0 ;
elIdx < 500 ; ++ elIdx ) { ( & ksq1cq4mtl [ 0 ] ) [ elIdx ] = pDataValues [ 0
] ; pDataValues += 1 ; } } } if ( rtDW . pvlsfgblh4 . WheelBase != rtP .
wheelbase ) { if ( rtDW . pvlsfgblh4 . isInitialized == 1 ) { rtDW .
pvlsfgblh4 . TunablePropsChanged = true ; rtDW . pvlsfgblh4 .
tunablePropertyChanged [ 0 ] = true ; } rtDW . pvlsfgblh4 . WheelBase = rtP .
wheelbase ; } lhkt0dexha = false ; p = true ; i = 0 ; exitg1 = false ; while
( ( ! exitg1 ) && ( i < 2 ) ) { if ( ! ( rtDW . pvlsfgblh4 .
VehicleSpeedRange [ i ] == rtP . AckermannKinematicModel_VehicleSpeedRange [
i ] ) ) { p = false ; exitg1 = true ; } else { i ++ ; } } if ( p ) {
lhkt0dexha = true ; } if ( ! lhkt0dexha ) { if ( rtDW . pvlsfgblh4 .
isInitialized == 1 ) { rtDW . pvlsfgblh4 . TunablePropsChanged = true ; rtDW
. pvlsfgblh4 . tunablePropertyChanged [ 1 ] = true ; } rtDW . pvlsfgblh4 .
VehicleSpeedRange [ 0 ] = rtP . AckermannKinematicModel_VehicleSpeedRange [ 0
] ; rtDW . pvlsfgblh4 . VehicleSpeedRange [ 1 ] = rtP .
AckermannKinematicModel_VehicleSpeedRange [ 1 ] ; } if ( rtDW . pvlsfgblh4 .
TunablePropsChanged ) { rtDW . pvlsfgblh4 . TunablePropsChanged = false ; if
( rtDW . pvlsfgblh4 . tunablePropertyChanged [ 1 ] ) { rtDW . pvlsfgblh4 .
KinModel . VehicleSpeedRange [ 0 ] = rtDW . pvlsfgblh4 . VehicleSpeedRange [
0 ] ; rtDW . pvlsfgblh4 . KinModel . VehicleSpeedRange [ 1 ] = rtDW .
pvlsfgblh4 . VehicleSpeedRange [ 1 ] ; } if ( rtDW . pvlsfgblh4 .
tunablePropertyChanged [ 0 ] ) { rtDW . pvlsfgblh4 . KinModel . WheelBase =
rtDW . pvlsfgblh4 . WheelBase ; } rtDW . pvlsfgblh4 . tunablePropertyChanged
[ 0 ] = false ; rtDW . pvlsfgblh4 . tunablePropertyChanged [ 1 ] = false ; }
cmds_idx_1 = ksq1cq4mtl [ ( int32_T ) rtB . ombmpdkbgd - 1 ] ; tmp =
muDoubleScalarMin ( muDoubleScalarMax ( rtB . lhrp3afpef , rtDW . pvlsfgblh4
. KinModel . VehicleSpeedRange [ 0 ] ) , rtDW . pvlsfgblh4 . KinModel .
VehicleSpeedRange [ 1 ] ) ; tmp_p [ 0 ] = muDoubleScalarCos ( rtB .
anf3fqcamm [ 2 ] ) ; tmp_p [ 4 ] = 0.0 ; tmp_p [ 1 ] = muDoubleScalarSin (
rtB . anf3fqcamm [ 2 ] ) ; tmp_p [ 5 ] = 0.0 ; tmp_p [ 2 ] =
muDoubleScalarTan ( rtB . anf3fqcamm [ 3 ] ) / rtDW . pvlsfgblh4 . KinModel .
WheelBase ; tmp_p [ 6 ] = 0.0 ; tmp_p [ 3 ] = 0.0 ; tmp_p [ 7 ] = 1.0 ; for (
i = 0 ; i <= 2 ; i += 2 ) { tmp_e = _mm_loadu_pd ( & tmp_p [ i ] ) ; tmp_i =
_mm_loadu_pd ( & tmp_p [ i + 4 ] ) ; _mm_storeu_pd ( & rtB . cmiugdp1qd [ i ]
, _mm_add_pd ( _mm_mul_pd ( tmp_i , _mm_set1_pd ( cmds_idx_1 ) ) , _mm_add_pd
( _mm_mul_pd ( tmp_e , _mm_set1_pd ( tmp ) ) , _mm_set1_pd ( 0.0 ) ) ) ) ; }
UNUSED_PARAMETER ( tid ) ; } void MdlOutputsTID2 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlUpdate ( int_T tid ) { if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW . l5p0ekiltu = rtB . ombmpdkbgd ; }
UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID2 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) { XDot * _rtXdot ;
_rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; if ( ( ( rtX . iij3drc5sy [ 0 ] >
rtP . Integrator_LowerSat [ 0 ] ) && ( rtX . iij3drc5sy [ 0 ] < rtP .
Integrator_UpperSat [ 0 ] ) ) || ( ( rtX . iij3drc5sy [ 0 ] <= rtP .
Integrator_LowerSat [ 0 ] ) && ( rtB . cmiugdp1qd [ 0 ] > 0.0 ) ) || ( ( rtX
. iij3drc5sy [ 0 ] >= rtP . Integrator_UpperSat [ 0 ] ) && ( rtB . cmiugdp1qd
[ 0 ] < 0.0 ) ) ) { _rtXdot -> iij3drc5sy [ 0 ] = rtB . cmiugdp1qd [ 0 ] ; }
else { _rtXdot -> iij3drc5sy [ 0 ] = 0.0 ; } if ( ( ( rtX . iij3drc5sy [ 1 ]
> rtP . Integrator_LowerSat [ 1 ] ) && ( rtX . iij3drc5sy [ 1 ] < rtP .
Integrator_UpperSat [ 1 ] ) ) || ( ( rtX . iij3drc5sy [ 1 ] <= rtP .
Integrator_LowerSat [ 1 ] ) && ( rtB . cmiugdp1qd [ 1 ] > 0.0 ) ) || ( ( rtX
. iij3drc5sy [ 1 ] >= rtP . Integrator_UpperSat [ 1 ] ) && ( rtB . cmiugdp1qd
[ 1 ] < 0.0 ) ) ) { _rtXdot -> iij3drc5sy [ 1 ] = rtB . cmiugdp1qd [ 1 ] ; }
else { _rtXdot -> iij3drc5sy [ 1 ] = 0.0 ; } if ( ( ( rtX . iij3drc5sy [ 2 ]
> rtP . Integrator_LowerSat [ 2 ] ) && ( rtX . iij3drc5sy [ 2 ] < rtP .
Integrator_UpperSat [ 2 ] ) ) || ( ( rtX . iij3drc5sy [ 2 ] <= rtP .
Integrator_LowerSat [ 2 ] ) && ( rtB . cmiugdp1qd [ 2 ] > 0.0 ) ) || ( ( rtX
. iij3drc5sy [ 2 ] >= rtP . Integrator_UpperSat [ 2 ] ) && ( rtB . cmiugdp1qd
[ 2 ] < 0.0 ) ) ) { _rtXdot -> iij3drc5sy [ 2 ] = rtB . cmiugdp1qd [ 2 ] ; }
else { _rtXdot -> iij3drc5sy [ 2 ] = 0.0 ; } if ( ( ( rtX . iij3drc5sy [ 3 ]
> rtP . Integrator_LowerSat [ 3 ] ) && ( rtX . iij3drc5sy [ 3 ] < rtP .
Integrator_UpperSat [ 3 ] ) ) || ( ( rtX . iij3drc5sy [ 3 ] <= rtP .
Integrator_LowerSat [ 3 ] ) && ( rtB . cmiugdp1qd [ 3 ] > 0.0 ) ) || ( ( rtX
. iij3drc5sy [ 3 ] >= rtP . Integrator_UpperSat [ 3 ] ) && ( rtB . cmiugdp1qd
[ 3 ] < 0.0 ) ) ) { _rtXdot -> iij3drc5sy [ 3 ] = rtB . cmiugdp1qd [ 3 ] ; }
else { _rtXdot -> iij3drc5sy [ 3 ] = 0.0 ; } } void MdlProjection ( void ) {
} void MdlTerminate ( void ) { rt_FREE ( rtDW . fslrnbblve . RSimInfoPtr ) ;
rt_FREE ( rtDW . oc52imyk4j . RSimInfoPtr ) ; { if ( rtDW . agh1p1wjbr .
AQHandles ) { sdiTerminateStreaming ( & rtDW . agh1p1wjbr . AQHandles ) ; } }
} static void mr_pathPlanningTest_cacheDataAsMxArray ( mxArray * destArray ,
mwIndex i , int j , const void * srcData , size_t numBytes ) ; static void
mr_pathPlanningTest_cacheDataAsMxArray ( mxArray * destArray , mwIndex i ,
int j , const void * srcData , size_t numBytes ) { mxArray * newArray =
mxCreateUninitNumericMatrix ( ( size_t ) 1 , numBytes , mxUINT8_CLASS ,
mxREAL ) ; memcpy ( ( uint8_T * ) mxGetData ( newArray ) , ( const uint8_T *
) srcData , numBytes ) ; mxSetFieldByNumber ( destArray , i , j , newArray )
; } static void mr_pathPlanningTest_restoreDataFromMxArray ( void * destData
, const mxArray * srcArray , mwIndex i , int j , size_t numBytes ) ; static
void mr_pathPlanningTest_restoreDataFromMxArray ( void * destData , const
mxArray * srcArray , mwIndex i , int j , size_t numBytes ) { memcpy ( (
uint8_T * ) destData , ( const uint8_T * ) mxGetData ( mxGetFieldByNumber (
srcArray , i , j ) ) , numBytes ) ; } static void
mr_pathPlanningTest_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex i
, int j , uint_T bitVal ) ; static void
mr_pathPlanningTest_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex i
, int j , uint_T bitVal ) { mxSetFieldByNumber ( destArray , i , j ,
mxCreateDoubleScalar ( ( double ) bitVal ) ) ; } static uint_T
mr_pathPlanningTest_extractBitFieldFromMxArray ( const mxArray * srcArray ,
mwIndex i , int j , uint_T numBits ) ; static uint_T
mr_pathPlanningTest_extractBitFieldFromMxArray ( const mxArray * srcArray ,
mwIndex i , int j , uint_T numBits ) { const uint_T varVal = ( uint_T )
mxGetScalar ( mxGetFieldByNumber ( srcArray , i , j ) ) ; return varVal & ( (
1u << numBits ) - 1u ) ; } static void
mr_pathPlanningTest_cacheDataToMxArrayWithOffset ( mxArray * destArray ,
mwIndex i , int j , mwIndex offset , const void * srcData , size_t numBytes )
; static void mr_pathPlanningTest_cacheDataToMxArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , const void * srcData ,
size_t numBytes ) { uint8_T * varData = ( uint8_T * ) mxGetData (
mxGetFieldByNumber ( destArray , i , j ) ) ; memcpy ( ( uint8_T * ) & varData
[ offset * numBytes ] , ( const uint8_T * ) srcData , numBytes ) ; } static
void mr_pathPlanningTest_restoreDataFromMxArrayWithOffset ( void * destData ,
const mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t
numBytes ) ; static void mr_pathPlanningTest_restoreDataFromMxArrayWithOffset
( void * destData , const mxArray * srcArray , mwIndex i , int j , mwIndex
offset , size_t numBytes ) { const uint8_T * varData = ( const uint8_T * )
mxGetData ( mxGetFieldByNumber ( srcArray , i , j ) ) ; memcpy ( ( uint8_T *
) destData , ( const uint8_T * ) & varData [ offset * numBytes ] , numBytes )
; } static void mr_pathPlanningTest_cacheBitFieldToCellArrayWithOffset (
mxArray * destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal )
; static void mr_pathPlanningTest_cacheBitFieldToCellArrayWithOffset (
mxArray * destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal )
{ mxSetCell ( mxGetFieldByNumber ( destArray , i , j ) , offset ,
mxCreateDoubleScalar ( ( double ) fieldVal ) ) ; } static uint_T
mr_pathPlanningTest_extractBitFieldFromCellArrayWithOffset ( const mxArray *
srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) ; static
uint_T mr_pathPlanningTest_extractBitFieldFromCellArrayWithOffset ( const
mxArray * srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) {
const uint_T fieldVal = ( uint_T ) mxGetScalar ( mxGetCell (
mxGetFieldByNumber ( srcArray , i , j ) , offset ) ) ; return fieldVal & ( (
1u << numBits ) - 1u ) ; } mxArray * mr_pathPlanningTest_GetDWork ( ) {
static const char * ssDWFieldNames [ 3 ] = { "rtB" , "rtDW" , "NULL_PrevZCX"
, } ; mxArray * ssDW = mxCreateStructMatrix ( 1 , 1 , 3 , ssDWFieldNames ) ;
mr_pathPlanningTest_cacheDataAsMxArray ( ssDW , 0 , 0 , ( const void * ) & (
rtB ) , sizeof ( rtB ) ) ; { static const char * rtdwDataFieldNames [ 5 ] = {
"rtDW.pvlsfgblh4" , "rtDW.l5p0ekiltu" , "rtDW.czyahcbvuz" , "rtDW.am00lbhjdq"
, "rtDW.jsnnhatzfm" , } ; mxArray * rtdwData = mxCreateStructMatrix ( 1 , 1 ,
5 , rtdwDataFieldNames ) ; mr_pathPlanningTest_cacheDataAsMxArray ( rtdwData
, 0 , 0 , ( const void * ) & ( rtDW . pvlsfgblh4 ) , sizeof ( rtDW .
pvlsfgblh4 ) ) ; mr_pathPlanningTest_cacheDataAsMxArray ( rtdwData , 0 , 1 ,
( const void * ) & ( rtDW . l5p0ekiltu ) , sizeof ( rtDW . l5p0ekiltu ) ) ;
mr_pathPlanningTest_cacheDataAsMxArray ( rtdwData , 0 , 2 , ( const void * )
& ( rtDW . czyahcbvuz ) , sizeof ( rtDW . czyahcbvuz ) ) ;
mr_pathPlanningTest_cacheDataAsMxArray ( rtdwData , 0 , 3 , ( const void * )
& ( rtDW . am00lbhjdq ) , sizeof ( rtDW . am00lbhjdq ) ) ;
mr_pathPlanningTest_cacheDataAsMxArray ( rtdwData , 0 , 4 , ( const void * )
& ( rtDW . jsnnhatzfm ) , sizeof ( rtDW . jsnnhatzfm ) ) ; mxSetFieldByNumber
( ssDW , 0 , 1 , rtdwData ) ; } return ssDW ; } void
mr_pathPlanningTest_SetDWork ( const mxArray * ssDW ) { ( void ) ssDW ;
mr_pathPlanningTest_restoreDataFromMxArray ( ( void * ) & ( rtB ) , ssDW , 0
, 0 , sizeof ( rtB ) ) ; { const mxArray * rtdwData = mxGetFieldByNumber (
ssDW , 0 , 1 ) ; mr_pathPlanningTest_restoreDataFromMxArray ( ( void * ) & (
rtDW . pvlsfgblh4 ) , rtdwData , 0 , 0 , sizeof ( rtDW . pvlsfgblh4 ) ) ;
mr_pathPlanningTest_restoreDataFromMxArray ( ( void * ) & ( rtDW . l5p0ekiltu
) , rtdwData , 0 , 1 , sizeof ( rtDW . l5p0ekiltu ) ) ;
mr_pathPlanningTest_restoreDataFromMxArray ( ( void * ) & ( rtDW . czyahcbvuz
) , rtdwData , 0 , 2 , sizeof ( rtDW . czyahcbvuz ) ) ;
mr_pathPlanningTest_restoreDataFromMxArray ( ( void * ) & ( rtDW . am00lbhjdq
) , rtdwData , 0 , 3 , sizeof ( rtDW . am00lbhjdq ) ) ;
mr_pathPlanningTest_restoreDataFromMxArray ( ( void * ) & ( rtDW . jsnnhatzfm
) , rtdwData , 0 , 4 , sizeof ( rtDW . jsnnhatzfm ) ) ; } } mxArray *
mr_pathPlanningTest_GetSimStateDisallowedBlocks ( ) { mxArray * data =
mxCreateCellMatrix ( 2 , 3 ) ; mwIndex subs [ 2 ] , offset ; { static const
char * blockType [ 2 ] = { "Scope" , "MATLABSystem" , } ; static const char *
blockPath [ 2 ] = { "pathPlanningTest/Scope" ,
"pathPlanningTest/Ackermann Kinematic Model/MATLAB System" , } ; static const
int reason [ 2 ] = { 0 , 6 , } ; for ( subs [ 0 ] = 0 ; subs [ 0 ] < 2 ; ++ (
subs [ 0 ] ) ) { subs [ 1 ] = 0 ; offset = mxCalcSingleSubscript ( data , 2 ,
subs ) ; mxSetCell ( data , offset , mxCreateString ( blockType [ subs [ 0 ]
] ) ) ; subs [ 1 ] = 1 ; offset = mxCalcSingleSubscript ( data , 2 , subs ) ;
mxSetCell ( data , offset , mxCreateString ( blockPath [ subs [ 0 ] ] ) ) ;
subs [ 1 ] = 2 ; offset = mxCalcSingleSubscript ( data , 2 , subs ) ;
mxSetCell ( data , offset , mxCreateDoubleScalar ( ( double ) reason [ subs [
0 ] ] ) ) ; } } return data ; } void MdlInitializeSizes ( void ) {
ssSetNumContStates ( rtS , 4 ) ; ssSetNumPeriodicContStates ( rtS , 0 ) ;
ssSetNumY ( rtS , 0 ) ; ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS
, 0 ) ; ssSetNumSampleTimes ( rtS , 2 ) ; ssSetNumBlocks ( rtS , 18 ) ;
ssSetNumBlockIO ( rtS , 4 ) ; ssSetNumBlockParams ( rtS , 1020 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.01 ) ; ssSetOffsetTime ( rtS , 0 , 0.0 ) ;
ssSetOffsetTime ( rtS , 1 , 0.0 ) ; } void raccel_set_checksum ( ) {
ssSetChecksumVal ( rtS , 0 , 1052907474U ) ; ssSetChecksumVal ( rtS , 1 ,
658466419U ) ; ssSetChecksumVal ( rtS , 2 , 2613212608U ) ; ssSetChecksumVal
( rtS , 3 , 2291559574U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) {
static struct _ssMdlInfo mdlInfo ; static struct _ssBlkInfo2 blkInfo2 ;
static struct _ssBlkInfoSLSize blkInfoSLSize ; ( void ) memset ( ( char * )
rtS , 0 , sizeof ( SimStruct ) ) ; ( void ) memset ( ( char * ) & mdlInfo , 0
, sizeof ( struct _ssMdlInfo ) ) ; ( void ) memset ( ( char * ) & blkInfo2 ,
0 , sizeof ( struct _ssBlkInfo2 ) ) ; ( void ) memset ( ( char * ) &
blkInfoSLSize , 0 , sizeof ( struct _ssBlkInfoSLSize ) ) ; ssSetBlkInfo2Ptr (
rtS , & blkInfo2 ) ; ssSetBlkInfoSLSizePtr ( rtS , & blkInfoSLSize ) ;
ssSetMdlInfoPtr ( rtS , & mdlInfo ) ; ssSetExecutionInfo ( rtS ,
executionInfo ) ; slsaAllocOPModelData ( rtS ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; }
ssSetSolverMode ( rtS , SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS ,
( ( void * ) & rtB ) ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof
( B ) ) ; } { real_T * x = ( real_T * ) & rtX ; ssSetContStates ( rtS , x ) ;
( void ) memset ( ( void * ) x , 0 , sizeof ( X ) ) ; } { void * dwork = (
void * ) & rtDW ; ssSetRootDWork ( rtS , dwork ) ; ( void ) memset ( dwork ,
0 , sizeof ( DW ) ) ; } { static DataTypeTransInfo dtInfo ; ( void ) memset (
( char_T * ) & dtInfo , 0 , sizeof ( dtInfo ) ) ; ssSetModelMappingInfo ( rtS
, & dtInfo ) ; dtInfo . numDataTypes = 23 ; dtInfo . dataTypeSizes = &
rtDataTypeSizes [ 0 ] ; dtInfo . dataTypeNames = & rtDataTypeNames [ 0 ] ;
dtInfo . BTransTable = & rtBTransTable ; dtInfo . PTransTable = &
rtPTransTable ; dtInfo . dataTypeInfoTable = rtDataTypeInfoTable ; }
pathPlanningTest_InitializeDataMapInfo ( ) ; ssSetIsRapidAcceleratorActive (
rtS , true ) ; ssSetRootSS ( rtS , rtS ) ; ssSetVersion ( rtS ,
SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS , "pathPlanningTest" ) ;
ssSetPath ( rtS , "pathPlanningTest" ) ; ssSetTStart ( rtS , 0.0 ) ;
ssSetTFinal ( rtS , rtInf ) ; ssSetStepSize ( rtS , 0.01 ) ;
ssSetFixedStepSize ( rtS , 0.01 ) ; { static RTWLogInfo rt_DataLoggingInfo ;
rt_DataLoggingInfo . loggingInterval = ( NULL ) ; ssSetRTWLogInfo ( rtS , &
rt_DataLoggingInfo ) ; } { { static int_T rt_LoggedStateWidths [ ] = { 4 } ;
static int_T rt_LoggedStateNumDimensions [ ] = { 1 } ; static int_T
rt_LoggedStateDimensions [ ] = { 4 } ; static boolean_T
rt_LoggedStateIsVarDims [ ] = { 0 } ; static BuiltInDTypeId
rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE } ; static int_T
rt_LoggedStateComplexSignals [ ] = { 0 } ; static RTWPreprocessingFcnPtr
rt_LoggingStatePreprocessingFcnPtrs [ ] = { ( NULL ) } ; static const char_T
* rt_LoggedStateLabels [ ] = { "CSTATE" } ; static const char_T *
rt_LoggedStateBlockNames [ ] = {
"pathPlanningTest/Ackermann Kinematic Model/Integrator" } ; static const
char_T * rt_LoggedStateNames [ ] = { "" } ; static boolean_T
rt_LoggedStateCrossMdlRef [ ] = { 0 } ; static RTWLogDataTypeConvert
rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 ,
1.0 , 0 , 0.0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 0 } ; static
RTWLogSignalInfo rt_LoggedStateSignalInfo = { 1 , rt_LoggedStateWidths ,
rt_LoggedStateNumDimensions , rt_LoggedStateDimensions ,
rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , rt_LoggingStatePreprocessingFcnPtrs
, { rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert , rt_LoggedStateIdxList
} ; static void * rt_LoggedStateSignalPtrs [ 1 ] ; rtliSetLogXSignalPtrs (
ssGetRTWLogInfo ( rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . iij3drc5sy [ 0 ] ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogXFinal ( ssGetRTWLogInfo ( rtS ) ,
"xFinal" ) ; rtliSetLogVarNameModifier ( ssGetRTWLogInfo ( rtS ) , "none" ) ;
rtliSetLogFormat ( ssGetRTWLogInfo ( rtS ) , 4 ) ; rtliSetLogMaxRows (
ssGetRTWLogInfo ( rtS ) , 0 ) ; rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS
) , 1 ) ; rtliSetLogY ( ssGetRTWLogInfo ( rtS ) , "" ) ;
rtliSetLogYSignalInfo ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ;
rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ; } { static
struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS , & statesInfo2 )
; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ; } { static
ssJacobianPerturbationBounds jacobianPerturbationBounds ;
ssSetJacobianPerturbationBounds ( rtS , & jacobianPerturbationBounds ) ; } {
static ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 4 ] ;
ssSetSolverInfo ( rtS , & slvrInfo ) ; ssSetSolverName ( rtS , "ode3" ) ;
ssSetVariableStepSolver ( rtS , 0 ) ; ssSetSolverConsistencyChecking ( rtS ,
0 ) ; ssSetSolverAdaptiveZcDetection ( rtS , 0 ) ;
ssSetSolverRobustResetMethod ( rtS , 0 ) ; ssSetSolverStateProjection ( rtS ,
0 ) ; ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelLogData ( rtS , rt_UpdateTXYLogVars ) ;
ssSetModelLogDataIfInInterval ( rtS , rt_UpdateTXXFYLogVars ) ;
ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetModelDerivatives ( rtS ,
MdlDerivatives ) ; ssSetTNextTid ( rtS , INT_MIN ) ; ssSetTNext ( rtS ,
rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ; ssSetNumNonsampledZCs ( rtS ,
0 ) ; ssSetContStateDisabled ( rtS , contStatesDisabled ) ; }
ssSetChecksumVal ( rtS , 0 , 1052907474U ) ; ssSetChecksumVal ( rtS , 1 ,
658466419U ) ; ssSetChecksumVal ( rtS , 2 , 2613212608U ) ; ssSetChecksumVal
( rtS , 3 , 2291559574U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 2 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) , &
ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo (
rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS )
, ssGetTPtr ( rtS ) ) ; } slsaDisallowedBlocksForSimTargetOP ( rtS ,
mr_pathPlanningTest_GetSimStateDisallowedBlocks ) ;
slsaGetWorkFcnForSimTargetOP ( rtS , mr_pathPlanningTest_GetDWork ) ;
slsaSetWorkFcnForSimTargetOP ( rtS , mr_pathPlanningTest_SetDWork ) ; rtP .
AckermannKinematicModel_VehicleSpeedRange [ 0 ] = rtMinusInf ; rtP .
AckermannKinematicModel_VehicleSpeedRange [ 1 ] = rtInf ; rtP .
Integrator_UpperSat [ 0 ] = rtInf ; rtP . Integrator_UpperSat [ 1 ] = rtInf ;
rtP . Integrator_UpperSat [ 2 ] = rtInf ; rtP . Integrator_LowerSat [ 0 ] =
rtMinusInf ; rtP . Integrator_LowerSat [ 1 ] = rtMinusInf ; rtP .
Integrator_LowerSat [ 2 ] = rtMinusInf ; rt_RapidReadMatFileAndUpdateParams (
rtS ) ; if ( ssGetErrorStatus ( rtS ) ) { return rtS ; } return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
const int_T gblParameterTuningTid = 2 ; void MdlOutputsParameterSampleTime (
int_T tid ) { MdlOutputsTID2 ( tid ) ; }
