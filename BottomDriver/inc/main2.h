// Headers
#include <stdbool.h>
#include <stdint.h>

#include "F28x_Project.h"
#include "f2803xpwmdac_PM37.h"
#include "IDDK_PM_Servo_F2837x-Settings.h"
#include "svgen_dq.h"
#include "bissc.h"
#include <string.h>



// TODO Defines, variables and structures
//



// Position of Renishaw ring
typedef struct
{
    const char Normal;
    const char Reversed;

} RotationModeEnum;

RotationModeEnum RotationMode = {0, 1};

typedef struct
{
    const char Closed;
    const char Closing;
    const char Opened;

} ArretirStatusEnum;

ArretirStatusEnum ArretirStatus = {1, 2, 3};

typedef struct
{
    const char Normal;
    const char BeginReload;
    const char EndReload;
    const char RecieveFirstPart;
    const char RecieveSecondPart;
    const char RecieveThirdPart;

    const char NotReadyToReload;
    const char ReadyToReload;
    const char ReloadIsEnded;
    const char FirstPartIsRecieved;
    const char SecondPartIsRecieved;
    const char ThirdPartIsRecieved;

} SciOperatingModeEnum;

SciOperatingModeEnum SciOperatingMode =
{ 0, 4, 5, 1, 2, 3,
  0, 4, 5, 1, 2, 3 };


typedef struct
{
    const unsigned char Beginning;
    const unsigned char MyAddress;
    const unsigned char HisAddress;
    const unsigned char End;
    const unsigned char MessageLength;
} SciCommandEnum;

SciCommandEnum SciCommand = { 0xAA, 0xA2, 0xB1, 0xFF, 32 };

typedef struct
{
    unsigned char   myAddress;

    unsigned char targeting;
    unsigned char arretirStatus;
    unsigned char holdPosition;
    unsigned char stabilization;
    unsigned char bridgeStatus;
    unsigned char rxMassageIsCorrect;

    unsigned char   heatStatus;
    unsigned char   operatingMode;
    unsigned char   target0;
    unsigned char   target1;
    unsigned char   target2;
    unsigned char   target3;
    int32           target;
    unsigned char   velocityXLo;
    unsigned char   velocityXHi;
    unsigned char   velocityYLo;
    unsigned char   velocityYHi;
    unsigned char   velocityZLo;
    unsigned char   velocityZHi;
    unsigned char   accelerationXLo;
    unsigned char   accelerationXHi;
    unsigned char   accelerationYLo;
    unsigned char   accelerationYHi;
    unsigned char   accelerationZLo;
    unsigned char   accelerationZHi;

    unsigned char   memsAngleXHigh;
    unsigned char   memsAngleXLow;
    long memsAngleX;
    unsigned char   memsAngleYHigh;
    unsigned char   memsAngleYLow;
    long memsAngleY;
    unsigned char   memsAngleZHigh;
    unsigned char   memsAngleZLow;
    long memsAngleZ;

    unsigned char   binsAngleXHigh;
    unsigned char   binsAngleXMid;
    unsigned char   binsAngleXLow;
    long   binsAngleX;

    unsigned char   binsAngleYHigh;
    unsigned char   binsAngleYMidHigh;
    unsigned char   binsAngleYMidLow;
    unsigned char   binsAngleYLow;
    long   binsAngleY;

    unsigned char   binsAngleZHigh;
    unsigned char   binsAngleZMid;
    unsigned char   binsAngleZLow;
    long   binsAngleZ;



    unsigned short   firmwareLength0;
    unsigned short   firmwareLength1;
    unsigned short   firmwareLength;

    unsigned short   numberOfPacket0;
    unsigned short   numberOfPacket1;
    unsigned short   numberOfPacket;
    unsigned short   oldNumberOfPacket;

    unsigned char   eeprom;

    unsigned char   otherTarget0;
    unsigned char   otherTarget1;
    unsigned char   otherTarget2;
    unsigned char   otherTarget3;
    int32           otherTarget;

} SciRxCommandStruct;

SciRxCommandStruct  sciRxCommand = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

typedef struct
{

    unsigned char targeting;
    unsigned char holdPosition;
    unsigned char stabilization;
    unsigned char bridgeStatus;

    unsigned char  operatingMode;
    unsigned char  positionOfRenishaw0;
    unsigned char  positionOfRenishaw1;
    unsigned char  positionOfRenishaw2;
    unsigned char  positionOfRenishaw3;
    long           positionOfRenishaw;
    unsigned char  velocityHi;
    unsigned char  velocityLo;
    unsigned char  accelerationHi;
    unsigned char  accelerationLo;

    unsigned char  accuracy;
    unsigned char  targetingStatus;
    unsigned char  firmwareVersion;
    unsigned char  heatStatus;

    unsigned char numberOfPacket0;
    unsigned char numberOfPacket1;
    unsigned int numberOfPacket;

    unsigned char  eeprom;

} SciTxCommandStruct;

SciTxCommandStruct sciTxCommand = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct
{
    _iq pPart;
    _iq iPart;
    _iq dPart;
    _iq wholePart;

    _iq iPartLong;  // long integral part (to see smaller movement changes)
    _iq iPartLimit;

    _iq pCoefLo;
    _iq iCoefLo;
    _iq dCoefLo;

    _iq pCoefHi;
    _iq iCoefHi;
    _iq dCoefHi;

} PidStruct;

PidStruct pid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct
{
    int32   electricalMotorNilPosition;
    int     mechToElectrCoef;     
    int32 phaseDivider;       
    int32  maxPhase;

    int32   globalPhase;
    int32   volatile   globalPhaseToGo;


    _iq  volatile phase;

    int16 pidPhase;
} PhaseStruct;

PhaseStruct phase = {0, 0, 0, 0, 0, 0, 0, 0};

typedef struct
{
    int readFlag;
    bool writeFlag;     //writeFlag
    bool oldWriteFlag;
    Uint16 error;

    int position;               // positionBottom
    int keepPositionFlag1;      // state
    int keepPositionFlag2;      // noChange
    int writeCounter;
    int positionIsRecieved;
} EepromStruct;

EepromStruct eeprom = {0, 0, 0, 0, 0, 0, 0, 0, 0};


typedef struct
{
    int32   positionOfRenishaw;
    int32   positionOfRenishawLong;

    Uint32  absolutePositionOfRenishaw;
    Uint32  absolutePositionOfRenishawLong;

    int32   positionOfTargetDesignation;

    Uint32  positionOfArretir;
    int32   positionOfArretirLong;

    Uint32  stopper1;
    Uint32  stopper2;

    Uint32 topStopper;
    Uint32 bottomStopper;
    Uint32 zeroPosition;

    long    velocity;
    Uint16  averageVelocity;

    Uint16  absolutePositionOfRenishawStepBack;
    long    deltaAngleForStabilization;

} PositionStruct;

PositionStruct position = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


// Rotation around during heating
typedef struct
{
    char mode;
    unsigned long timer;
    char direction;
    long tmpPosition;

} HeatRotationStruct;

HeatRotationStruct heatRotation = {0, 0, 1, 0};

// Find position at start
typedef struct
{
    char arretirPositionCounter;
    _iq pidSpeed;
    char mode;
    char firstPointWasFound;
    char secondPointWasFound;
    char counterWasCalculated;
    char atArretirPoint;
    char wasAtArretirPoint;
    int velocityCounter;
    int stopTime;


} FindPositionStruct;

FindPositionStruct findPosition = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 20000 };


// Command to find a overrotate while entangled
typedef struct
{
    char rotationCommand;
    char oldRotationCommand;
    char rotationFlag;
    char firstRotationPoint;
    char secondRotationPoint;
} TechnologicalRotationStruct;

TechnologicalRotationStruct technologicalRotation = {0, 0, 0, 0, 0 };



// Target division
// TargetDesignation comes by 100 Hz
// Each message is splitted to 10 targets
// that are reached consequently
// So that, we have smoother movement
#define TARGET_DIVIDER 10

typedef struct
{
    int32 oldPositionOfTargetDesignation;
    int32 newPositionOfTargetDesignation [TARGET_DIVIDER];
    float deltaPositionOfTargetDesignation;
    Uint32 positionOfTargetDesignationCounter;


} TargetDividerStruct;

TargetDividerStruct targetDivider = { 0, 0, 0, 0 };


typedef struct
{
    _iq Max;
    _iq HoldPosition;
    _iq Heating;
} AmplitudeStruct;

AmplitudeStruct Amplitude = { 0, 0, 0 };

typedef struct
{
    long cpuTimer;
    long i2cTimer;
    long globalTimer;
} TimerStruct;

TimerStruct timer = { 0, 0, 0 };

typedef struct
{
    char turnOn;
    char turnOff;
    char mode;
    char resetArretirStatus;
    char resetPositionOfTargetDesignation;
    char resetPidParts;
    char resetHoldPosition;
} TechnologicalMode;

TechnologicalMode technologicalMode = { 0, 0, 0, 0, 0, 0, 0 };

////////////////////////////////////////////////////////////////////////////////////////////////////
bool rotationMode;


Uint16    PositioningAccuracy1;
Uint16    PositioningAccuracy2;
Uint16    PositioningAccuracy3;
Uint16    PositioningAccuracy4;
Uint16    PositioningAccuracy5;


_iq SpeedRef1 = 0, SpeedRef2 = 0, SpeedRef3 = 0, SpeedRef4 = 0, SpeedRef5 = 0, SpeedRef6 = 0;

_iq VqTestingNormal = 0, VqTestingCold = 0, VqTestingVeryCold = 0, VqTestingHot = 0, VqTestingHeating = 0;

_iq amplitude = 0;

int sign;


unsigned long tmpCounter[10];
unsigned int tmpFlag[10];

Uint16 sciRxBufferArray[32];
Uint16 sciRxBufferCounter = 0;

unsigned char wasArretired = 1;
unsigned int arretirCounter = 0;




char i2cFlag = 0;

_iq a1 = 0;
_iq a2 = 0;
_iq a3 = 0;


#define I2C_SLAVE_ADDR        0x50
#define I2C_NUMBYTES          2
#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x01

//int positionBottom;
//int state;
//int noChange;
//int eeprom.counterWrite;

struct I2CMSG *CurrentMsgPtr;
struct I2CMSG I2cMsgOut = { I2C_MSGSTAT_SEND_WITHSTOP,
                            I2C_SLAVE_ADDR,
                            I2C_NUMBYTES,
                            I2C_EEPROM_HIGH_ADDR,
                            I2C_EEPROM_LOW_ADDR};

struct I2CMSG I2cMsgIn = { I2C_MSGSTAT_SEND_NOSTOP,
                           I2C_SLAVE_ADDR,
                           I2C_NUMBYTES,
                           I2C_EEPROM_HIGH_ADDR,
                           I2C_EEPROM_LOW_ADDR};




static _iq Ampl2 = _IQ(0.0);

static _iq     PID_Vq = _IQ(0.15);


static _iq tmpPID = 0;


unsigned int velocityCounter = 0;
int summaryVelocity = 0;



float MyFloat;


Uint32  IsrTickerForPID_I_Part = 0;
Uint16  DoPID_I_PartToNull = 0;

_iq angle = 0;
Uint32 CounterForPID_I_PartForNull = 0;

#define TMP100_SLAVE            0x49 //TMP102   //0x48    // slave address TMP100 (ADDR0=ADDR1=0)
#define DS1624_SLAVE            0x4C
#define POINTER_TEMPERATURE     0
#define POINTER_CONFIGURATION   1
#define POINTER_T_LOW           2
#define POINTER_T_HIGH          3

int temperature = -127;    // temperature = 2' Complement of temperature (-128 ... +127 Celsius)
int temperatureDS1624 = -127;
char heatStatus = 2;

Uint32  TimerTempCounter = 0;
int32 counterForI2c = 0;

Uint16    positiveDirection = 1;
Uint16    definitionOfOrientation = 0;

Uint32    FalseDefinitionOfOrientationCounter = 0;



int32     ManagmentOffset = 0;

Uint16    DRV_OFF = 0;


//-----//----

Uint16 resetDriver = 1; // On/Off motor


int16 Znak = 1;

Uint32 myPosition = 0;

Uint16 sciRxData = 0;
unsigned char sciRxArray[32];

unsigned char sciTxArray[32];



typedef struct {
    _iq ElecTheta;       // Output: Motor Electrical angle (Q24)
    _iq MechTheta;       // Output: Motor Mechanical Angle (Q24)
    _iq RawTheta;        // Variable: Raw position data from resolver (Q0)
    _iq Speed;           // Variable: Speed data from resolver (Q4)
    _iq InitTheta;       // Parameter: Raw angular offset between encoder index and phase a (Q0)
    _iq MechScaler;      // Parameter: 0.9999/total count (Q30)
    _iq StepsPerTurn;    // Parameter: Number of discrete positions (Q0)
    Uint16 PolePairs;       // Parameter: Number of pole pairs (Q0)

}  ABS_ENCODER;
/*-----------------------------------------------------------------------------
Default initializer for the Encoder Object.
-----------------------------------------------------------------------------*/
#define ABSENC_DEFAULTS {                                                   \
        0x0,            /*  ElecTheta    - (Q24)  */   \
        0x0,            /*  MechTheta    - (Q24)  */   \
        0x0,            /*  RawTheta     - (Q0)   */   \
        0x0,            /*  Speed        - (Q4)   */   \
        0x0,            /*  InitTheta    - (Q0)   */   \
        0x00020000,     /*  MechScaler   - (Q30)  */   \
        0x0,            /*  StepsPerTurn - (Q0)   */   \
        8,              /*  PolePairs    - (Q0)   */   \
        }



#define StatusCOM  0xC0
#define Driver     0xB1
#define Komp       0xA1
#define Velocity   0xC1
#define Limit      0xC2
#define Targeting  0xD0
#define lenght_Com  8
#define Finish     0xFF
#define Plus     1
#define Minus   0


typedef struct{
    uint32_t  data;
    uint32_t  data1;
}My_Biss;
My_Biss My_Biss1 = {0,0};



typedef struct
{
    unsigned int  accuracy;
    int32  position;
    Uint32  position32;
} My_Parametr;

My_Parametr My_Parametr1 = {20000, 0, 0};


volatile struct EPWM_REGS *ePWM[] = {
                                     &EPwm1Regs,            //intentional: (ePWM[0] not used)
                                     &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
                                     &EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs};

int16   VTimer0[4];         // Virtual Timers slaved off CPU Timer 0 (A events)
int16   VTimer1[4];         // Virtual Timers slaved off CPU Timer 1 (B events)
int16   VTimer2[4];         // Virtual Timers slaved off CPU Timer 2 (C events)
int16   SerialCommsTimer;


///int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};



SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;


// ****************************************************************************
// Flag variables
// ****************************************************************************
volatile Uint16 EnableFlag = 0;


Uint32 motorIsrTicker = 0;  // РЎС‡РµС‚С‡РёРє РїСЂРµСЂС‹РІР°РЅРёСЏ РЁР�РњР° (10 РєР“С†)

Uint16 BackTicker = 0,
        lsw = 0,
        TripFlagDMC = 0,                //PWM trip status
        clearTripFlagDMC = 0;



//// ****************************************************************************
//// Variables for Field Oriented Control
//// ****************************************************************************
float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h
_iq VdTesting = _IQ(0.0),           // Vd reference (pu)
        vqTesting = _IQ(0.03),//_IQ(0.1819),          // Vq reference (pu)
        IdRef     = _IQ(0.0),           // Id reference (pu)
        IqRef     = _IQ(0.08),          // Iq reference (pu)
        MyIQ= _IQ(0.0),
        SpeedRef  = _IQ(0);           // For Closed Loop tests
_iq cal_offset_A = _IQ15(0.5076);
_iq cal_offset_B = _IQ15(0.5055);
_iq cal_filt_gain;

//  Instance a position estimator
SMOPOS smo1 = SMOPOS_DEFAULTS;

// Instance a sliding-mode position observer constant Module
SMOPOS_CONST smo1_const = SMOPOS_CONST_DEFAULTS;

// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK   park1   = PARK_DEFAULTS;
IPARK  ipark1  = IPARK_DEFAULTS;

PID_GRANDO_CONTROLLER pid1_id = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_iq = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER pid1_spd = {PID_TERM_DEFAULTS,PID_PARAM_DEFAULTS,PID_DATA_DEFAULTS};

typedef struct {
    Uint16 PeriodMax;     // Parameter: PWM Half-Period in CPU clock cycles (Q0)
    int16 MfuncPeriod;    // Input: Period scaler (Q15)
    int16 MfuncC1;        // Input: EPWM1 A&B Duty cycle ratio (Q15)
    int16 MfuncC2;        // Input: EPWM2 A&B Duty cycle ratio (Q15)
    int16 MfuncC3;        // Input: EPWM3 A&B Duty cycle ratio (Q15)
    int16 PWM1out;
    int16 PWM2out;
    int16 PWM3out;
} PWMGEN ;

/*-----------------------------------------------------------------------------
    Define a PWMGEN_handle
-----------------------------------------------------------------------------*/
typedef PWMGEN *PWMGEN_handle;

/*------------------------------------------------------------------------------
    Default Initializers for the F280X PWMGEN Object
------------------------------------------------------------------------------*/
#define F280X_FC_PWM_GEN    { 5000,   \
        0x7FFF, \
        0x4000, \
        0x4000, \
        0x4000, \
        0x4000, \
        0x4000, \
        0x4000, \
}


#define PWMGEN_DEFAULTS     F280X_FC_PWM_GEN
int16 MPeriod;
int32 Tmp;

PWMGEN pwm1 = PWMGEN_DEFAULTS;

SVGENDQ svgen_dq1 = SVGENDQ_DEFAULTS;
// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RMPCNTL rc2 = RMPCNTL_DEFAULTS; // for speed

//  Instance a ramp generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//  Instance a phase voltage calculation
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;
Uint16 SpeedLoopPrescaler = 10;      // Speed loop pre scalar
Uint16      SpeedLoopCount = 1;
// Instance a speed calculator based on Encoder position
//SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

_iq  cntr=0,
        alignCnt = 20000;
_iq  IdRef_start = _IQ(0.0),
        IdRef_run   = _IQ(0.0);

// Variables for position reference generation and control
// =========================================================
_iq   posArray[8] = { _IQ(1.5), _IQ(-1.5), _IQ(2.5), _IQ(-2.5) },
        cntr1=0 ,
        posSlewRate = _IQ(0.001);

int16 ptrMax = 2,
        ptr1=0;
volatile ABS_ENCODER biss1 = ABSENC_DEFAULTS;

DLOG_4CH_F dlog_4ch1;
int16 PwmDacCh1=0;
int16 PwmDacCh2=0;
int16 PwmDacCh3=0;
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;
_iq posEncElecTheta[6],
posEncMechTheta[6];

unsigned char SectrMy=0;

//definitions for selecting DACH reference
#define REFERENCE_VDDA     0

//definitions for COMPH input selection
#define NEGIN_DAC          0
#define NEGIN_PIN          1

//definitions for CTRIPH/CTRIPOUTH output selection
#define CTRIP_ASYNCH       0
#define CTRIP_SYNCH        1
#define CTRIP_FILTER       2
#define CTRIP_LATCH        3

Uint32 ManagmentOffsetLocal = 0;


// Reload
#pragma DATA_SECTION(firmwareBuffer, "FirmwareBufferSection");
Uint16 firmwareBuffer[20000];

#pragma DATA_SECTION(firmwareLengthInRam, "FirmwareLengthSection");
Uint16 firmwareLengthInRam = 0;
Uint16 firmwareLength = 0;

#pragma DATA_SECTION(firmwareSumInRam, "FirmwareSumSection");
Uint16 firmwareSumInRam = 0;
Uint16 firmwareSum = 0;

Uint16 firmwareCounter = 0;

#define JUMP_TO_FLASHAPI asm("    JMP 0x080000")

/// Reload

#ifndef ON
#define ON 1
#define OFF 0
#endif


//
// TODO Prototypes
//

#pragma INTERRUPT (ResolverIsr, HPI)
#pragma INTERRUPT (MotorControlIsr, LPI)

void ConfigureEeprom(void);
Uint16 I2CA_ReadData(struct I2CMSG *msg);
Uint16 I2CA_WriteData(struct I2CMSG *msg);

void CheckPositionForEeprom(void);

interrupt void MotorControlIsr(void);
interrupt void SciRxIsr(void);
interrupt void ResolverIsr(void);
interrupt void CpuTimerIsr(void);
interrupt void I2cEepromIsr(void);
void Wait(long delay);

void SwitchKomandRx();
void SciRx(Uint16 sciBuffer);
void SciPrepareTxData();

void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);

void DeviceInit();
void ConfigureADC(void);
void DataBiss(void);

void SetMotorPosition();
void ReadRenishaw();
void SetPositionOfRenishaw();
void SetMotorPwm();

void InitI2c(void);

_iq refPosGen(_iq out);
_iq ramper(_iq in, _iq out, _iq rampDelta);

void A0(void);  //state A0
void B0(void);  //state B0
void C0(void);  //state C0

// A branch states
void A1(void);  //state A1
void A2(void);  //state A2
void A3(void);  //state A3

// B branch states
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3

// C branch states
void C1(void);  //state C1
void C2(void);  //state C2
void C3(void);  //state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

void My_Com_Tx32(unsigned char flagswitch,unsigned char lenght, unsigned char *mass);
void SwitchKomand(unsigned char flagswitch,unsigned char *mass);

void InitSci(void);

void InitSystem();
void InitVariables();
void InitPwm();
void InitEveryGpioWeNeed();
void InitMotor();
void InitBissc();
void InitInterrupts();
void StartCpuTimer();
void ReadTmp100();
void ReadFromEeprom();
void WriteToEeprom();
void CheckPwm();
void CheckSci();
void FindPosition();

void SetTarget(void);
void SciParseNormal(void);
void SciParseBeginReload(void);
void SciParseRecieveThirdPart();
void SciParseEndReload();
void SciParseRecieveFirstPart();
void SciParseRecieveSecondPart();
void SciPrepareNormal();
void SciParseReload();
void SciPrepareReload();

void ClearMotorInterrupt(void);
void CalculateAverageVelocity();

void CheckTechnologicalMode();

long Stabilization(float inputAlpha, float inputBeta);
float DegreeToRadian(float degree);
float RadianToDegree(float radian);
//
// TODO Rarely used functions
//

#define PWM_MACRO(v)                                                                        \
        \
        /* Compute the timer period (Q0) from the period modulation input (Q15)*/                   \
        \
        Tmp = (int32)v.PeriodMax*(int32)v.MfuncPeriod;          /* Q15 = Q0*Q15 */              \
        MPeriod = (int16)(Tmp>>16) + (int16)(v.PeriodMax>>1);   /*Q0 = (Q15->Q0)/2 + (Q0/2)*/   \
        EALLOW;                                                                                  \
        EPwm1Regs.TBPRD = MPeriod;                                                              \
        EPwm2Regs.TBPRD = MPeriod;                                                              \
        EPwm3Regs.TBPRD = MPeriod;                                                              \
        EDIS;                                                                            \
        /*Compute the compare value (Q0) from the related duty cycle ratio (Q15)*/                  \
        \
        Tmp = (int32)MPeriod*(int32)v.MfuncC1;              /* Q15 = Q0*Q15 */                  \
        v.PWM1out = (int16)(Tmp>>16) + (int16)(MPeriod>>1); /*Q0 = (Q15->Q0)/2 + (Q0/2)*/       \
        \
        /*Compute the compare value (Q0) from the related duty cycle ratio (Q15)*/                  \
        \
        Tmp = (int32)MPeriod*(int32)v.MfuncC2;              /* Q15 = Q0*Q15 */                  \
        v.PWM2out= (int16)(Tmp>>16) + (int16)(MPeriod>>1);  /*Q0 =  (Q15->Q0)/2 + (Q0/2)*/       \
        \
        /*Compute the compare value (Q0) from the related duty cycle ratio (Q15)*/                  \
        \
        Tmp = (int32)MPeriod*(int32)v.MfuncC3;              /* Q15 = Q0*Q15 */                  \
        v.PWM3out= (int16)(Tmp>>16) + (int16)(MPeriod>>1);  /*Q0 = (Q15->Q0)/2 + (Q0/2)*/


void PosEncoderSuite(void)
{
    // ----------------------------------
    // lsw = 0 ---> Alignment Routine
    // ----------------------------------


    //Read position data in EnDat21 mode. Function defined in endat.c
    if(bissc_data_struct.dataReady == 1)
    {
        bissc_readPosition();
    }

    biss1.RawTheta = (_IQ(1.0)-(bissc_data_struct.position)/4);

    // during alignment, assign the current shaft position as initial position
    if (lsw == 0)
    {
        biss1.InitTheta = biss1.RawTheta;
    }

    biss1.MechTheta   = biss1.RawTheta - biss1.InitTheta;      /* MechTheta in step counts */
    if(biss1.MechTheta < 0)
        biss1.MechTheta = biss1.MechTheta + 1.0;
    else if (biss1.MechTheta > 1.0)
        biss1.MechTheta = biss1.MechTheta - 1.0;

    // Compute the electrical angle in Q24
    biss1.ElecTheta  = _IQfrac(biss1.PolePairs * biss1.MechTheta);

    // Reverse the sense of position if needed - comment / uncomment accordingly
    // Position Sense as is
    //posEncElecTheta[BISS_POS_ENCODER] = biss1.ElecTheta;
    // posEncMechTheta[BISS_POS_ENCODER] = biss1.MechTheta;

    // Position Sense Reversal
    posEncElecTheta[BISS_POS_ENCODER] = 1.0 - biss1.ElecTheta;
    posEncMechTheta[BISS_POS_ENCODER] = 1.0 - biss1.MechTheta;

    return;
}

void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db) {
    EALLOW;
    // Time Base SubModule Registers
    (*ePWM[n]).TBCTL.bit.FREE_SOFT=10;
    (*ePWM[n]).TBCTL.bit.PHSDIR=1;

    (*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load

    (*ePWM[n]).TBPRD =1000;    // PWM frequency = 1 / period
    (*ePWM[n]).TBPHS.bit.TBPHS = 0;
    (*ePWM[n]).TBCTR = 0;
    (*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
    (*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
    (*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;

    (*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
    //   (*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"
    (*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    // Counter Compare Submodule Registers
    (*ePWM[n]).CMPA.bit.CMPA = 0; // set duty 0% initially
    (*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    // (*ePWM[n]).TZSEL.all=0x0600;

    (*ePWM[n]).AQCTLA.all=0x0060;

    (*ePWM[n]).DBCTL.all=0x0000;

    (*ePWM[n]).DBRED.all = 0;
    (*ePWM[n]).DBFED.all = 0;
    EDIS;
}

// slew programmable ramper
_iq ramper(_iq in, _iq out, _iq rampDelta) {
    _iq err;

    err = in - out;
    if (err > rampDelta)
        return(out + rampDelta);
    else if (err < -rampDelta)
        return(out - rampDelta);
    else
        return(in);
}

// Ramp Controller for speed reference (Not currently used)
_iq ramper_speed(_iq in, _iq out, _iq rampDelta) {
    _iq err;

    err = in - out;
    if (err > rampDelta)
    {
        if((out+rampDelta)>1.0)
            return(1.0);
        else
            return (out+rampDelta);
    }
    else if (err < -rampDelta)
    {
        if(out-rampDelta<=0.0)
            return(0.0);
        else
            return(out - rampDelta);
    }
    else
        return(in);
}

// Reference Position Generator for position loop
_iq refPosGen(_iq out)
{
    _iq in = posArray[ptr1];

    out = ramper(in, out, posSlewRate);

    if (in == out)
        if (++cntr1 > 1000)
        {
            cntr1 = 0;
            if (++ptr1 >= ptrMax)
                ptr1 = 0;
        }
    return (out);
}





void ConfigureDS1624(void)
{
    I2caRegs.I2CSAR.all = DS1624_SLAVE;
    I2caRegs.I2CCNT = 2;
    I2caRegs.I2CDXR.all = 0xAC;//POINTER_CONFIGURATION;
    I2caRegs.I2CMDR.all = 0x6E20;

    while ((I2caRegs.I2CSTR.bit.XRDY == 0)&&(counterForI2c<10000)) counterForI2c++;   // wait for 1st byte
    if (I2caRegs.I2CSTR.bit.XRDY != 0)
    {
        I2caRegs.I2CDXR.all = 0x00;//0x60;
    }

    counterForI2c = 0;
    while ((I2caRegs.I2CSTR.bit.SCD == 0)&&(counterForI2c<10000)) counterForI2c++;   // wait for 1st byte
    if (I2caRegs.I2CSTR.bit.SCD != 0)
    {
        I2caRegs.I2CSTR.bit.SCD = 1;
    }

    Wait(1);

    // Send START, set pointer to Configuration register and set resolution to 12 bit
    I2caRegs.I2CSAR.all = DS1624_SLAVE;
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.all = 0xEE;//POINTER_CONFIGURATION;
    I2caRegs.I2CMDR.all = 0x6E20;

    counterForI2c = 0;
    while ((I2caRegs.I2CSTR.bit.SCD == 0)&&(counterForI2c<10000)) counterForI2c++;   // wait for 1st byte
    if (I2caRegs.I2CSTR.bit.SCD != 0)
    {
        I2caRegs.I2CSTR.bit.SCD = 1;
    }
    Wait(1);
}

void PrepareToReadDS1624(void)
{
    I2caRegs.I2CSAR.all = DS1624_SLAVE;
    I2caRegs.I2CCNT     = 1;                // pointer to temperature register
    I2caRegs.I2CDXR.all     = 0xAA;//POINTER_TEMPERATURE;
    // Send start as master transmitter
    I2caRegs.I2CMDR.all = 0x6E20;

    counterForI2c = 0;
    while ((I2caRegs.I2CSTR.bit.SCD == 0)&&(counterForI2c<10000)) counterForI2c++;   // wait for 1st byte
    if (I2caRegs.I2CSTR.bit.SCD != 0)
    {
        I2caRegs.I2CSTR.bit.SCD = 1;
    }
}

void ReadDS1624(void)
{

    I2caRegs.I2CCNT     = 2;    // read 2 byte temperature
    I2caRegs.I2CMDR.all = 0x6C20;

    counterForI2c = 0;
    while ((I2caRegs.I2CSTR.bit.RRDY == 0)&&(counterForI2c<10000)) counterForI2c++;   // wait for 1st byte
    if (I2caRegs.I2CSTR.bit.RRDY != 0)
    {
        temperatureDS1624 = I2caRegs.I2CDRR.all << 8;
    }


    counterForI2c = 0;
    while ((I2caRegs.I2CSTR.bit.RRDY == 0)&&(counterForI2c<10000)) counterForI2c++;   // wait for 1st byte
    if (I2caRegs.I2CSTR.bit.RRDY != 0)
    {
        temperatureDS1624 += I2caRegs.I2CDRR.all;
    }

    temperatureDS1624 /= 256;

}


void ConfigureEeprom(void)
{
    I2caRegs.I2CSAR.all = 0x0050;     // Slave address - EEPROM control code


    I2caRegs.I2CPSC.all = 149;          // Prescaler - need 7-12 Mhz on module clk
    I2caRegs.I2CCLKL = 95;            // NOTE: must be non zero
    I2caRegs.I2CCLKH = 95;             // NOTE: must be non zero
    I2caRegs.I2CIER.all = 0x24;       // Enable SCD & ARDY __interrupts

    I2caRegs.I2CMDR.all = 0x0020;     // Take I2C out of reset
    // Stop I2C when suspended

    I2caRegs.I2CFFTX.all = 0x6000;    // Enable FIFO mode and TXFIFO
    I2caRegs.I2CFFRX.all = 0x2040;    // Enable RXFIFO, clear RXFFINT,


    return;
}

Uint16 I2CA_ReadData(struct I2CMSG *msg)
{
    if(I2caRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }

    I2caRegs.I2CSAR.all = msg->SlaveAddress;

    if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
    {
        if(I2caRegs.I2CSTR.bit.BB == 1)
        {
            return I2C_BUS_BUSY_ERROR;
        }

        I2caRegs.I2CCNT = 2;
        I2caRegs.I2CDXR.all = msg->MemoryHighAddr;
        I2caRegs.I2CDXR.all = msg->MemoryLowAddr;
        I2caRegs.I2CMDR.all = 0x2620; // Send data to setup EEPROM address
    }
    else if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
    {
        I2caRegs.I2CCNT = 3;    // Setup how many bytes to expect
        I2caRegs.I2CMDR.all = 0x2C20;         // Send restart as master receiver
    }
    return I2C_SUCCESS;
}

Uint16 I2CA_WriteData (struct I2CMSG *msg)
{
    Uint16 i;

    if(I2caRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }

    I2caRegs.I2CSAR.all = msg->SlaveAddress;

    if(I2caRegs.I2CSTR.bit.BB == 1)
    {
        return I2C_BUS_BUSY_ERROR;
    }

    I2caRegs.I2CCNT = 3+2;
    I2caRegs.I2CDXR.all = msg->MemoryHighAddr;
    I2caRegs.I2CDXR.all = msg->MemoryLowAddr;

    for (i=0; i < 3; i++)
    {
        I2caRegs.I2CDXR.all = *(msg->MsgBuffer+i);
    }
    I2caRegs.I2CMDR.all = 0x6E20;
    return I2C_SUCCESS;
}

interrupt void I2cEepromIsr(void)
{
    Uint16 IntSource, i;

    IntSource = I2caRegs.I2CISRC.all;
    if(IntSource == I2C_SCD_ISRC)
    {
        if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
        {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
        }
        else
        {
            if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
            {
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
            }
            else if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)
            {

                for(i = 0; i < 3; i++)
                {
                    CurrentMsgPtr->MsgBuffer[i] = I2caRegs.I2CDRR.all; //reading
                }
                eeprom.position = CurrentMsgPtr->MsgBuffer[0];
                eeprom.keepPositionFlag1 = CurrentMsgPtr->MsgBuffer[1];
                eeprom.keepPositionFlag2 = CurrentMsgPtr->MsgBuffer[2];
                if( eeprom.position == 255 || eeprom.position > 3)
                {
                    eeprom.position = 1;
                    eeprom.keepPositionFlag1 = 0;
                    eeprom.keepPositionFlag2 = 0;
                }

                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
                {
                }
            }
        }
    }

    else if(IntSource == I2C_ARDY_ISRC)
    {
        if(I2caRegs.I2CSTR.bit.NACK == 1)
        {
            I2caRegs.I2CMDR.bit.STP = 1;
            I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
        }
        else if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
        {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;
        }
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

