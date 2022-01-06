int messageRx[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int messageTx[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int sciCounter = 0;
long txDelayer = 0;

#include <stdbool.h>
#include <stdint.h>

#include "F28x_Project.h"
#include "f2803xpwmdac_PM37.h"
#include "IDDK_PM_Servo_F2837x-Settings.h"
#include "svgen_dq.h"

#include "bissc.h"

//#define _FLASH 1
#ifdef _FLASH
//#pragma CODE_SECTION(MotorControlISR,"ramfuncs");
//#pragma CODE_SECTION(ResolverISR,"ramfuncs");
#endif

#define Zaar 1
//#define Razaar 0
#define Razaar 3

//------**TMP100
// TMP100 commands
#define TMP100_SLAVE            0x49 //TMP102   //0x48    // slave address TMP100 (ADDR0=ADDR1=0)
#define POINTER_TEMPERATURE     0
#define POINTER_CONFIGURATION   1
#define POINTER_T_LOW           2
#define POINTER_T_HIGH          3

int temperature;    // temperature = 2' Komplement of temperature (-128 ... +127 Celsius)
// is an I8Q8 - Value
Uint32  TimerTempCounter = 0;
//------**TMP100




#pragma INTERRUPT (ResolverISR, HPI)
#pragma INTERRUPT (MotorControlISR, LPI)


//-----//----
Uint16    PositionOfReanishowUint16 = 0;      //выравниваем по арретиру, знак определ€ем по положению р€дом с новым нулем
int32     PositionOfReanishowInt32 = 0;       //при переходе через ноль необходимо более 16 бит
Uint16    PositiveDirection = 1;              //работаем в позитивном направлении при переходе через ноль
Uint16    DefinitionOfOrientation = 0;        //определенность в понимании в какой четверти находимс€
///Uint16    PositionOfArretirUint16 = 59929;    //положение арретира в отсчетах Reniashow 16 бит
Uint16    PositionOfArretirUint16 = 27273;    //положение арретира в отсчетах Reniashow 16 бит
Uint32    FalseDefinitionOfOrientationCounter = 0;  //счетчик 10к√ц времени когда положение двигателей не определено

Uint16    PositionOfTargetDesignationUint16 = 32768;
int32     PositionOfTargetDesignationInt32 = 0;

int32     ManagmentOffset = 0;

Uint16    DRV_OFF = 0;                        //выключение драйвера

Uint16    PositioningAccuracy1 = 3;
Uint16    PositioningAccuracy2 = 300;
Uint16    PositioningAccuracy3 = 600;
Uint16    PositioningAccuracy4 = 5000;
//-----//----

Uint16 DRV_RESET = 1; //драйвер выключен

#define Sign -1; //направление движени€
int16 Znak = 1;

Uint32 My_Position = 0; //показани€ датчиков Reanishow с учетом сдвига

Uint16 Bukva = 0;       //прием данных по COM-порту (почему-то глобальна€?)
unsigned char My_Com_32_Tx[32];     //массив передачи COM-порту (почему-то глобальный?)

#define com16 1
#define com32 2
//unsigned char F_COM=com16;
unsigned char F_COM=com32;


// Prototype statements for functions found within this file.
interrupt void MotorControlISR(void);
interrupt void SCIB_RX_isr(void);
interrupt void ResolverISR(void);

void SwitchKomand32_Rx(unsigned char flagswitch);
void SCIB_RX_32(Uint16 bukva,unsigned char start1, unsigned char start2, unsigned char finish);
void SwitchKomand32_Tx(unsigned char flagswitch);

void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);

void DeviceInit();
void ConfigureADC(void);
void DataBiss(void);    //процедура определени€ My_Position по датчикам Reanishow

void My_Function_Position(unsigned char flagSwitch);

void I2CA_Init(void);

// **********************************************************
// ********************* Functions **************************
// **********************************************************
_iq refPosGen(_iq out);
_iq ramper(_iq in, _iq out, _iq rampDelta);

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

        My_Komand_Com32  Komand_Com32 =defaul_My_Com32;
        My_Otvet_Com32 Otvet_Com32 =My_Komand_Com32_DEFAULTS;

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

        typedef struct  {
            unsigned char  flag_switch_F_position;
            unsigned char  flag;
        } My_Flags;
        My_Flags My_Flags1 = {0,0};

        typedef struct  {
            unsigned int  accuracy;
            //unsigned int  position;
            int  position;
            Uint32  position32;
        } My_Parametr;
        My_Parametr My_Parametr1 = {20000, 0, 0};

        typedef struct  {
            unsigned char  Switch_tx_COM;
            unsigned char  Switch_tx_COM2;
            unsigned char  Switch_tx_COM32;
            unsigned char  Switch_rx_COM;
            unsigned char  Switch_status;
            unsigned char  index;
            unsigned char  i;
            unsigned char  Switch_Otvet_COM;
            unsigned char  Switch_Otvet_COM32;
            unsigned char  u;
            unsigned char  Start;
            unsigned char  finish;
            unsigned char  Komand;
            unsigned char  flagTx;
            unsigned char  flagViv;
            unsigned char  checksum;
        } My_Flags2;
        My_Flags2 Flags = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

        typedef struct  {
            unsigned char  Bukva;
            unsigned char  Bukva1;
            unsigned char  index;
            unsigned char  i;
            unsigned char  flagMess;
            unsigned char  u;
            unsigned char  Start;
            unsigned char  finish;
            unsigned char  Komand;
            unsigned char  reserv;
            unsigned char  flagViv;
            unsigned char  checksum;
            unsigned char  Sklad[8];
            unsigned char  message1[8];
        } My_Com;
#define   defaul_My_Com  {  \
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0\
}
        My_Com My_Com1=defaul_My_Com;
#define   defaul_My_Com2  {  \
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0\
}
        My_Com My_Com2=defaul_My_Com2;

        typedef struct  {
            unsigned char  SwitchDriver;
            unsigned char  SwitchVideo;
            unsigned char  SwitchReset;
            unsigned char  Status;
            unsigned int  Freq;
            unsigned int  Target;
            unsigned int  HighLimit;
            unsigned int  LowLimit;
            unsigned int  Accuracity;
            unsigned int  velocity1;
        } My_Komand;

#define   defaul_My_Komand2 {  \
        0,\
        0,\
        0,\
        0,\
        20,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0,\
        0\
}

        //   My_Komand My_Komand1=defaul_My_Komand2;
        My_Komand My_Komand1={  0,
                                0,
                                0,
                                0,
                                20,
                                0,
                                0,
                                0,
                                0,
                                0
        };

        // State Machine function prototypes
        //------------------------------------
        // Alpha states
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

        // ****************************************************************************
        // Variables for CPU control
        // ****************************************************************************

        // Used to indirectly access all EPWM modules
        volatile struct EPWM_REGS *ePWM[] = {
                                             &EPwm1Regs,         //intentional: (ePWM[0] not used)
                                             &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
                                             &EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs};

        int16   VTimer0[4];         // Virtual Timers slaved off CPU Timer 0 (A events)
        int16   VTimer1[4];         // Virtual Timers slaved off CPU Timer 1 (B events)
        int16   VTimer2[4];         // Virtual Timers slaved off CPU Timer 2 (C events)
        int16   SerialCommsTimer;


        int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};



        SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;


        // ****************************************************************************
        // Flag variables
        // ****************************************************************************
        volatile Uint16 EnableFlag = 0;

        Uint32 IsrTicker = 0;

        Uint16 BackTicker = 0,
                lsw = 0,
                TripFlagDMC = 0,             //PWM trip status
                clearTripFlagDMC = 0,
                RunMotor = 0;

        int    EnableResolverISR,  // not used
        Run_Delay,          // not used
        LedCnt1=0;



        // ****************************************************************************
        // Variables for Field Oriented Control
        // ****************************************************************************
        float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h
        _iq VdTesting = _IQ(0.0),           // Vd reference (pu)
                ///VqTesting = _IQ(0.1819),//_IQ(0.1819),           // Vq reference (pu)
                VqTesting = _IQ(0.28),//_IQ(0.1819),          // Vq reference (pu)
                IdRef     = _IQ(0.0),           // Id reference (pu)
                IqRef     = _IQ(0.08),          // Iq reference (pu)
                MyIQ= _IQ(0.0),
                SpeedRef  = _IQ(0);           // For Closed Loop tests
        //_iq cal_offset_A = _IQ15(0.4706);
        //_iq cal_offset_B = _IQ15(0.4656);
        _iq cal_offset_A = _IQ15(0.5076);
        _iq cal_offset_B = _IQ15(0.5055);
        //_iq cal_offset_A = _IQ15(0.5);
        //_iq cal_offset_B = _IQ15(0.5);
        _iq K1=_IQ(0.998);      //Offset filter coefficient K1: 0.05/(T+0.05);
        _iq K2=_IQ(0.001999);   //Offset filter coefficient K2: T/(T+0.05);
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


        // Instance a PWM driver instance
        PWMGEN pwm1 = PWMGEN_DEFAULTS;

        // Instance a Space Vector PWM modulator. This modulator generates a, b and c
        // phases based on the d and q stationery reference frame inputs
        //SVGEN svgen1 = SVGEN_DEFAULTS;
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
        // ****************************************************************************
        // Extern functions and variables referred from resolver.c
        // ****************************************************************************
        extern void baseParamsInit(void);
        extern void derivParamsCal(void);

        //extern RESOLVER_OUTPUT rslvrOut;

        // ****************************************************************************
        // Variables for Datalog module
        // ****************************************************************************
        float DBUFF_4CH1[200],
        DBUFF_4CH2[200],
        DBUFF_4CH3[200],
        DBUFF_4CH4[200],
        DlogCh1,
        DlogCh2,
        DlogCh3,
        DlogCh4;

        // Create an instance of DATALOG Module
        DLOG_4CH_F dlog_4ch1;
        int16 PwmDacCh1=0;
        int16 PwmDacCh2=0;
        int16 PwmDacCh3=0;
        PWMDAC pwmdac1 = PWMDAC_DEFAULTS;
        _iq posEncElecTheta[6],
        posEncMechTheta[6];
        void My_Com_Tx(unsigned char flagswitch,unsigned char *mass);
        void My_Com_Tx2(unsigned char flagswitch, unsigned char *mass);
        void My_Com_Tx32(unsigned char flagswitch,unsigned char lenght, unsigned char *mass);

        void SwitchKomand(unsigned char flagswitch,unsigned char *mass);

        void posEncoderSuite(void)
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


        void scib_echoback_init(void);
        unsigned char SectrMy=0;



        void A0(void)
        {
            if(CpuTimer0Regs.TCR.bit.TIF == 1)
            {
                CpuTimer0Regs.TCR.bit.TIF = 1;  // clear flag
                DataBiss();
                //My_Biss1.data=My_Position;
                My_Biss1.data=PositionOfReanishowInt32*1024;
                Otvet_Com32.Status_Zel_0=0x000000FF&My_Biss1.data;
                Otvet_Com32.Status_Zel_1=(0x0000FF00&My_Biss1.data)>>8;
                Otvet_Com32.Status_Zel_2=(0x00FF0000&My_Biss1.data)>>16;
                Otvet_Com32.Status_Zel_3=(0xFF000000&My_Biss1.data)>>24;


                //-----------------------------------------------------------
                (*A_Task_Ptr)();        // jump to an A Task (A1,A2,A3,...)
                //-----------------------------------------------------------

                VTimer0[0]++;           // virtual timer 0, instance 0 (spare)
                SerialCommsTimer++;

                ///My_Function_Position(My_Flags1.flag_switch_F_position);
            }
            Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
        }

        void B0(void)
        {
            // loop rate synchronizer for B-tasks
            if(CpuTimer1Regs.TCR.bit.TIF == 1)
            {
                CpuTimer1Regs.TCR.bit.TIF = 1;              // clear flag

                //-----------------------------------------------------------
                (*B_Task_Ptr)();        // jump to a B Task (B1,B2,B3,...)
                //-----------------------------------------------------------
                VTimer1[0]++;           // virtual timer 1, instance 0 (spare)
            }

            Alpha_State_Ptr = &C0;      // Allow C state tasks
        }

        void C0(void)
        {
            // loop rate synchronizer for C-tasks
            if(CpuTimer2Regs.TCR.bit.TIF == 1)
            {
                CpuTimer2Regs.TCR.bit.TIF = 1;              // clear flag

                //-----------------------------------------------------------
                (*C_Task_Ptr)();        // jump to a C Task (C1,C2,C3,...)
                //-----------------------------------------------------------
                VTimer2[0]++;           //virtual timer 2, instance 0 (spare)
            }

            Alpha_State_Ptr = &A0;  // Back to State A0
        }

        Uint32 ManagmentOffsetLocal = 0;

        void My_Function_Position(unsigned char flagSwitch)
        {
            //char Znak;

            PositionOfTargetDesignationInt32 = My_Parametr1.position;

            ///PositionOfTargetDesignationUint16 = My_Parametr1.position;
            ///PositionOfTargetDesignationInt32 = - (int32)((int32)PositionOfTargetDesignationUint16 - (int32)32768);

            ManagmentOffset = PositionOfTargetDesignationInt32 - PositionOfReanishowInt32;

            if (DefinitionOfOrientation)
            {
                if(flagSwitch==1)
                {
                    if (Komand_Com32.Komand_CU.Priznak_Arr==Razaar)
                    {
                        DRV_RESET=0;
                        if (ManagmentOffset<0) {Znak = - Sign; ManagmentOffsetLocal = - ManagmentOffset;}
                        else {Znak = Sign; ManagmentOffsetLocal = ManagmentOffset;}

                        if (ManagmentOffsetLocal<PositioningAccuracy1) SpeedRef=_IQ24(0);
                        else
                        {
                            if(ManagmentOffsetLocal<PositioningAccuracy2) SpeedRef=Znak*_IQ24(0.0001);
                            else
                            {
                                if(ManagmentOffsetLocal<PositioningAccuracy3) SpeedRef=Znak*_IQ24(0.0002);
                                else
                                {
                                    if(ManagmentOffsetLocal<PositioningAccuracy4) SpeedRef=Znak*_IQ24(0.003);
                                    else SpeedRef=Znak*_IQ24(0.02);
                                }
                            }
                        }
                    }
                    else {SpeedRef=0;DRV_RESET=1;}
                }
            }else
            {
                if (Komand_Com32.Komand_CU.Priznak_Arr==Razaar)
                {
                    DRV_RESET=0;

                    FalseDefinitionOfOrientationCounter++;

                    if (FalseDefinitionOfOrientationCounter < 100000) //10 сек
                    {
                        //if (PositionOfReanishowUint16>(PositionOfArretirUint16-32768)) Znak = Sign
                        if (PositionOfReanishowUint16>0) Znak = Sign
                                else Znak = -Sign;
                        SpeedRef = Znak*_IQ24(0.002);
                    }
                    else
                    {
                        if (FalseDefinitionOfOrientationCounter<300000) SpeedRef = ((-Znak)*_IQ24(0.02));
                        else SpeedRef = Znak*_IQ24(0.02);
                    }
                }
            }

        }

        //=================================================================================
        //  A - TASKS (executed in every 50 usec)
        //=================================================================================


        //--------------------------------------------------------
        void A1(void) // SPARE (not used)
        //--------------------------------------------------------
        {
            if (EnableFlag == FALSE)
            {
                // if any EPwm's OST is set, force OST on all three to DISABLE inverter
                EALLOW;
                EPwm1Regs.TZFRC.bit.OST = 1;
                EPwm2Regs.TZFRC.bit.OST = 1;
                EPwm3Regs.TZFRC.bit.OST = 1;
                EDIS;

                RunMotor = 0;
            }
            else if((EnableFlag == TRUE) && (RunMotor == FALSE))
                // If clear cmd received, reset PWM trip
            {

                // clear EPWM trip flags
                DELAY_US(1L);


                rg1.Freq=0;
                rg1.Out=0;
                rg1.Angle=0;
                rc1.TargetValue=0;
                rc1.SetpointValue=0;
                smo1.Theta=0;
                smo1.Ealpha=0;
                smo1.Ebeta=0;

                pid1_id.data.d1 = 0;
                pid1_id.data.d2 = 0;
                pid1_id.data.i1 = 0;
                pid1_id.data.ud = 0;
                pid1_id.data.ui = 0;
                pid1_id.data.up = 0;
                pid1_id.data.v1 = 0;
                pid1_id.data.w1 = 0;
                pid1_id.term.Out = 0;

                pid1_iq.data.d1 = 0;
                pid1_iq.data.d2 = 0;
                pid1_iq.data.i1 = 0;
                pid1_iq.data.ud = 0;
                pid1_iq.data.ui = 0;
                pid1_iq.data.up = 0;
                pid1_iq.data.v1 = 0;
                pid1_iq.data.w1 = 0;
                pid1_iq.term.Out = 0;

                pid1_spd.data.d1 = 0;
                pid1_spd.data.d2 = 0;
                pid1_spd.data.i1 = 0;
                pid1_spd.data.ud = 0;
                pid1_spd.data.ui = 0;
                pid1_spd.data.up = 0;
                pid1_spd.data.v1 = 0;
                pid1_spd.data.w1 = 0;
                pid1_spd.term.Out = 0;
                lsw=0;
                RunMotor = TRUE;
                EALLOW;

                EPwm1Regs.TZCLR.bit.OST = 1;
                EPwm2Regs.TZCLR.bit.OST = 1;
                EPwm3Regs.TZCLR.bit.OST = 1;
                EDIS;
            }

            //-------------------
            //the next time CpuTimer0 'counter' reaches Period value go to A2
            A_Task_Ptr = &A2;
            //-------------------
        }

        //-----------------------------------------------------------------
        void A2(void) // SPARE (not used)
        //-----------------------------------------------------------------
        {

            //-------------------
            //the next time CpuTimer0 'counter' reaches Period value go to A3

            //------**TMP100
            if (!(TimerTempCounter%6000))
            {
                I2caRegs.I2CCNT     = 1;    // read 2 byte temperature
                I2caRegs.I2CMDR.all = 0x6C20;
                while(I2caRegs.I2CSTR.bit.RRDY == 0);   // wait for 1st byte
                temperature = I2caRegs.I2CDRR.all << 8;         // read upper 8 Bit (integers)
                messageTx[3] = (temperature >> 8) & 0xFF;
                messageTx[4] = temperature & 0xFF;
                TimerTempCounter++;

            }
            //------**TMP100



            A_Task_Ptr = &A3;
            //-------------------
        }

        //-----------------------------------------
        void A3(void) // SPARE (not used)
        //-----------------------------------------
        {

            //-----------------
            //the next time CpuTimer0 'counter' reaches Period value go to A1
            A_Task_Ptr = &A1;
            //-----------------
        }



        //=================================================================================
        //  B - TASKS (executed in every 100 usec)
        //=================================================================================

        //----------------------------------- USER ----------------------------------------

        //----------------------------------------
        void B1(void) // Toggle GPIO-00
        //----------------------------------------
        {

            //-----------------
            //the next time CpuTimer1 'counter' reaches Period value go to B2
            B_Task_Ptr = &B2;
            //-----------------
        }

        //----------------------------------------
        void B2(void) //  SPARE
        //----------------------------------------
        {

            //-----------------
            //the next time CpuTimer1 'counter' reaches Period value go to B3
            B_Task_Ptr = &B3;
            //-----------------
        }

        //----------------------------------------
        void B3(void) //  SPARE
        //----------------------------------------
        {

            //-----------------
            //the next time CpuTimer1 'counter' reaches Period value go to B1
            B_Task_Ptr = &B1;
            //-----------------
        }


        //=================================================================================
        //  C - TASKS (executed in every 150 usec)
        //=================================================================================

        //--------------------------------- USER ------------------------------------------

        //----------------------------------------
        void C1(void)   // Toggle GPIO-34
        //----------------------------------------
        {
            if(F_COM==com32)
            {
                ///----
                VqTesting = _IQ(0.20);
                ///----
                SwitchKomand32_Rx(Flags.Switch_Otvet_COM32);
                SwitchKomand32_Tx(Flags.Switch_tx_COM32);
            }

            if((LedCnt1==My_Komand1.Freq)&&(F_COM==com16))
            {
                DataBiss();

                My_Com2.Sklad[1]=Targeting;
                My_Com2.Sklad[2]=(My_Position/1000)>>8;
                My_Com2.Sklad[3]=(My_Position*0.001);
                My_Com2.Sklad[4]=0;
                My_Com2.Sklad[5]=0;
                Flags.Switch_tx_COM=1;
                My_Com_Tx(Flags.Switch_tx_COM, My_Com2.Sklad);
                Flags.Switch_tx_COM=0;
                LedCnt1=0;
                EALLOW;
                GpioDataRegs.GPBTOGGLE.bit.GPIO43 = 1;
                GpioDataRegs.GPBTOGGLE.bit.GPIO42 = 1;
                EDIS;
            }
            else{ LedCnt1++;Flags.Switch_tx_COM=0;}

            //-----------------
            //the next time CpuTimer2 'counter' reaches Period value go to C2
            C_Task_Ptr = &C2;
            //-----------------

        }

        //----------------------------------------
        void C2(void) //  SPARE
        //----------------------------------------
        {

            //-----------------
            //the next time CpuTimer2 'counter' reaches Period value go to C3
            C_Task_Ptr = &C3;
            //-----------------
        }


        //-----------------------------------------
        void C3(void) //  SPARE
        //-----------------------------------------
        {

            //-----------------
            //the next time CpuTimer2 'counter' reaches Period value go to C1
            C_Task_Ptr = &C1;
            //-----------------
        }







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



        void ConfigureADC(void)
        {
            //Write ADC configurations and power up the ADC for both ADC A
            Uint16 i;

            EALLOW;

            //write configurations for ADC-A
            // External REFERENCE must be provided
            AdcaRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
            AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

            //Set pulse positions to late
            AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

            //write configurations for ADC-B
            // External REFERENCE must be provided
            AdcbRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
            AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

            //Set pulse positions to late
            AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

            //  //write configurations for ADC-C
            // External REFERENCE must be provided
            AdccRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdccRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
            AdccRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

            //Set pulse positions to late
            AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
            //
            //  //write configurations for ADC-D
            // External REFERENCE must be provided
            AdcdRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
            AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
            AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

            //Set pulse positions to late
            AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

            //power up the ADC
            AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

            //delay for > 1ms to allow ADC time to power up
            for(i = 0; i < 1000; i++){
                asm("   RPT#255 || NOP");
            }

            EDIS;
        }

        // ****************************************************************************
        // ****************************************************************************
        // POSITION LOOP UTILITY FUNCTIONS
        // ****************************************************************************
        // ****************************************************************************

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


        void TxSend()
        {
            int i = 0;

            for (i = 0; i < 31; i++)
                messageTx[31] = messageTx[31] ^ messageTx[i];
            for (i = 0; i < 32; i++)
            {
                while ( ScibRegs.SCICTL2.bit.TXEMPTY == 0); //wait for TX -empty
                ScibRegs.SCITXBUF.all = messageTx[i];
            }
            messageTx[31] = 0;
        }

        interrupt void SCIB_RX_isr(void)
        {
            //    Bukva=ScibRegs.SCIRXBUF.all;
            //    if(F_COM==com16)
            //    {
            //                while ( ScibRegs.SCICTL2.bit.TXEMPTY == 0); //wait for TX -empty
            //                ScibRegs.SCITXBUF.all=0xA3;
            //                unsigned char i=0;
            //            My_Com1.Bukva=Bukva;
            //
            //            if (My_Com1.Bukva==Komp)
            //            {
            //                if (My_Com1.u>0){}
            //                else{My_Com1.flagMess=1; My_Com1.u=0;}
            //            }
            //
            //            if (My_Com1.flagMess==1)
            //            {
            //                if (My_Com1.u==0)
            //                {
            //
            //                    My_Com1.Sklad[0]=Komp;
            //                    My_Com1.u+=1;
            //                }
            //            else
            //                {
            //                My_Com1.Sklad[ My_Com1.u]=My_Com1.Bukva;
            //
            //                if (My_Com1.Sklad[ My_Com1.u]==My_Com1.Start)
            //                {
            //                    if (My_Com1.Sklad[ My_Com1.u-1]!=1){}
            //                }
            //
            //                My_Com1.u+=1;
            //                }
            //            }
            //            if (My_Com1.u==8)
            //            {
            //                My_Com1.u=0;
            //                My_Com1.flagMess=0;
            //                My_Com1.Bukva=0;
            //
            //                My_Com1.flagViv=0;
            //                My_Com1.checksum=My_Com1.Sklad[0];
            //
            //            //контрольна€ сумма
            //            for (My_Com1.index=1;My_Com1.index<7;My_Com1.index++)
            //              {
            //                  My_Com1.checksum=My_Com1.checksum^My_Com1.Sklad[My_Com1.index];
            //              }
            //
            //                if((My_Com1.Sklad[7])==(My_Com1.checksum))
            //                {
            //                //ќтправл€ет массив message1
            //                  for (i=0;i<8;i++)
            //                      {
            //                        My_Com1.message1[i]=My_Com1.Sklad[i];
            //                      }
            //                  Flags.Switch_Otvet_COM=1;
            //                  SwitchKomand(Flags.Switch_Otvet_COM,My_Com1.message1);
            //                }
            //            }
            //    }
            //    if(F_COM==com32){SCIB_RX_32(Bukva, My_Com_32.Start1,  My_Com_32.Start2,  My_Com_32.finish);}
            //
            //
            PieCtrlRegs.PIEACK.all|=0x100;

        }

        void My_Com_Tx(unsigned char flagswitch,unsigned char *mass)
        {
            int i,j;

            if(flagswitch==1)
            {
                mass[0]=Driver;
                mass[6]=Finish;
                mass[7]=Driver;
                My_Com2.Sklad[7]=Driver;
                for (i=1;i<(lenght_Com-1);i++)
                {
                    My_Com2.Sklad[7]=My_Com2.Sklad[7]^My_Com2.Sklad[i];
                }
                mass[7]= My_Com2.Sklad[7];

                for (j=0;j<8;j++)
                {
                    while ( ScibRegs.SCICTL2.bit.TXEMPTY == 0); //wait for TX -empty
                    ScibRegs.SCITXBUF.all=mass[j];
                }
                flagswitch=0;
                Flags.Switch_tx_COM=0;
            }
        }
        void My_Com_Tx2(unsigned char flagswitch,unsigned char *mass)
        {
            int i,j;
            if(flagswitch==1)
            {
                mass[7]=mass[0];
                mass[6]=Finish;
                for (i=1;i<(lenght_Com-1);i++)
                {
                    mass[7]=mass[7]^mass[i];
                }
                My_Com2.message1[7]=mass[7];

                for (j=0;j<8;j++)
                {
                    My_Com2.message1[j]=mass[j];
                    while ( ScibRegs.SCICTL2.bit.TXEMPTY == 0); //wait for TX -empty
                    ScibRegs.SCITXBUF.all=mass[j];
                }
                flagswitch=0;
                Flags.Switch_tx_COM2=0;
            }
        }

        void SwitchKomand(unsigned char flagswitch,unsigned char *mass)
        {

            if(flagswitch==1)
            {
                switch(mass[1])
                {
                case StatusCOM:
                {
                    My_Komand1.Status=0x0A;
                    My_Com2.message1[0]=Driver;
                    My_Com2.message1[1]=StatusCOM;
                    My_Com2.message1[2]=My_Komand1.Status;
                    My_Com2.message1[3]=My_Komand1.Freq;
                    My_Com2.message1[4]=0;
                    My_Com2.message1[5]=0;
                    Flags.flagTx=1;
                    break;
                }
                case  Velocity:
                {
                    My_Com2.message1[0]=Driver;
                    My_Com2.message1[1]=Velocity;
                    My_Com2.message1[2]=My_Komand1.velocity1>>8;
                    My_Com2.message1[3]=0x00FF&My_Komand1.velocity1;
                    My_Com2.message1[4]=My_Komand1.Accuracity>>8;
                    My_Com2.message1[5]=0x00FF&My_Komand1.Accuracity;
                    My_Parametr1.accuracy=(mass[4]<<8)|mass[5];
                    Flags.flagTx=1;
                    break;
                }
                case Limit:
                {

                    My_Com2.message1[0]=Driver;
                    My_Com2.message1[1]=Limit;
                    My_Com2.message1[2]=My_Komand1.LowLimit>>8;
                    My_Com2.message1[3]=0x00FF&My_Komand1.LowLimit;
                    My_Com2.message1[4]=My_Komand1.HighLimit>>8;
                    My_Com2.message1[5]=0x00FF&My_Komand1.HighLimit;
                    Flags.flagTx=1;
                    break;
                }
                case Targeting:
                {
                    if (DefinitionOfOrientation) My_Parametr1.position=(mass[2]<<8) + mass[3];
                    break;
                }
                }
                if(Flags.flagTx==1)
                {
                    Flags.Switch_tx_COM2=1;
                    My_Com_Tx2(Flags.Switch_tx_COM2, My_Com2.message1);
                    Flags.Switch_tx_COM2=0;
                    Flags.flagTx=0;
                }

                flagswitch=0;
                Flags.Switch_Otvet_COM=0;
            }

        }


        void SCIB_RX_32(Uint16 bukva,unsigned char start1, unsigned char start2, unsigned char finish)
        {
            unsigned char bukva1;
            unsigned char i;
            My_Com_32.Bukva=bukva;
            if(bukva==start1){bukva1=start1;}
            if(bukva==start2)
            {
                if(bukva1==start1)
                {
                    if (My_Com_32.u>1)
                    {bukva1=0;}
                    else{My_Com_32.flagMess=1; My_Com_32.u=1;bukva1=0;}
                }

                else{bukva1=0;}
            }
            if(My_Com_32.flagMess==1)
            {
                if (My_Com_32.u==1)
                {
                    My_Com_32.Sklad[0]=start1;
                    My_Com_32.Sklad[1]=start2;
                    bukva1=0;
                    My_Com_32.u+=1;
                }
                else
                {
                    My_Com_32.Sklad[ My_Com_32.u]=My_Com_32.Bukva;

                    if (My_Com_32.Sklad[ My_Com_32.u]==start1)
                    {
                        if (My_Com_32.Sklad[ My_Com_32.u-1]!=1){bukva1=0;}
                    }
                    My_Com_32.u+=1;
                }
            }
            if(My_Com_32.u==32)
            {
                My_Com_32.u=0;
                My_Com_32.flagMess=0;
                My_Com_32.Bukva=0;
                bukva1=0;
                My_Com_32.flagViv=0;
                My_Com_32.checksum=My_Com_32.Sklad[0];

                //контрольна€ сумма
                for (My_Com_32.index=1;My_Com_32.index<(32-1);My_Com_32.index++)
                {
                    My_Com_32.checksum=My_Com_32.checksum^My_Com_32.Sklad[My_Com_32.index];
                }

                if((My_Com_32.Sklad[32-1])==(My_Com_32.checksum))
                {
                    //ќтправл€ет массив message1
                    for (i=0;i<32;i++){My_Com_32.message1[i]=My_Com_32.Sklad[i];}
                    Flags.Switch_Otvet_COM32=1;
                }
            }

            PieCtrlRegs.PIEACK.all|=0x100;

        }

        void My_Com_Tx32(unsigned char flagswitch,unsigned char lenght, unsigned char *mass)
        {
            int i;
            if(flagswitch==1)
            {
                for (i=0;i<lenght;i++)
                {
                    while ( ScibRegs.SCICTL2.bit.TXEMPTY == 0);
                    ScibRegs.SCITXBUF.all=mass[i];
                }
                flagswitch=0;
                Flags.Switch_tx_COM32=0;
            }
        }


        void SwitchKomand32_Rx(unsigned char flagswitch)
        {

            if(flagswitch==1)
            {

                Komand_Com32.Addres_Rx =My_Com_32.message1[2];
                switch(Komand_Com32.Addres_Rx)
                {
                case 0xA2:
                {
                    Komand_Com32.Komand_CU.Otk_Uderg= My_Com_32.message1[3]&0x01;
                    Komand_Com32.Komand_CU.Priznak_Arr= (My_Com_32.message1[3]>>1)&0x03;
                    Komand_Com32.Komand_CU.Stabil= (My_Com_32.message1[3]>>3)&0x01;
                    Komand_Com32.Komand_CU.Vkl_Most= (My_Com_32.message1[3]>>4)&0x03;
                    Komand_Com32.Komand_CU.Zadanie_CU= (My_Com_32.message1[3]>>6)&0x01;
                    Komand_Com32.Komand_Nagrev.Vkl_Regim_Nagreva=(My_Com_32.message1[4])&0x01;
                    Komand_Com32.Tek_Regim_Raboti=(My_Com_32.message1[5])&0x01;
                    Komand_Com32.Zel_0=My_Com_32.message1[6];
                    Komand_Com32.Zel_1=My_Com_32.message1[7];
                    Komand_Com32.Zel_2=My_Com_32.message1[8];
                    Komand_Com32.Zel_3=My_Com_32.message1[9];
                    Komand_Com32.Mems_V_x_Low=My_Com_32.message1[10];
                    Komand_Com32.Mems_V_x_High=My_Com_32.message1[11];
                    Komand_Com32.Mems_V_y_Low=My_Com_32.message1[12];
                    Komand_Com32.Mems_V_y_High=My_Com_32.message1[13];
                    Komand_Com32.Mems_V_z_Low=My_Com_32.message1[14];
                    Komand_Com32.Mems_V_z_High=My_Com_32.message1[15];
                    Komand_Com32.Mems_a_x_Low=My_Com_32.message1[16];
                    Komand_Com32.Mems_a_x_High=My_Com_32.message1[17];
                    Komand_Com32.Mems_a_y_Low=My_Com_32.message1[18];
                    Komand_Com32.Mems_a_y_High=My_Com_32.message1[19];
                    Komand_Com32.Mems_a_z_Low=My_Com_32.message1[20];
                    Komand_Com32.Mems_a_z_High=My_Com_32.message1[21];
                    /*
        if(My_Sector.flag_star_define_sector==1)
        {
         My_Parametr1.position32=((((Uint32)Komand_Com32.Zel_3<<8)|(Uint32)Komand_Com32.Zel_2)<<16)|(((Uint32)Komand_Com32.Zel_1<<8)|(Uint32)Komand_Com32.Zel_0);
         My_Parametr1.position=  My_Parametr1.position32/1024;
        }
                     */
                    if(DefinitionOfOrientation)
                    {
                        My_Parametr1.position32=((((Uint32)Komand_Com32.Zel_3<<8)|(Uint32)Komand_Com32.Zel_2)<<16)|(((Uint32)Komand_Com32.Zel_1<<8)|(Uint32)Komand_Com32.Zel_0);
                        My_Parametr1.position=  (int32)My_Parametr1.position32/1024;
                    }

                    Flags.Switch_Otvet_COM32=0;
                    flagswitch=0;
                    Flags.Switch_tx_COM32=1;
                    break;
                }
                case 0xA1:{break;}

                }


            }
        }
        void SwitchKomand32_Tx(unsigned char flagswitch)
        {
            unsigned char i;
            if(flagswitch==1)
            {
                My_Com_32_Tx[0]=My_Com_32.Start1;
                My_Com_32_Tx[1]=0xA2;
                My_Com_32_Tx[2]=My_Com_32.Start2;
                My_Com_32_Tx[3]=(Otvet_Com32.Status_CU.CU<<1)|(Otvet_Com32.Status_CU.Uderg<<3)|(Otvet_Com32.Status_CU.Stabil<<4)|(Otvet_Com32.Status_CU.Vkl_Most<<5);
                My_Com_32_Tx[4]=Otvet_Com32.Status_Nagrev.Vkl_Regim_Nagreva;
                My_Com_32_Tx[5]=Otvet_Com32.Status_Tek_Regim_Raboti;
                My_Com_32_Tx[6]=Otvet_Com32.Status_Zel_0;
                My_Com_32_Tx[7]=Otvet_Com32.Status_Zel_1;
                My_Com_32_Tx[8]=Otvet_Com32.Status_Zel_2;
                My_Com_32_Tx[9]=Otvet_Com32.Status_Zel_3;
                My_Com_32_Tx[10]=Otvet_Com32.Status_V_Low;
                My_Com_32_Tx[11]=Otvet_Com32.Status_V_High;
                My_Com_32_Tx[12]=Otvet_Com32.Status_a_Low;
                My_Com_32_Tx[13]=Otvet_Com32.Status_a_High;
                My_Com_32_Tx[14]=Otvet_Com32.Stopper1_0;
                My_Com_32_Tx[15]=Otvet_Com32.Stopper1_1;
                My_Com_32_Tx[16]=Otvet_Com32.Stopper1_2;
                My_Com_32_Tx[17]=Otvet_Com32.Stopper1_3;
                My_Com_32_Tx[18]=Otvet_Com32.Stopper2_0;
                My_Com_32_Tx[19]=Otvet_Com32.Stopper2_1;
                My_Com_32_Tx[20]=Otvet_Com32.Stopper2_2;
                My_Com_32_Tx[21]=Otvet_Com32.Stopper2_3;
                My_Com_32_Tx[22]=Otvet_Com32.Arr_0;
                My_Com_32_Tx[23]=Otvet_Com32.Arr_1;
                My_Com_32_Tx[24]=Otvet_Com32.Arr_2;
                My_Com_32_Tx[25]=Otvet_Com32.Arr_3;
                My_Com_32_Tx[26]=Otvet_Com32.Status_Accuracy;
                My_Com_32_Tx[27]=Otvet_Com32.Status_T_CU;
                My_Com_32_Tx[28]=Otvet_Com32.versiya_PO;
                My_Com_32_Tx[29]=0;
                My_Com_32_Tx[30]=My_Com_32.finish;


                My_Com_32_Tx[31]=My_Com_32_Tx[0];
                for (i=1;i<(32-1);i++) { My_Com_32_Tx[31]= My_Com_32_Tx[31]^My_Com_32_Tx[i];}
                My_Com_Tx32( 1, 32, My_Com_32_Tx);

            }


        }
        void scib_echoback_init(void)
        {
            //
            // Note: Clocks were turned on to the SCIA peripheral
            // in the InitSysCtrl() function
            //

            ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
            // No parity,8 char bits,
            // async mode, idle-line protocol
            ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
            // Disable RX ERR, SLEEP, TXWAKE
            // SciaRegs.SCICTL2.all = 0x0003;
            // SciaRegs.SCICTL2.bit.TXINTENA = 1;
            ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

            //
            /* // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
    ScibRegs.SCIHBAUD.all = 0x0002;
    ScibRegs.SCILBAUD.all = 0x008B;*/
            ScibRegs.SCIHBAUD.all = 0x0000;

            ScibRegs.SCILBAUD.all = 0x0036;

            ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
        }









        void main(void)
        {
            int i;
            messageTx[0] = 0xAA;
            messageTx[1] = 0xB1;
            messageTx[2] = 0xA1;

            messageTx[30] = 0xFF;




            InitSysCtrl();// Initialize System Control:  // PLL, WatchDog, enable Peripheral Clocks  // This example function is found in the F28M3Xx_SysCtrl.c file.

#ifdef _FLASH  // Only used if running from FLASH    // Note that the variable FLASH is defined by the compiler
            // Call Flash Initialization to setup flash waitstates   // This function must reside in RAM
            InitFlash();    // Call the flash wrapper init function
#endif //(FLASH)

            EnableFlag = 1;
            ///Komand_Com32.Komand_CU.Priznak_Arr=Zaar;//Razaar;
            Komand_Com32.Komand_CU.Priznak_Arr=Razaar;//Zaar;

            // Waiting for enable flag set
            while (EnableFlag == FALSE) BackTicker++;

            DINT;   // Clear all interrupts and initialize PIE vector table:   // Disable CPU interrupts

            InitPieCtrl(); // Initialize the PIE control registers to their default state.   // The default state is all PIE interrupts disabled and flags are cleared.

            // Disable CPU interrupts and clear all CPU interrupt flags:
            IER = 0x0000;
            IFR = 0x0000;
            // Initialize the PIE vector table with pointers to the shell Interrupt
            // Service Routines (ISR).
            // This will populate the entire table, even if the interrupt
            // is not used in this example.  This is useful for debug purposes.
            // The shell ISR routines are found in F28M3Xx_DefaultIsr.c.
            // This function is found in F28M3Xx_PieVect.c.
            InitPieVectTable();


            //------**TMP100
            //  Initialize I2C
            I2CA_Init();
            //------**TMP100



            // Timing sync for background loops
            // Timer period definitions found in device specific PeripheralHeaderIncludes.h
            CpuTimer0Regs.PRD.all =  60000;     // A tasks
            CpuTimer1Regs.PRD.all =  300000;        // B tasks
            CpuTimer2Regs.PRD.all =  3000000;       // C tasks

            // Tasks State-machine init
            Alpha_State_Ptr = &A0;
            A_Task_Ptr = &A1;
            B_Task_Ptr = &B1;
            C_Task_Ptr = &C1;

            // Initialize PWM module
            pwmdac1.PeriodMax = 834;   //  1000-> 50kHz, 500->100kHz, 834->60kHz
            pwmdac1.PwmDacInPointer0 = &PwmDacCh1;
            pwmdac1.PwmDacInPointer1 = &PwmDacCh2;
            pwmdac1.PwmDacInPointer2 = &PwmDacCh3;

            PWMDAC_INIT_MACRO(pwmdac1);

            EALLOW;
            CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

            // *****************************************
            // Inverter PWM configuration
            // ****************************************
            /* By default on soprano the PWM clock is divided by 2
             * ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1
             * Deadband needs to be 2.0us => 10ns*200=2us
             */
            PWM_1ch_UpDwnCnt_CNF(1,INV_PWM_TICKS,200);
            PWM_1ch_UpDwnCnt_CNF(2,INV_PWM_TICKS,200);
            PWM_1ch_UpDwnCnt_CNF(3,INV_PWM_TICKS,200);

            InitGpio();
            GPIO_SetupPinMux(51, GPIO_MUX_CPU1, 0);
            GPIO_SetupPinOptions(1, GPIO_OUTPUT, 0);
            GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 0);
            GPIO_SetupPinOptions(3, GPIO_OUTPUT, 0);
            GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
            GPIO_SetupPinOptions(5, GPIO_OUTPUT, 0);

            InitEPwm1Gpio();
            InitEPwm2Gpio();
            InitEPwm3Gpio();



            GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 3; // asynch input
            GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3; // asynch input
            GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3; // choose GPIO for mux option
            GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;

            EDIS;

            biss1.InitTheta=_IQ(0.5);

            //Configure the ADC and power it up
            ConfigureADC();

            EALLOW;
            AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 0;   // A0
            AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   //
            AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;   //

            AdcaRegs.ADCSOC1CTL.bit.CHSEL     = 1;    // A1
            AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 30;   //
            AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    //

            AdcaRegs.ADCSOC2CTL.bit.CHSEL     = 2;    // A2
            AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 30;
            AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;

            AdcaRegs.ADCSOC3CTL.bit.CHSEL     = 3;    // A3
            AdcaRegs.ADCSOC3CTL.bit.ACQPS     = 30;
            AdcaRegs.ADCSOC3CTL.bit.TRIGSEL   = 5;

            AdcaRegs.ADCSOC5CTL.bit.CHSEL     = 4;   // A4
            AdcaRegs.ADCSOC5CTL.bit.ACQPS     = 30;   //
            AdcaRegs.ADCSOC5CTL.bit.TRIGSEL   = 5;   //

            AdcaRegs.ADCSOC4CTL.bit.CHSEL     = 4;   // A5
            AdcaRegs.ADCSOC4CTL.bit.ACQPS     = 30;   //
            AdcaRegs.ADCSOC4CTL.bit.TRIGSEL   = 5;   //

            AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 0;   // B0
            AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;   //
            AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;   //

            AdcbRegs.ADCSOC2CTL.bit.CHSEL     = 1;    //B1
            AdcbRegs.ADCSOC2CTL.bit.ACQPS     = 30;   //
            AdcbRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    //

            AdcbRegs.ADCSOC3CTL.bit.CHSEL    = 2;     // B2
            AdcbRegs.ADCSOC3CTL.bit.ACQPS    = 30;    //
            AdcbRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;     //

            AdcbRegs.ADCSOC4CTL.bit.CHSEL    = 3;     // B3
            AdcbRegs.ADCSOC4CTL.bit.ACQPS    = 30;    //
            AdcbRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;     //

            AdcbRegs.ADCSOC6CTL.bit.CHSEL    = 4;     // B4
            AdcbRegs.ADCSOC6CTL.bit.ACQPS    = 30;    //
            AdcbRegs.ADCSOC6CTL.bit.TRIGSEL  = 5;     //

            AdcbRegs.ADCSOC5CTL.bit.CHSEL    = 4;     // B5
            AdcbRegs.ADCSOC5CTL.bit.ACQPS    = 30;    //
            AdcbRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;     //

            AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 0;    // D0
            AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 30;
            AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;

            EPwm1Regs.ETSEL.bit.SOCASEL = 1;
            EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
            EPwm1Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group
            EPwm1Regs.ETCLR.bit.SOCA = 1;       /* Clear SOCA flag */
            EDIS;

            speed1.K1 = _IQ21(1/(BASE_FREQ*T));
            speed1.K2 = _IQ(1/(1+T*2*PI*5));      // Low-pass cut-off frequency
            speed1.K3 = _IQ(1)-speed1.K2;
            speed1.BaseRpm = 120*(BASE_FREQ/POLES);
            biss1.PolePairs    = POLES/2;

            rg1.StepAngleMax = _IQ(BASE_FREQ*T);

            //  pid1_id.param.Kp = _IQ(BASE_CURRENT/BASE_VOLTAGE);
            pid1_id.param.Kp = _IQ(0.015);
            pid1_id.param.Kr = _IQ(1.0);
            pid1_id.param.Ki = _IQ(0.00019);
            pid1_id.param.Kd = _IQ(0/T);
            pid1_id.param.Km = _IQ(1.0);
            pid1_id.param.Umax = _IQ(0.40);
            pid1_id.param.Umin = _IQ(-0.40);

            // Initialize the PID_GRANDO_CONTROLLER module for Iq
            // pid1_iq.param.Kp = _IQ(BASE_CURRENT/BASE_VOLTAGE);
            pid1_iq.param.Kp = _IQ(0.015);
            pid1_iq.param.Kr = _IQ(1.0);
            pid1_iq.param.Ki = _IQ(0.00019);
            pid1_iq.param.Kd = _IQ(0/T);
            pid1_iq.param.Km = _IQ(1.0);
            pid1_iq.param.Umax = _IQ(0.4);
            pid1_iq.param.Umin = _IQ(-0.4);

            // Initialize the PID_GRANDO_CONTROLLER module for Speed
            /* pid1_spd.param.Kp = _IQ(0.2);
        pid1_spd.param.Kr = _IQ(1.0);
        pid1_spd.param.Ki = _IQ(0.0002);
        pid1_spd.param.Kd = _IQ(0/(T*SpeedLoopPrescaler));
        pid1_spd.param.Km = _IQ(1.0);
        pid1_spd.param.Umax = _IQ(0.95);
        pid1_spd.param.Umin = _IQ(-0.95);*/
            pid1_spd.param.Kp=_IQ(0.05);
            pid1_spd.param.Km=_IQ(1.0);
            pid1_spd.param.Ki=_IQ(0.0001);
            pid1_spd.param.Kd=_IQ(0.0);
            pid1_spd.param.Kr=_IQ(1.0);
            pid1_spd.param.Umax=_IQ(0.9);
            pid1_spd.param.Umin=_IQ(-0.9);

            pid1_spd.term.c1=1;
            pid1_spd.term.c2=1;
            // Initialize the PID module for speed
            EALLOW;
            GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO28
            GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3; // Asynch input
            GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;  // GPIO28 = SCIRXDA
            GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO29
            GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;  // GPIO29 = SCITXDA
            EDIS;

            //------**TMP100
            EALLOW;

            GpioCtrlRegs.GPBMUX1.all = 0;           // GPIO47 ... GPIO32 = General Purpose I/O
            GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;    // GPIO32 = I2C - SDA
            GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;    // GPIO33 = I2C - SCL

            GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
            GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // Enable pull-up for GPIO33 (SCLA)

            GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
            GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)

            EDIS;
            //------**TMP100


            bissc_init();
            DELAY_US(800L);     //Delay 800us
            bissc_data_struct.dataReady = 1;

            /*
#if BUILDLEVEL == LEVEL3
    IqRef    = 0.0;
#else
    IqRef = _IQ(0.2);
#endif
             */

            RunMotor = 0;
            LedCnt1  = 0;
            EnableResolverISR = 1;

#if (BUILDLEVEL==LEVEL2)
            Run_Delay = 10;
#else
            Run_Delay = 100;
#endif
            My_Flags1.flag_switch_F_position=1;







            EALLOW;
            CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
            EDIS;
            EALLOW;
            PieVectTable.EPWM1_INT = &MotorControlISR;
            EPwm1Regs.ETSEL.bit.INTSEL = 1;
            EPwm1Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
            EPwm1Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event
            EPwm1Regs.ETCLR.bit.INT=1;

            PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // Enable PWM11INT in PIE group 3
            PieVectTable.SCIB_RX_INT=SCIB_RX_isr;
            PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
            //PWM11 INT is used to trigger Motor Control ISR

            for (i=0;i<10;i++)
            {
                posEncoderSuite();      //принимаем данные с Reanishow
                DataBiss();             //в My_Position записываем значение датчиков Reanishow
                DELAY_US(1000L);
            }


            IER |= M_INT3; // Enable group 3 interrupts
            IER |= M_INT1; // Enable group 1 interrupts
            IER |= M_INT9;


            EINT;          // Enable Global interrupt INTM
            ERTM;          // Enable Global realtime interrupt DBGM
            EDIS;
            scib_echoback_init();

            //// //------**TMP100
            //     I2caRegs.I2CCNT     = 1;    // read 2 byte temperature
            //     I2caRegs.I2CMDR.all = 0x6C20;
            //         while(I2caRegs.I2CSTR.bit.RRDY == 0);   // wait for 1st byte
            //         temperature = I2caRegs.I2CDRR.all << 8;         // read upper 8 Bit (integers)
            //         temperature = temperature/256;
            //// //------**TMP100

            // //------**TMP100
            I2caRegs.I2CCNT     = 1;    // read 2 byte temperature
            I2caRegs.I2CMDR.all = 0x6C20;
            while(I2caRegs.I2CSTR.bit.RRDY == 0);   // wait for 1st byte
            temperature = I2caRegs.I2CDRR.all;         // read upper 8 Bit (integers)
            messageTx[3] = (temperature >> 8) & 0xFF;
            messageTx[4] = temperature & 0xFF;
            // //------**TMP100


            //------**TMP100
            /*if ((CpuTimer0.InterruptCount - TimeForTMP100)>6000)
         {
             TimeForTMP100 = CpuTimer0.InterruptCount;
             I2caRegs.I2CCNT     = 1;    // read 2 byte temperature
             I2caRegs.I2CMDR.all = 0x6C20;
                 while(I2caRegs.I2CSTR.bit.RRDY == 0);   // wait for 1st byte
                 temperature = I2caRegs.I2CDRR.all << 8;         // read upper 8 Bit (integers)
                 temperature = temperature/256;
         }*/
            //------**TMP100

//DRV_OFF = 1;
            for(;;)  //infinite loop
            {
                // State machine entry & exit point
                //===========================================================
                (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
                //===========================================================



                if(DRV_RESET)
                {
                    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
                    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
                    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

                }
                else
                {
                    GpioDataRegs.GPASET.bit.GPIO1 = 1;
                    GpioDataRegs.GPASET.bit.GPIO3 = 1;
                    GpioDataRegs.GPASET.bit.GPIO5 = 1;
                }

                if (DRV_OFF)
                {
                    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
                    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
                    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
                }

                txDelayer++;
                if(txDelayer == 10000000)
                {
                    TxSend();
                    txDelayer = 0;
                }
            }
        } //END MAIN CODE


        void DataBiss(void)
        {
            My_Position= bissc_data_struct.position;
        }

        //целеуказание выбираем от знака определ€емого ранее, т.е. приводим к 0 и вычисл€ем исход€ из знака!!!! и вычисл€ем разницу с учетом наиболее близкого подхода, но не выход€ за границы
        //необходимо определить допустимый заход за пределы круга при целеукаазании (ограничение) с учетом эволюций!!!


        interrupt void MotorControlISR(void)
        {
            EINT;

            // Verifying the ISR
            IsrTicker++;

            posEncoderSuite();      //принимаем данные с Reanishow
            DataBiss();             //в My_Position записываем значение датчиков Reanishow

            PositionOfReanishowUint16 = (Uint16)(My_Position>>10);

            if ((PositionOfReanishowUint16<(43680-30))&&(PositionOfReanishowUint16>PositionOfArretirUint16))
            {
                DefinitionOfOrientation = TRUE;
                PositiveDirection = TRUE;
            }

            if ((PositionOfReanishowUint16<PositionOfArretirUint16)&&(PositionOfReanishowUint16>(10952+30)))
            {
                DefinitionOfOrientation = TRUE;
                PositiveDirection = FALSE;
            }

            if (DefinitionOfOrientation)
            {
                if (PositiveDirection)
                {
                    if (PositionOfReanishowUint16>=PositionOfArretirUint16) PositionOfReanishowInt32 = PositionOfReanishowUint16 - PositionOfArretirUint16;
                    else PositionOfReanishowInt32 = PositionOfReanishowUint16 + 65536 - PositionOfArretirUint16;
                }
                else
                {
                    if (PositionOfReanishowUint16<PositionOfArretirUint16)  PositionOfReanishowInt32 = (int32)((int32)PositionOfReanishowUint16 - (int32)PositionOfArretirUint16);
                    else PositionOfReanishowInt32 = - (int32)PositionOfArretirUint16 - (65536 - (int32)PositionOfReanishowUint16);
                }
            }


            /*
    if ((PositionOfReanishowUint16<(10877-30))||(PositionOfReanishowUint16>PositionOfArretirUint16))
    {
        DefinitionOfOrientation = TRUE;
        PositiveDirection = TRUE;
    }

    if ((PositionOfReanishowUint16<PositionOfArretirUint16)&&(PositionOfReanishowUint16>(43801+30)))
    {
        DefinitionOfOrientation = TRUE;
        PositiveDirection = FALSE;
    }

    if (DefinitionOfOrientation)
    {
        if (PositiveDirection)
        {
            if (PositionOfReanishowUint16>=PositionOfArretirUint16) PositionOfReanishowInt32 = PositionOfReanishowUint16 - PositionOfArretirUint16;
            else PositionOfReanishowInt32 = PositionOfReanishowUint16 + 65536 - PositionOfArretirUint16;

        }
        else PositionOfReanishowInt32 = (int32)((int32)PositionOfReanishowUint16 - (int32)PositionOfArretirUint16);
    }
             */
            My_Function_Position(My_Flags1.flag_switch_F_position);


            // ------------------------------------------------------------------------------
            //  Connect inputs of the RMP module and call the ramp control macro
            // ------------------------------------------------------------------------------

            rc1.TargetValue = SpeedRef;
            RC_MACRO(rc1)
            // ------------------------------------------------------------------------------
            //  Connect inputs of the RAMP GEN module and call the ramp generator macro
            // ------------------------------------------------------------------------------
            rg1.Freq = rc1.SetpointValue;
            RG_MACRO(rg1)
            // ------------------------------------------------------------------------------
            //  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
            //  Connect inputs of the CLARKE module and call the clarke transformation macro
            // ------------------------------------------------------------------------------
            // ((ADCmeas(q12)/2^12)-0.5)*2
            clarke1.As=-(_IQ15toIQ((AdcaResultRegs.ADCRESULT1<<3)-cal_offset_A)<<1);
            clarke1.Bs=-(_IQ15toIQ((AdcbResultRegs.ADCRESULT2<<3)-cal_offset_B)<<1);
            CLARKE_MACRO(clarke1)
            // ------------------------------------------------------------------------------
            //  Connect inputs of the PARK module and call the park trans. macro
            // ------------------------------------------------------------------------------
            park1.Alpha = clarke1.Alpha;
            park1.Beta = clarke1.Beta;
            park1.Angle = rg1.Out;
            park1.Sine = _IQsinPU(park1.Angle);
            park1.Cosine = _IQcosPU(park1.Angle);
            PARK_MACRO(park1)
            // ------------------------------------------------------------------------------
            //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
            // ------------------------------------------------------------------------------
            ipark1.Ds = VdTesting;
            ipark1.Qs = VqTesting;
            ipark1.Sine=park1.Sine;
            ipark1.Cosine=park1.Cosine;
            IPARK_MACRO(ipark1)
            // ------------------------------------------------------------------------------
            //  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
            // ------------------------------------------------------------------------------
            volt1.DcBusVolt = _IQ15toIQ((AdcbResultRegs.ADCRESULT3<<3));  // DC Bus voltage meas.
            volt1.MfuncV1 = svgen_dq1.Ta;
            volt1.MfuncV2 = svgen_dq1.Tb;
            volt1.MfuncV3 = svgen_dq1.Tc;
            VOLT_MACRO(volt1)
            // ------------------------------------------------------------------------------
            //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
            // ------------------------------------------------------------------------------
            svgen_dq1.Ualpha = ipark1.Alpha;
            svgen_dq1.Ubeta = ipark1.Beta;
            SVGEN_MACRO(svgen_dq1)
            // ------------------------------------------------------------------------------
            //  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
            // ------------------------------------------------------------------------------
            pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta);
            pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb);
            pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc);
            PWM_MACRO(pwm1)                                // Calculate the new PWM compare values

            EPwm1Regs.CMPA.bit.CMPA=pwm1.PWM1out;  // PWM 1A - PhaseA
            EPwm2Regs.CMPA.bit.CMPA=pwm1.PWM2out;  // PWM 2A - PhaseB
            EPwm3Regs.CMPA.bit.CMPA=pwm1.PWM3out;  // PWM 3A - PhaseC  // Calculate the new PWM compare values
            // ------------------------------------------------------------------------------
            //  Connect inputs of the PWCCMDAC module
            // ------------------------------------------------------------------------------
            PwmDacCh1 = _IQtoQ15(clarke1.As);
            PwmDacCh2 = _IQtoQ15(clarke1.Bs);
            PwmDacCh3 = _IQtoQ15(volt1.Valpha);
            // ------------------------------------------------------------------------------
            //  Connect inputs of the DATALOG module
            // ------------------------------------------------------------------------------
            DlogCh1 = _IQtoQ15(volt1.Valpha);
            DlogCh2 = _IQtoQ15(clarke1.Alpha);
            DlogCh3 = _IQtoQ15(volt1.Vbeta );
            DlogCh4 = _IQtoQ15(clarke1.Beta);

            // =============================== LEVEL 4 ======================================
            //  Level 4 verifies the dq-axis current regulation performed by PID and speed
            //  measurement modules
            // ==============================================================================
            //  lsw=0: lock the rotor of the motor
            //  lsw=1: close the current loop

            PWMDAC_MACRO(pwmdac1)
            // ------------------------------------------------------------------------------
            //    Call the DATALOG update function.
            // ------------------------------------------------------------------------------
            //DLOG_4CH_F_FUNC(&dlog_4ch1);
            EALLOW;
            EPwm1Regs.ETCLR.bit.INT = 1;
            PieCtrlRegs.PIEACK.all   = PIEACK_GROUP3;
            EDIS;
        }// MainISR Ends Here

        void I2CA_Init(void)
        {

            I2caRegs.I2CMDR.bit.IRS = 0;    // Reset the I2C module
            // I2C slave address register
            I2caRegs.I2CSAR.all = TMP100_SLAVE;
            // I2C Prescale Register
            I2caRegs.I2CPSC.all = 14;       // Internal I2C module clock = SYSCLK/(PSC +1)
            // = 10 MHz

            I2caRegs.I2CCLKL = 95;          // Tmaster = (PSC +1)[ICCL + 5 + ICCH + 5] / 150MHz
            I2caRegs.I2CCLKH = 95;          // Tmaster =  15 [ICCL + ICCH + 10] / 150 MHz
            // d = 5  for IPSC >1

            // for I2C 50 kHz:
            // Tmaster = 20 µs * 150 MHz / 15 = 200 = (ICCL + ICCH +10)
            // ICCL + ICCH = 190
            // ICCL = ICH = 190/2 = 95

            //  I2caRegs.I2CCLKL = 45;
            //  I2caRegs.I2CCLKH = 45;          // for I2C 100 kHz:
            // Tmaster = 10 µs *150 MHz / 15 = 100 = (ICCL + ICCH + 10)
            // ICCL + ICCH = 90
            // ICCL = ICH = 90/2 = 45

            I2caRegs.I2CMDR.bit.IRS = 1;    // Take I2C out of reset
        }
