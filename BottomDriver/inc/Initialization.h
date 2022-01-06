#include <FirmwareParameters.h>


void InitSci(void)
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
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    /* // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
    ScibRegs.SCIHBAUD.all = 0x0002;
    ScibRegs.SCILBAUD.all = 0x008B;*/

    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x000B;

    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

void InitI2c(void)
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

void InitSystem()
{
    InitSysCtrl();// Initialize System Control:  // PLL, WatchDog, enable Peripheral Clocks  // This example function is found in the F28M3Xx_SysCtrl.c file.

#ifdef _FLASH  // Only used if running from FLASH    // Note that the variable FLASH is defined by the compiler
    // Call Flash Initialization to setup flash waitstates   // This function must reside in RAM
    InitFlash();    // Call the flash wrapper init function
#endif //(FLASH)

    InitPieCtrl(); // Initialize the PIE control registers to their default state.   // The default state is all PIE interrupts disabled and flags are cleared.

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();


    //------**TMP100
    //  Initialize I2C
    //I2CA_Init();
    //------**TMP100
    InitCpuTimers();

    // Timing sync for background loops
    // Timer period definitions found in device specific PeripheralHeaderIncludes.h
    CpuTimer0Regs.PRD.all =  60000;     // A tasks
    CpuTimer1Regs.PRD.all =  300000;        // B tasks
    CpuTimer2Regs.PRD.all =  3000000;       // C tasks

    ConfigCpuTimer(&CpuTimer0, 200, 60000);
}

void InitPwm()
{
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
    EDIS;
}

void InitEveryGpioWeNeed()
{

    EALLOW;
    InitGpio();
    GPIO_SetupPinMux(51, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, 0);
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(3, GPIO_OUTPUT, 0);
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, 0);
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(33, GPIO_MUX_CPU1, 1);

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    EDIS;

    EALLOW;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 3; // asynch input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3; // asynch input
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3; // choose GPIO for mux option
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO28
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO29

    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;    // GPIO32 = I2C - SDA
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;    // GPIO33 = I2C - SCL
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // Enable pull-up for GPIO33 (SCLA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)

    EDIS;
}

void InitMotor()
{


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


    pid1_iq.param.Kp = _IQ(0.015);
    pid1_iq.param.Kr = _IQ(1.0);
    pid1_iq.param.Ki = _IQ(0.00019);
    pid1_iq.param.Kd = _IQ(0/T);
    pid1_iq.param.Km = _IQ(1.0);
    pid1_iq.param.Umax = _IQ(0.4);
    pid1_iq.param.Umin = _IQ(-0.4);

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
}

void InitBissc()
{
    int i = 0;
    biss1.InitTheta=_IQ(0.5);

    bissc_init();
    DELAY_US(800L);     //Delay 800us
    bissc_data_struct.dataReady = 1;

    for (i=0;i<10;i++)
    {
        PosEncoderSuite();      //принимаем данные с Renishaw
        DataBiss();             //в myPosition записываем значение датчиков Renishaw
        DELAY_US(1000L);
    }
}


void InitInterrupts()
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
    EALLOW;
    PieVectTable.EPWM1_INT = &MotorControlIsr;
    PieVectTable.TIMER0_INT = &CpuTimerIsr;
    //    PieVectTable.I2CA_INT = &I2cEepromIsr;
    EPwm1Regs.ETSEL.bit.INTSEL = 1;
    EPwm1Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event
    EPwm1Regs.ETCLR.bit.INT=1;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // Enable PWM11INT in PIE group 3
    PieVectTable.SCIB_RX_INT=SciRxIsr;
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    //    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
    //PWM11 INT is used to trigger Motor Control ISR

    IER |= M_INT3;
    IER |= M_INT1;
    IER |= M_INT9;
    //    IER |= M_INT8;

    EINT;          // Enable Global interrupt INTM
    ERTM;          // Enable Global realtime interrupt DBGM
    EDIS;
}












