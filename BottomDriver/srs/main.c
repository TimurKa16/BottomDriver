// Разбор в формулах матрицы
// Переделал на другую, с поворотом
//
// Исправление ошибки в функции Stabilization()


#include "Initialization.h"
#include "Stabilization.h"





// TODO
// main
void main(void)
{
    InitSystem();
    InitFirmwareParameters();
    InitVariables();
    InitPwm();
    InitEveryGpioWeNeed();
    InitBissc();
    InitMotor();
    InitInterrupts();
    InitSci();

    StartCpuTimer();
    ReadTmp100();

    position.topStopper = 50365 * 4;
    position.bottomStopper = 54370 * 4;
    position.zeroPosition = 61991 * 4;

    for(;;)
    {
        (*Alpha_State_Ptr)();

        CheckPwm();
        CheckSci();
    }
}

////////////////////    Прерывания  ////////////////////
// TODO

// Функция обработки прерывания ШИМа (частота - 10 кГц)
interrupt void MotorControlIsr(void)
{
    EINT;

    motorIsrTicker++;

    CheckTechnologicalMode();

    if (sciRxCommand.operatingMode == SciOperatingMode.Normal)
    {
        ReadRenishaw();
        SetPositionOfRenishaw();
        SetMotorPosition();

        if (motorIsrTicker % 10 == 0)
            CalculateAverageVelocity();

        if (motorIsrTicker % 100 == 0)
        {
            CheckPositionForEeprom();
        }
        if (stabilizationFlag)
        position.positionOfTargetDesignation = - Stabilization(sciRxCommand.target, sciRxCommand.otherTarget) + sciRxCommand.target;

        SetMotorPwm();
    }
    else
        resetDriver = 1;

    ClearMotorInterrupt();
}

// Функция обработки прерывания по COM-порту
interrupt void SciRxIsr(void)
{

    tmpCounter[0]++;
    frequencyTimer++;
    sciRxData = ScibRegs.SCIRXBUF.all;
    SciRx(sciRxData);

    PieCtrlRegs.PIEACK.all|=0x100;
}

// Функция обработки прерывания таймера
interrupt void CpuTimerIsr(void)
{
    //CpuTimer0.InterruptCount++;
    timer.cpuTimer++; // 16.6667 тиков - 1 секунда
    timer.i2cTimer++;
    timer.globalTimer++;

    unsigned long i = 0;
    unsigned long tmpLongVar = 0;
    for (i = 0; i < numberOfValues; i++)
        tmpLongVar += sumValue;
    longVar = tmpLongVar;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



void ClearMotorInterrupt()
{
    EALLOW;
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all   = PIEACK_GROUP3;
    EDIS;
}

//@@@
int motorFrequency = 10000;
int sciFrequency = 100;
// Как часто я буду брать новые поделенные ЦУ
void GetSplittedTarget()
{
    if (motorIsrTicker % ((motorFrequency / sciFrequency) / TARGET_DIVIDER) == 0)
    {
        if (targetDivider.positionOfTargetDesignationCounter < TARGET_DIVIDER)
        {
            position.positionOfTargetDesignation = targetDivider.newPositionOfTargetDesignation[targetDivider.positionOfTargetDesignationCounter];
            targetDivider.positionOfTargetDesignationCounter++;
        }
    }
}

void SetMotorAmplitude()
{
    if (sciRxCommand.arretirStatus == ArretirStatus.Opened)
    {
        if (wasArretired == 1)
        {
            if (arretirCounter > 10000)
            {
                wasArretired = 0;
                arretirCounter = 0;
            }
            else
            {
                amplitude = Amplitude.Max / 2;
                arretirCounter++;
                pid.iPart = 0;
            }
        }
        else
        {
            if (sciRxCommand.holdPosition == 0)
                amplitude = Amplitude.Max;
            else
                amplitude = Amplitude.HoldPosition;
        }
    }
    else if (sciRxCommand.arretirStatus == ArretirStatus.Closing)
        amplitude = Amplitude.Max / 2;

    resetDriver = 0;
}

int deltaAngle;
void SetPidParts()
{
    position.deltaAngleForStabilization = position.positionOfTargetDesignation - position.positionOfRenishaw;
    position.deltaAngleForStabilization = - position.deltaAngleForStabilization;
    deltaAngle = (int)position.deltaAngleForStabilization;

    position.velocity = (int)(position.absolutePositionOfRenishaw - position.absolutePositionOfRenishawStepBack);
    position.absolutePositionOfRenishawStepBack = position.absolutePositionOfRenishaw;

    if (abs(position.deltaAngleForStabilization) > 4000)
    {
        pid.pPart = pid.pCoefLo * (position.deltaAngleForStabilization) / 100;

        pid.iPartLong = pid.iPartLong + pid.iCoefLo * (position.deltaAngleForStabilization);

        if (pid.iPartLong > pid.iPartLimit * 1000)
            pid.iPartLong = pid.iPartLimit * 1000;
        else if (pid.iPartLong < -pid.iPartLimit * 1000)
            pid.iPartLong = -pid.iPartLimit * 1000;

        pid.iPart = pid.iPartLong / 1000;

        pid.dPart = pid.dCoefLo * position.velocity;
    }
    else
    {
        pid.pPart = pid.pCoefHi * (position.deltaAngleForStabilization) / 100;

        pid.iPartLong = pid.iPartLong + pid.iCoefHi * (position.deltaAngleForStabilization);

        if (pid.iPartLong > pid.iPartLimit * 1000)
            pid.iPartLong = pid.iPartLimit * 1000;
        else if (pid.iPartLong < -pid.iPartLimit * 1000)
            pid.iPartLong = -pid.iPartLimit * 1000;

        pid.iPart = pid.iPartLong / 1000;

        pid.dPart = pid.dCoefHi * position.velocity;
    }

    if (technologicalMode.resetPidParts == FALSE)
        pid.wholePart = pid.pPart + pid.dPart + pid.iPart;
    else
        pid.wholePart = 0;

    if (pid.wholePart > phase.maxPhase)
        pid.wholePart = phase.maxPhase;
    else if (pid.wholePart < -phase.maxPhase)
        pid.wholePart = -phase.maxPhase;
}

// Технологическое перекручивание
void CheckTechnologicalRotation()
{
    if (technologicalRotation.rotationFlag == 2)
        pid.wholePart = 4000;
    else if (technologicalRotation.rotationFlag == 1)
        pid.wholePart = -4000;

    if (position.absolutePositionOfRenishaw > ((int32)100000) && position.absolutePositionOfRenishaw < ((int32)128000))
        technologicalRotation.firstRotationPoint = 1;

    if (technologicalRotation.firstRotationPoint == 1)
    {
        if (position.absolutePositionOfRenishaw > (232000))
            technologicalRotation.secondRotationPoint = 1;

        if (technologicalRotation.secondRotationPoint == 1)
        {
            technologicalRotation.firstRotationPoint = 0;
            technologicalRotation.secondRotationPoint = 0;
            technologicalRotation.rotationFlag = 0;

            eeprom.position = 1;
            eeprom.keepPositionFlag1 = 0;
            eeprom.keepPositionFlag2 = 0;
        }
    }
}


void CheckTechnologicalMode()
{
    if (technologicalMode.turnOn == ON)
    {
        technologicalMode.turnOn = FALSE;
        technologicalMode.mode = TRUE;
        eeprom.position = 1;
        eeprom.positionIsRecieved = 1;
        technologicalMode.resetArretirStatus = 1;
        sciRxCommand.arretirStatus = ArretirStatus.Opened;
    }
    if (technologicalMode.turnOff == ON)
    {
        technologicalMode.mode = FALSE;
        technologicalMode.turnOff = FALSE;
    }
}



void SetMotorPhase()
{
    phase.pidPhase = pid.wholePart;
    phase.globalPhase = phase.electricalMotorNilPosition + phase.mechToElectrCoef * position.positionOfRenishaw;
    phase.globalPhaseToGo = phase.globalPhase + phase.pidPhase;

    if (sciRxCommand.holdPosition == 0)
        phase.phase = _IQ((phase.globalPhaseToGo % phase.phaseDivider) / (float)phase.phaseDivider);
}

void CalculateAverageVelocity()
{
    if (velocityCounter < 10)
    {
        summaryVelocity += abs(position.velocity);
        velocityCounter++;
    }
    else
    {
        position.averageVelocity = summaryVelocity;
        summaryVelocity = 0;
        velocityCounter = 0;
    }
}


// На сколько я буду дробить ЦУ
void SplitTarget()
{
    targetDivider.deltaPositionOfTargetDesignation = (float)(sciRxCommand.target - targetDivider.oldPositionOfTargetDesignation) /
            (float)TARGET_DIVIDER;

    int i;
    for (i = 0; i < (TARGET_DIVIDER - 1); i++)
        targetDivider.newPositionOfTargetDesignation[i] = targetDivider.oldPositionOfTargetDesignation +
        targetDivider.deltaPositionOfTargetDesignation * (i + 1);

    targetDivider.newPositionOfTargetDesignation[TARGET_DIVIDER - 1] = sciRxCommand.target;
    targetDivider.oldPositionOfTargetDesignation = sciRxCommand.target;
    targetDivider.positionOfTargetDesignationCounter = 0;
}

// Выполнение ЦУ
void SetMotorPosition()
{
    if (stabilizationFlag == 0)
        if (technologicalMode.resetPositionOfTargetDesignation == FALSE)
            GetSplittedTarget();

    if (heatStatus == 2)
    {
        if (definitionOfOrientation)
        {
            if (sciRxCommand.arretirStatus != ArretirStatus.Closed)
            {
                SetMotorAmplitude();
                SetPidParts();
                CheckTechnologicalRotation();

                if (findPosition.mode == 1)
                    FindPosition();

                SetMotorPhase();
            }
            else if (sciRxCommand.arretirStatus == ArretirStatus.Closed)
            {
                resetDriver = 1;
                wasArretired = 1;
            }
        }
    }
    else if (heatStatus == 1)
        amplitude = Amplitude.Heating;
}

// Определение позиции для EEPROM-ки
// 1 - область арретира
// 3 - слева от арретира
// 2 - справа от арретира
void CheckPositionForEeprom(void)
{
    if(rotationMode == RotationMode.Normal)
    {
        if((position.absolutePositionOfRenishaw > position.stopper2) && (position.absolutePositionOfRenishaw < position.stopper1))  //3
        {
            if(eeprom.keepPositionFlag2 == 0 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag2 = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if((eeprom.keepPositionFlag1 == 0 || eeprom.keepPositionFlag1 == 1 ) && eeprom.position == 1)
            {
                eeprom.position = 3;
                eeprom.keepPositionFlag1 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 1 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag1 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 3)
            {
                eeprom.keepPositionFlag2 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
        }
        else if(position.absolutePositionOfRenishaw > position.stopper1) //1
        {
            if(eeprom.keepPositionFlag2 == 0 && eeprom.keepPositionFlag1 == 0 && (eeprom.position == 2 || eeprom.position == 3))
            {
                eeprom.position = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag1 = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
        }
        else if (position.absolutePositionOfRenishaw < position.stopper2) //2
        {
            if(( eeprom.keepPositionFlag1 == 0 || eeprom.keepPositionFlag1 == 1) && eeprom.position == 1)
            {
                eeprom.position = 2;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if( eeprom.keepPositionFlag1 == 2 && eeprom.position == 2)
            {
                eeprom.position = 2;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 0 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 3)
            {
                eeprom.keepPositionFlag2 = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag2 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
        }
    }
    else if (rotationMode == RotationMode.Reversed)
    {
        if((position.absolutePositionOfRenishaw > position.stopper2) || (position.absolutePositionOfRenishaw < position.stopper1))  //3
        {
            if(eeprom.keepPositionFlag2 == 0 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag2 = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if((eeprom.keepPositionFlag1 == 0 || eeprom.keepPositionFlag1 == 1 ) && eeprom.position == 1)
            {
                eeprom.position = 3;
                eeprom.keepPositionFlag1 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 1 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag1 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 3)
            {
                eeprom.keepPositionFlag2 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
        }
        else if(position.absolutePositionOfRenishaw > position.stopper1 && (position.absolutePositionOfRenishaw < position.stopper2 - ((int32)10000 * 4))) //1
        {
            if(eeprom.keepPositionFlag2 == 0 && eeprom.keepPositionFlag1 == 0 && (eeprom.position == 2 || eeprom.position == 3))
            {
                eeprom.position = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag1 = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
        }
        else if (position.absolutePositionOfRenishaw < position.stopper2 && position.absolutePositionOfRenishaw > position.stopper2 - ((int32)10000 * 4) ) //2
        {
            if(( eeprom.keepPositionFlag1 == 0 || eeprom.keepPositionFlag1 == 1) && eeprom.position == 1)
            {
                eeprom.position = 2;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if( eeprom.keepPositionFlag1 == 2 && eeprom.position == 2)
            {
                eeprom.position = 2;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 0 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 3)
            {
                eeprom.keepPositionFlag2 = 1;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
            if (eeprom.keepPositionFlag2 == 1 && eeprom.keepPositionFlag1 == 0 && eeprom.position == 2)
            {
                eeprom.keepPositionFlag2 = 0;
                eeprom.writeFlag = !eeprom.writeFlag;
            }
        }
    }
}


// Обработка байта, формирование посылки
void SciRx(Uint16 sciRxBuffer)
{
    unsigned char i;
    unsigned char checksum = 0;

    if ((sciRxBufferArray[1] == SciCommand.HisAddress) && (sciRxBufferArray[0] == SciCommand.Beginning))
    {
        if (sciRxBufferCounter < (SciCommand.MessageLength - 1))
        {
            sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;
            sciRxBufferCounter++;
        }
        else if (sciRxBufferCounter == SciCommand.MessageLength - 1)
        {
            sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;

            if (sciRxBufferArray[SciCommand.MessageLength - 2] == SciCommand.End)
            {
                checksum = 0;
                for (i = 0; i < SciCommand.MessageLength - 1; i++)
                    checksum = sciRxBufferArray[i] ^ checksum;

                if (sciRxBufferArray[SciCommand.MessageLength - 1] == checksum)
                {
                    for (i = 0; i < SciCommand.MessageLength; i++)
                        sciRxArray[i] = sciRxBufferArray[i];
                    sciRxCommand.rxMassageIsCorrect = TRUE;
                }
            }

            for (i = 0; i < SciCommand.MessageLength; i++)
                sciRxBufferArray[i] = 0;
            sciRxBufferCounter = 0;
        }
    }
    else if ((sciRxBufferCounter == 0) && (sciRxBuffer == SciCommand.Beginning))
    {
        sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;
        sciRxBufferCounter++;
    }
    else if ((sciRxBufferCounter == 1) && (sciRxBuffer == SciCommand.HisAddress))
    {
        sciRxBufferArray[sciRxBufferCounter] = sciRxBuffer;
        sciRxBufferCounter++;
    }
    else
    {
        for (i = 0; i < SciCommand.MessageLength; i++)
            sciRxBufferArray[i] = 0;
        sciRxBufferCounter = 0;
    }

    PieCtrlRegs.PIEACK.all|=0x100;

}


void SciSendData(unsigned char lenght, unsigned char *arr)
{

    int i;
    for (i = 0; i < lenght; i++)
    {
        while ( ScibRegs.SCICTL2.bit.TXEMPTY == 0);
        ScibRegs.SCITXBUF.all = arr[i];
    }
}

// TODO
// Разбор посылки
void SciParseRxData()
{
    sciRxCommand.myAddress = sciRxArray[2];

    if(sciRxCommand.myAddress == SciCommand.MyAddress)
    {
        sciRxCommand.operatingMode = sciRxArray[5];

        if (sciRxCommand.operatingMode == SciOperatingMode.Normal)
            SciParseNormal();
        else
            SciParseReload();

        sciRxCommand.rxMassageIsCorrect = 0;
    }
}



//TODO
// Сборка посылки на отправку
void SciPrepareTxData()
{
    unsigned char i;
    memset(&sciTxArray, 0, sizeof(sciTxArray));

    sciTxArray[0] = SciCommand.Beginning;
    sciTxArray[1] = SciCommand.MyAddress;
    sciTxArray[2] = SciCommand.HisAddress;

    if (sciRxCommand.operatingMode == SciOperatingMode.Normal)
        SciPrepareNormal();
    else
        SciPrepareReload();

    sciTxArray[30] = SciCommand.End;
    sciTxArray[31] = sciTxArray[0];

    for (i = 1; i < (SciCommand.MessageLength - 1); i++)
        sciTxArray[SciCommand.MessageLength - 1] = sciTxArray[31] ^ sciTxArray[i];
}


void DataBiss(void)
{
    myPosition= bissc_data_struct.position;
}



void Wait(long delay)
{
    for (timer.cpuTimer = 0; timer.cpuTimer < delay; );
}

void CheckPwm()
{
    if(heatStatus == 2)
    {
        if(resetDriver)
        {
            if (GpioDataRegs.GPADAT.bit.GPIO1 == 1)
            {
                GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
                GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
                GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
            }
        }
        else
        {
            if (GpioDataRegs.GPADAT.bit.GPIO1 == 0)
            {
                GpioDataRegs.GPASET.bit.GPIO1 = 1;
                GpioDataRegs.GPASET.bit.GPIO3 = 1;
                GpioDataRegs.GPASET.bit.GPIO5 = 1;
            }
        }
    }
    else
    {
        if (GpioDataRegs.GPADAT.bit.GPIO1 == 0)
        {
            GpioDataRegs.GPASET.bit.GPIO1 = 1;
            GpioDataRegs.GPASET.bit.GPIO3 = 1;
            GpioDataRegs.GPASET.bit.GPIO5 = 1;
        }
    }
}

void CheckSci()
{
    if (sciRxCommand.rxMassageIsCorrect == TRUE)
    {
        tmpCounter[1]++;
        SciParseRxData();
        SciPrepareTxData();
        SciSendData(SciCommand.MessageLength, sciTxArray);
        tmpCounter[2]++;
    }
    if(sciTxCommand.operatingMode == SciOperatingMode.ReloadIsEnded)
    {
        firmwareLengthInRam = firmwareLength;
        int i = 0;
        firmwareSumInRam = 0;
        for (i = 0; i < firmwareLength; i++)
            firmwareSumInRam = firmwareSumInRam ^ firmwareBuffer[i];

        JUMP_TO_FLASHAPI;
    }
}



void InitVariables()
{
    int i;
    for (i = 0; i < 32; i++)
        sciRxBufferArray[i] = 0;

    for (i = 0; i < 10; i++)
    {
        tmpCounter[i] = 0;
        tmpFlag[i] = 0;
    }

    for (i = 0; i < TARGET_DIVIDER; i++)
        targetDivider.newPositionOfTargetDesignation[i] = 0;

    sciRxCommand.arretirStatus = ArretirStatus.Closed;

    position.positionOfArretirLong = (long)position.positionOfArretir * 256;
    position.positionOfArretir = (long)position.positionOfArretir;

    timer.globalTimer = 0;

    findPosition.pidSpeed = phase.phaseDivider / 10;
    findPosition.mode = 0;
}



void StartCpuTimer()
{
    CpuTimer0Regs.TCR.bit.TSS = 0;
}

void ReadTmp100()
{
    InitI2c();
    I2caRegs.I2CCNT     = 1;    // read 2 byte temperature
    I2caRegs.I2CMDR.all = 0x6C20;

    counterForI2c = 0;
    while ((I2caRegs.I2CSTR.bit.RRDY == 0)&&(counterForI2c<10000)) counterForI2c++;   // wait for 1st byte
    if (I2caRegs.I2CSTR.bit.RRDY != 0)
    {
        temperature = I2caRegs.I2CDRR.all << 8;         // read upper 8 Bit (integers)
        temperature = temperature/256;
    }
    I2caRegs.I2CMDR.bit.IRS = 0;    // Reset the I2C module
    I2caRegs.I2CMDR.bit.IRS = 1;    // Take I2C out of reset
}

void ReadFromEeprom()
{
    ConfigureEeprom();
    eeprom.readFlag = 1;
    eeprom.oldWriteFlag = eeprom.writeFlag;
    I2caRegs.I2CMDR.bit.IRS = 0;    // Reset the I2C module
    I2caRegs.I2CMDR.bit.IRS = 1;    // Take I2C out of reset

    while(I2cMsgIn.MsgStatus != I2C_MSGSTAT_INACTIVE)
    {
        if(eeprom.readFlag == 1)
        {
            if(I2cMsgIn.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
            {
                while(I2CA_ReadData(&I2cMsgIn) != I2C_SUCCESS){}
                CurrentMsgPtr = &I2cMsgIn;
                I2cMsgIn.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
            }
            else if(I2cMsgIn.MsgStatus == I2C_MSGSTAT_RESTART)
            {
                while(I2CA_ReadData(&I2cMsgIn) != I2C_SUCCESS){}
                CurrentMsgPtr = &I2cMsgIn;
                eeprom.readFlag = 0;
                I2cMsgIn.MsgStatus = I2C_MSGSTAT_READ_BUSY; //Р·РґРµСЃСЊ РїРµСЂРµС…РѕРґ РґР»СЏ С‡С‚РµРЅРёСЏ
            }
        }
    }
    if(I2cMsgIn.MsgStatus == I2C_MSGSTAT_INACTIVE)
    {
        IER &= 0xFF7F;
    }
}

void WriteToEeprom()
{
    if ( eeprom.oldWriteFlag != eeprom.writeFlag )
    {
        ConfigureEeprom();
        I2cMsgOut.MsgBuffer[0] = eeprom.position;
        I2cMsgOut.MsgBuffer[1] = eeprom.keepPositionFlag1;
        I2cMsgOut.MsgBuffer[2] = eeprom.keepPositionFlag2;
        eeprom.error = I2CA_WriteData(&I2cMsgOut);
        if(eeprom.error == I2C_SUCCESS)
        {
            eeprom.writeCounter++;
            eeprom.oldWriteFlag = eeprom.writeFlag;
        }
        I2caRegs.I2CMDR.bit.IRS = 0;    // Reset the I2C module
        I2caRegs.I2CMDR.bit.IRS = 1;    // Take I2C out of reset
    }
}

void ReadRenishaw()
{
    PosEncoderSuite();
    DataBiss();

    position.absolutePositionOfRenishaw = (Uint32)(myPosition >> 8);
    position.absolutePositionOfRenishawLong = myPosition;
}

// Вычисляется Положение Renishaw относительно арретира
void SetPositionOfRenishaw()
{
    if (rotationMode == RotationMode.Reversed)
    {
        if (eeprom.position == 3 || (eeprom.position == 1 && (position.absolutePositionOfRenishaw < position.positionOfArretir)))
        {
            definitionOfOrientation = TRUE;
            positiveDirection = FALSE;
        }
        if (eeprom.position == 2 )
        {
            definitionOfOrientation = TRUE;
            positiveDirection = TRUE;
        }
        if (eeprom.position == 1 && position.absolutePositionOfRenishaw > position.positionOfArretir )
        {
            definitionOfOrientation = TRUE;
            positiveDirection = TRUE;
        }

        if (positiveDirection)
        {
            if ( eeprom.position == 2 )
            {
                position.positionOfRenishaw = position.absolutePositionOfRenishaw - position.positionOfArretir;
                position.positionOfRenishawLong = position.absolutePositionOfRenishawLong - position.positionOfArretirLong;
            }
            if (eeprom.position == 1 )
            {
                if(position.absolutePositionOfRenishaw > position.positionOfArretir)
                {
                    position.positionOfRenishaw = position.absolutePositionOfRenishaw - position.positionOfArretir;
                    position.positionOfRenishawLong = position.absolutePositionOfRenishawLong - position.positionOfArretirLong;
                }
                else
                {
                    position.positionOfRenishaw = (int32)((int32)position.absolutePositionOfRenishaw  - (int32)position.positionOfArretir);
                    position.positionOfRenishawLong = position.absolutePositionOfRenishawLong  - position.positionOfArretirLong;
                }
            }
        }
        else
        {
            if (position.absolutePositionOfRenishaw < position.positionOfArretir)
            {
                position.positionOfRenishaw = (int32)((int32)position.absolutePositionOfRenishaw - (int32)position.positionOfArretir);
                position.positionOfRenishawLong = position.absolutePositionOfRenishawLong - position.positionOfArretirLong;
            }
            else
            {
                position.positionOfRenishaw = - (int32)position.positionOfArretir - ((65536 * 4) - (int32)position.absolutePositionOfRenishaw);
                position.positionOfRenishawLong = - (int32)position.positionOfArretir - (int32)((65536 * 4) - position.positionOfArretir) * 256;
            }
        }
    }
    else if(rotationMode == RotationMode.Normal)
    {
        if (eeprom.position == 3 || (eeprom.position == 1 && (position.absolutePositionOfRenishaw < position.positionOfArretir) && (position.absolutePositionOfRenishaw > (position.stopper1+30))))
        {
            definitionOfOrientation = TRUE;
            positiveDirection = FALSE;
        }
        if (eeprom.position == 2 )
        {
            definitionOfOrientation = TRUE;
            positiveDirection = TRUE;
        }
        if (eeprom.position == 1 && position.absolutePositionOfRenishaw>position.positionOfArretir )
        {
            definitionOfOrientation = TRUE;
            positiveDirection = TRUE;
        }


        if (positiveDirection)
        {
            if ( eeprom.position == 2 )
            {
                position.positionOfRenishaw = position.absolutePositionOfRenishaw + (65536 * 4) - position.positionOfArretir;
                position.positionOfRenishawLong = position.absolutePositionOfRenishawLong + (int32)67108864 - position.positionOfArretirLong;
            }
            if (eeprom.position == 1 )
            {
                if(position.absolutePositionOfRenishaw > position.positionOfArretir)
                {
                    position.positionOfRenishaw = position.absolutePositionOfRenishaw - position.positionOfArretir;
                    position.positionOfRenishawLong = position.absolutePositionOfRenishawLong - position.positionOfArretirLong;
                }
                else
                {
                    position.positionOfRenishaw = (int32)((int32)position.absolutePositionOfRenishaw  - (int32)position.positionOfArretir);
                    position.positionOfRenishawLong = position.absolutePositionOfRenishawLong  - position.positionOfArretirLong;
                }
            }
        }
        else
        {
            position.positionOfRenishaw = (int32)((int32)position.absolutePositionOfRenishaw - (int32)position.positionOfArretir);
            position.positionOfRenishawLong = position.absolutePositionOfRenishawLong - position.positionOfArretirLong;
        }
    }
}

// Библиотечные функции управления двигателем
// Здесь отправляем фазу
void SetMotorPwm()
{
    park1.Angle = phase.phase;//rg1.Out;//angle;//rg1.Out;
    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1)
    // ------------------------------------------------------------------------------
    //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
    // ------------------------------------------------------------------------------
    vqTesting = amplitude;

    ipark1.Ds = VdTesting;
    ipark1.Qs = vqTesting;
    ipark1.Sine = park1.Sine;
    ipark1.Cosine = park1.Cosine;
    IPARK_MACRO(ipark1)

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
    //    DlogCh1 = _IQtoQ15(volt1.Valpha);
    //    DlogCh2 = _IQtoQ15(clarke1.Alpha);
    //    DlogCh3 = _IQtoQ15(volt1.Vbeta);
    //    DlogCh4 = _IQtoQ15(clarke1.Beta);

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
}

// Функция нахождения положения при старте
// Сначала крутится в одну сторону, потом в другую, потом находит арретир
void FindPosition()
{
    // Крутится в одну сторону
    if (findPosition.firstPointWasFound == 0)
    {
        pid.wholePart = findPosition.pidSpeed;
        if (position.averageVelocity == 0)
        {
            findPosition.velocityCounter++;

            if (findPosition.velocityCounter > findPosition.stopTime)
            {
                findPosition.firstPointWasFound = 1;
                heatRotation.timer = 0;
                findPosition.velocityCounter = 0;
            }
        }
    }
    else if (findPosition.secondPointWasFound == 0)
    {
        pid.wholePart = - findPosition.pidSpeed;
        if (position.averageVelocity == 0)
        {
            findPosition.velocityCounter++;

            if (findPosition.velocityCounter > findPosition.stopTime)
            {
                findPosition.secondPointWasFound = 1;
                findPosition.velocityCounter = 0;
                heatRotation.timer = 0;
            }
        }

        if ((position.absolutePositionOfRenishaw >= position.positionOfArretir - 400) && (position.absolutePositionOfRenishaw <= position.positionOfArretir + 400))
        {
            if (findPosition.atArretirPoint == 0)
            {
                findPosition.arretirPositionCounter++;
                findPosition.atArretirPoint = 1;
            }
        }
        else
            findPosition.atArretirPoint = 0;

    }
    else if (findPosition.counterWasCalculated == 0)
    {
        findPosition.arretirPositionCounter = findPosition.arretirPositionCounter / 2 + 1;
        findPosition.counterWasCalculated = 1;
    }
    else if (findPosition.arretirPositionCounter > 0)
    {
        pid.wholePart = findPosition.pidSpeed;

        if ((position.absolutePositionOfRenishaw >= position.positionOfArretir - 400) && (position.absolutePositionOfRenishaw <= position.positionOfArretir + 400))
        {
            if (findPosition.atArretirPoint == 0)
            {
                findPosition.arretirPositionCounter--;
                findPosition.atArretirPoint = 1;
            }
        }
        else
            findPosition.atArretirPoint = 0;
    }
    else
    {
        findPosition.mode = 0;
        eeprom.position = 1;
        eeprom.keepPositionFlag2 = 0;
        eeprom.keepPositionFlag1 = 0;
        findPosition.arretirPositionCounter = 0;
        findPosition.firstPointWasFound = 0;
        findPosition.secondPointWasFound = 0;
        findPosition.counterWasCalculated = 0;
        findPosition.atArretirPoint = 0;
    }
}

void SetTarget()
{
    sciRxCommand.target = (int32)(((((Uint32)sciRxCommand.target3<<8)|(Uint32)sciRxCommand.target2)<<16)|
            (((Uint32)sciRxCommand.target1<<8)|(Uint32)sciRxCommand.target0)) / 256;

    //positionArray[positionCounter++ % 1000] = sciRxCommand.target;

    // Обработка ЦУ в области 180 градусов
    if ((eeprom.position == 3) && (sciRxCommand.target > ((int32)31767 * 4)) &&
            ((position.positionOfRenishaw < ((int32)-32767 * 4)) || ((((int32)32767 * 4) + position.positionOfRenishaw + ((int32)32767 * 4) - sciRxCommand.target) < ((int32)1000 * 4))))
        sciRxCommand.target = sciRxCommand.target - ((int32)65535 * 4);

    if ((eeprom.position == 2) && (sciRxCommand.target < ((int32)-31767 * 4)) &&
            ((position.positionOfRenishaw > ((int32)32767 * 4)) || ((((int32)32767 * 4) - position.positionOfRenishaw + ((int32)32767 * 4) + sciRxCommand.target) < ((int32)1000 * 4))))
        sciRxCommand.target = sciRxCommand.target + ((int32)65535 * 4);


}

void SetOtherTarget()
{
    if (sciRxCommand.otherTarget3 != 255)
    {
        sciRxCommand.otherTarget = (int32)(((((Uint32)sciRxCommand.otherTarget3<<8) | (Uint32)sciRxCommand.otherTarget2)<<16)|
                        (((Uint32)sciRxCommand.otherTarget1<<8) | (Uint32)sciRxCommand.otherTarget0)) / 256;
    }

}





// TODO SciParseNormal()
void SciParseNormal()
{
    if (technologicalMode.resetArretirStatus == FALSE)
        sciRxCommand.arretirStatus = (sciRxArray[3] >> 1) & 0x03;

    if (technologicalMode.resetHoldPosition == FALSE)
        sciRxCommand.holdPosition = (sciRxArray[3] >> 3) & 0x01;

    sciRxCommand.stabilization = (sciRxArray[3] >> 4) & 0x01;
    stabilizationFlag = sciRxCommand.stabilization;
    sciRxCommand.targeting = (sciRxArray[3] >> 6) & 0x01;
    sciRxCommand.heatStatus = sciRxArray[4];

        sciRxCommand.target0 = sciRxArray[6];
        sciRxCommand.target1 = sciRxArray[7];
        sciRxCommand.target2 = sciRxArray[8];
        sciRxCommand.target3 = sciRxArray[9];

    sciRxCommand.otherTarget0 = sciRxArray[10];
    sciRxCommand.otherTarget1 = sciRxArray[11];
    sciRxCommand.otherTarget2 = sciRxArray[12];
    sciRxCommand.otherTarget3 = sciRxArray[13];

    if(definitionOfOrientation)
    {
        SetTarget();
        SetOtherTarget();
    }

    if (sciRxArray[28])
    {
        if (eeprom.positionIsRecieved == 0)
        {
            eeprom.position = (sciRxArray[28] >> 2) & 0x03;
            if (eeprom.position == 0)
                eeprom.position = 1;
            eeprom.keepPositionFlag1 = (sciRxArray[28] >> 1) & 0x01;
            eeprom.keepPositionFlag2 = (sciRxArray[28]) & 0x01;
            eeprom.positionIsRecieved = 1;
        }

        technologicalRotation.rotationCommand = (sciRxArray[28] >> 4) & 0x03;
        if (technologicalRotation.rotationCommand)
            if (technologicalRotation.oldRotationCommand != technologicalRotation.rotationCommand)
            {
                technologicalRotation.rotationFlag = technologicalRotation.rotationCommand;
                technologicalRotation.oldRotationCommand = technologicalRotation.rotationCommand;
            }
    }
    else
        technologicalRotation.oldRotationCommand = 0;




    if (sciRxCommand.heatStatus == 1 || sciRxCommand.heatStatus == 2)
    {
        sciTxCommand.heatStatus = sciRxCommand.heatStatus;
        heatStatus = sciRxCommand.heatStatus;
    }


    SplitTarget();

    ///
}





void SciPrepareNormal()
{
    sciTxCommand.operatingMode = SciOperatingMode.Normal;

    Uint32 tmpPositionOfRenishawLong = position.absolutePositionOfRenishawLong;
    positionArray[positionCounter++ % 1000] = position.positionOfRenishaw;

    sciTxCommand.positionOfRenishaw0 = 0x000000FF & position.positionOfRenishawLong;
    sciTxCommand.positionOfRenishaw1 = (0x0000FF00 & position.positionOfRenishawLong) >> 8;
    sciTxCommand.positionOfRenishaw2 = (0x00FF0000 & position.positionOfRenishawLong) >> 16;
    sciTxCommand.positionOfRenishaw3 = (0xFF000000 & position.positionOfRenishawLong) >> 24;

    sciTxArray[3]=sciTxCommand.operatingMode;
    sciTxArray[4]=sciTxCommand.heatStatus;


    if (heatStatus == 2)
    {
        sciTxArray[6] = sciTxCommand.positionOfRenishaw0;
        sciTxArray[7] = sciTxCommand.positionOfRenishaw1;
        sciTxArray[8] = sciTxCommand.positionOfRenishaw2;
        sciTxArray[9] = sciTxCommand.positionOfRenishaw3;
    }
    else
    {
        sciTxArray[6] = 0;
        sciTxArray[7] = 0;
        sciTxArray[8] = 0;
        sciTxArray[9] = 0;
    }

    sciTxArray[10] = sciTxCommand.velocityHi;
    sciTxArray[11] = sciTxCommand.velocityLo;
    sciTxArray[12] = sciTxCommand.accelerationHi;
    sciTxArray[13] = sciTxCommand.accelerationLo;

    sciTxArray[22] = (char)temperature;
    sciTxArray[23] = (char)temperatureDS1624;
    sciTxArray[24] = sciTxCommand.heatStatus;
    sciTxArray[25] = (char)(position.averageVelocity) & 0xFF;
    sciTxArray[26] = ((char)(position.averageVelocity) >> 8)& 0xFF;
    sciTxArray[27] = ((char)(eeprom.position) << 2) | ((char)(eeprom.keepPositionFlag1) << 1) | ((char)(eeprom.keepPositionFlag2));
    sciTxArray[28] = 0;
    sciTxArray[29] = 0;
}

/*
    Плата видео принимает посылки по 64 байта (32 слова)
    Дробит на 3 посылки и передает
    Первая часть: 22 байта
    Вторая часть: 22 байта
    Третья часть: 20 байт
 */

void SciParseReload()
{
    if (sciRxCommand.operatingMode == SciOperatingMode.BeginReload)
        SciParseBeginReload();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.RecieveFirstPart) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseRecieveFirstPart();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.RecieveSecondPart) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseRecieveSecondPart();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.RecieveThirdPart) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseRecieveThirdPart();

    else if ((sciRxCommand.operatingMode == SciOperatingMode.EndReload) &&
            (sciRxCommand.operatingMode != sciTxCommand.operatingMode))
        SciParseEndReload();

    sciTxCommand.numberOfPacket0 = sciRxCommand.numberOfPacket0;
    sciTxCommand.numberOfPacket1 = sciRxCommand.numberOfPacket1;
}



void SciParseBeginReload()
{
    int i = 0;
    sciRxCommand.firmwareLength0 = sciRxArray[3];
    sciRxCommand.firmwareLength1 = sciRxArray[4];
    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];

    sciRxCommand.firmwareLength = (Uint32)(((Uint32)sciRxCommand.firmwareLength1 << 8) | (Uint32)sciRxCommand.firmwareLength0);

    firmwareLength = sciRxCommand.firmwareLength / 2;
    firmwareCounter = 0;
    for (i = 0; i < 20000; i++)
        firmwareBuffer[i] = 0;




    sciTxCommand.operatingMode = SciOperatingMode.ReadyToReload;
}

void SciParseRecieveFirstPart()
{
    int i = 0;
    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];
    sciRxCommand.numberOfPacket = sciRxCommand.numberOfPacket0 | (sciRxCommand.numberOfPacket1 << 8);
    firmwareCounter = sciRxCommand.numberOfPacket * 64 / 2; // Считает от последней метки с 0-го

    for (i = 8; i < 30; i += 2)
    {
        if (firmwareCounter < firmwareLength)
        {
            firmwareBuffer[firmwareCounter] = (sciRxArray[i + 1] << 8) | sciRxArray[i];
            firmwareCounter++;
        }
        else
            break;
    }

    sciTxCommand.operatingMode = SciOperatingMode.FirstPartIsRecieved;

}

void SciParseRecieveSecondPart()
{
    int i = 0;
    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];
    sciRxCommand.numberOfPacket = sciRxCommand.numberOfPacket0 | (sciRxCommand.numberOfPacket1 << 8);
    firmwareCounter = sciRxCommand.numberOfPacket * 64 / 2 + 11; // Считает от последней метки с 11-го

    for (i = 8; i < 30; i += 2)
    {
        if (firmwareCounter < firmwareLength)
        {
            firmwareBuffer[firmwareCounter] = (sciRxArray[i + 1] << 8) | sciRxArray[i];
            firmwareCounter++;
        }
        else
            break;
    }

    sciTxCommand.operatingMode = SciOperatingMode.SecondPartIsRecieved;
}

void SciParseRecieveThirdPart()

{
    int i = 0;

    sciRxCommand.numberOfPacket0 = sciRxArray[6];
    sciRxCommand.numberOfPacket1 = sciRxArray[7];
    sciRxCommand.numberOfPacket = sciRxCommand.numberOfPacket0 | (sciRxCommand.numberOfPacket1 << 8);
    firmwareCounter = sciRxCommand.numberOfPacket * 64 / 2 + 22; // Считает от последней метки с 22-го

    for (i = 8; i < 28; i += 2)
    {
        if (firmwareCounter < firmwareLength)
        {
            firmwareBuffer[firmwareCounter] = (sciRxArray[i + 1] << 8) | sciRxArray[i];
            firmwareCounter++;
        }
        else
            break;
    }

    sciTxCommand.operatingMode = SciOperatingMode.ThirdPartIsRecieved;

}

void SciParseEndReload()
{
    int i = 0;
    firmwareSum = 1;
    for (i = 0; i < firmwareLength; i++)
        firmwareSum += (unsigned short)firmwareBuffer[i];

    sciTxCommand.operatingMode = SciOperatingMode.ReloadIsEnded;
}

void SciPrepareReload()
{
    sciTxArray[3] = sciTxCommand.operatingMode;
    sciTxArray[4] = sciTxCommand.numberOfPacket0;
    sciTxArray[5] = sciTxCommand.numberOfPacket1;

    if (sciRxCommand.operatingMode == SciOperatingMode.EndReload)
    {
        sciTxArray[6] = firmwareSum & 0xFF;
        sciTxArray[7] = (firmwareSum >> 8) & 0xFF;
    }
}


void A0(void)
{
    if(CpuTimer0Regs.TCR.bit.TIF == 1)
    {
        CpuTimer0Regs.TCR.bit.TIF = 1;  // clear flag

        //-----------------------------------------------------------
        (*A_Task_Ptr)();        // jump to an A Task (A1,A2,A3,...)
        //-----------------------------------------------------------

        VTimer0[0]++;           // virtual timer 0, instance 0 (spare)
        SerialCommsTimer++;
    }
    Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
}

void B0(void)
{
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

//=================================================================================
//  A - TASKS (executed in every 50 usec)
//=================================================================================


//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
    if (timer.i2cTimer % 100 == 0)
        ReadTmp100();

    A_Task_Ptr = &A2;
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{
    //WriteToEeprom();


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



float RenishawToDegree(float renishaws)
{
    /*
     1 degree = 182 * 4 Renishaws
     3600 seconds = 182 * 4 Renishaws
     4,945 seconds = 1 Renishaws
     */

    return ((float)renishaws / 182 / 4);
}

float DegreeToRenishaw(float degrees)
{
    return ((float)degrees * 182 * 4);
}

float DegreeToRadian(float degree)
{
    return (degree * PI / 180);
}

float RadianToDegree(float radian)
{
    return (radian * 180 / PI);
}





float atanVar = 0;
float degreeVar = 0;
// Get new target designation during stabilization
// Input alpha and beta are in renishaws
long Stabilization(float inputAlpha, float inputBeta)
{
    //float tmpInputBeta =
    point.alpha = - RenishawToDegree(inputAlpha);
    point.beta = RenishawToDegree(inputBeta);

    if (point.alpha == 0)
        point.alpha = 0.001;
    else if (point.alpha == 90)
        point.alpha = 90.001;
    else if (point.alpha == 180)
        point.alpha = 180.001;
    else if (point.alpha == 270)
        point.alpha = 270.001;

    if (point.beta == 0)
        point.beta = 0.001;
    else if (point.beta == 90)
        point.beta = 90.001;
    else if (point.beta == 180)
        point.beta = 180.001;
    else if (point.beta == 270)
        point.beta = 270.001;


    //    atanVar = RadianToDegree(atan(degreeVar));
    //atanVar = tan(DegreeToRadian(degreeVar));
    float alphaInRadians = DegreeToRadian(point.alpha);
    float betaInRadians = DegreeToRadian(point.beta);

    Point pointInRadians;
    pointInRadians.alpha = alphaInRadians;
    pointInRadians.beta = betaInRadians;

    // Get vector from alpha, beta
    vector = SphereToRectangle(pointInRadians);

    // Get transition matrix
    if (stabilizationMode == MEMS)
        matrix = GetTransitionMatrix(DegreeToRadian((float)memsAngles.tangage / 3600),
                                     DegreeToRadian((float)memsAngles.course / 3600),
                                     DegreeToRadian((float)memsAngles.careen / 3600));
    else if (stabilizationMode == BINS)
        matrix = GetTransitionMatrix(DegreeToRadian((float)binsAngles.careen / 3600),
                                     DegreeToRadian((float)binsAngles.tangage / 3600),
                                     DegreeToRadian((float)binsAngles.course / 3600));

    // Multiply matrixes
    newVector = MultiplyMatrixes(matrix, vector);


    // Get new alpha, beta from new vector

    float tmpVarX = 0, tmpVarY = 0;
    float sina = 0;
    float cosa = 0;

    tmpVarY = asin(newVector.y);
    sina = newVector.z / cos(tmpVarY);
    cosa = newVector.x / cos(tmpVarY);

    if (cosa >= 0)
        tmpVarX = asin(sina);
    if (cosa < 0)
        tmpVarX = - asin(sina) + PI;

    newPoint.alpha = RadianToDegree(tmpVarX);
    if (newPoint.alpha > 180)
        newPoint.alpha = newPoint.alpha - 360;
    else if (newPoint.alpha < -180)
        newPoint.alpha = newPoint.alpha + 360;

    // Поставить ограничения!!!!!!!
    newPoint.beta = RadianToDegree(tmpVarY);


    return DegreeToRenishaw(point.alpha - newPoint.alpha);
}



