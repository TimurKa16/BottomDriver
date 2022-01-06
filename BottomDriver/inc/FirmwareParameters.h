/*
    Параметры прошивок
    Для удобства работы сохранены данные Renishaw 16 разрядов (65536)
 * 4 - Повышение точности выхода на ЦУ
 */

#include <stdbool.h>
#include <stdint.h>
#include "F28x_Project.h"
#include "DeviceNumber.h"
#include "main2.h"


void InitFirmwareParameters()
{
    if (DEVICE_NUMBER == BOTTOM_AONIDA1)
    {
        sign = 1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43793 * 4;
        position.stopper2 = (Uint32)10951 * 4;
        position.positionOfArretir = 59974 * 4;

        PositioningAccuracy1 = 1;
        PositioningAccuracy2 = 200;
        PositioningAccuracy3 = 500;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0006);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.02);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.12);
        VqTestingCold = _IQ(0.18);
        VqTestingVeryCold = _IQ(0.23);
        VqTestingHot = _IQ(0.08);

        Amplitude.Max = _IQ(0.1);
        Amplitude.HoldPosition = _IQ(0.1);

        pid.pCoefLo = 70;
        pid.iCoefLo = 20;
        pid.dCoefLo = 400;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 30;
        pid.dCoefHi = 2000;

        pid.iPartLimit = 5000;

        phase.electricalMotorNilPosition = 2000;
        phase.mechToElectrCoef = 1;
        phase.phaseDivider = (long)8192 * 4;
        phase.maxPhase = (int32)2048 * 4;
    }

    if (DEVICE_NUMBER == BOTTOM_181001)
    {
        sign = 1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43793 * 4;
        position.stopper2 = (Uint32)10951 * 4;
        position.positionOfArretir = 59974 * 4;

        PositioningAccuracy1 = 1;
        PositioningAccuracy2 = 200;
        PositioningAccuracy3 = 500;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0006);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.02);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.12);
        VqTestingCold = _IQ(0.18);
        VqTestingVeryCold = _IQ(0.23);
        VqTestingHot = _IQ(0.08);

        Amplitude.Max = _IQ(0.3);
        Amplitude.HoldPosition = _IQ(0.35);

        pid.pCoefLo = 70;
        pid.iCoefLo = 20;
        pid.dCoefLo = 400;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 30;
        pid.dCoefHi = 2000;

        pid.iPartLimit = 5000;

        phase.electricalMotorNilPosition = -2555 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = (long)8192 * 4;
        phase.maxPhase = (int32)2048 * 4;
    }

    if (DEVICE_NUMBER == BOTTOM_191003)     // Old BOTTOM_181002
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43760 * 4;
        position.stopper2 = (Uint32)10989 * 4;
        position.positionOfArretir = (Uint32)60081 * 4;

        PositioningAccuracy1 = 3;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 600;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;


        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0006);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.02);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.18);
        VqTestingCold = _IQ(0.23);
        VqTestingVeryCold = _IQ(0.38);   //ј
        VqTestingHot = _IQ(0.08);

        Amplitude.Heating = _IQ(0.38);

        Amplitude.Max = _IQ(0.14);

        pid.pCoefLo = -1000;
        pid.iCoefLo = -30;
        pid.dCoefLo = -1000;

        pid.pCoefHi = -1000;
        pid.iCoefHi = -30;
        pid.dCoefHi = -2000;

        pid.iPartLimit = 5000;

        phase.electricalMotorNilPosition = 700 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = (long)8192 * 4;
        phase.maxPhase = (int32)2048 * 4;
    }

    if (DEVICE_NUMBER == BOTTOM_181003)
    {
        sign = 1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)45342 * 4;
        position.stopper2 = (Uint32)9401 * 4;
        position.positionOfArretir = (Uint32)59994 * 4;

        PositioningAccuracy1 = 3;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 600;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0006);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.02);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.12);
        VqTestingCold = _IQ(0.18);
        VqTestingVeryCold = _IQ(0.23);
        VqTestingHot = _IQ(0.08);


        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_191001)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43721 * 4;
        position.stopper2 = (Uint32)10802 * 4;
        position.positionOfArretir = (Uint32)60080 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 600;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0006);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.02);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.12);
        VqTestingCold = _IQ(0.18);
        VqTestingVeryCold = _IQ(0.23);
        VqTestingHot = _IQ(0.08);
    }

    if (DEVICE_NUMBER == BOTTOM_191002)
    {
        sign = 1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43774 * 4;
        position.stopper2 = (Uint32)10960 * 4;
        position.positionOfArretir = 60000 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 200;
        PositioningAccuracy3 = 500;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0006);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.02);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.12);
        VqTestingCold = _IQ(0.18);
        VqTestingVeryCold = _IQ(0.23);
        VqTestingHot = _IQ(0.08);


        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_181002)     // Old BOTTOM_191003
    {
        sign = 1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)(int32)43778 * 4;
        position.stopper2 = (Uint32)10932 * 4;
        position.positionOfArretir = 59975 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 200;
        PositioningAccuracy3 = 500;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0006);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.02);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.10);
        VqTestingCold = _IQ(0.15);
        VqTestingVeryCold = _IQ(0.23);
        VqTestingHot = _IQ(0.08);

        Amplitude.Max = _IQ(0.1);
        Amplitude.HoldPosition = _IQ(0.1);
        Amplitude.Heating = _IQ(0.23);

        pid.pCoefLo = 300;
        pid.iCoefLo = 10;
        pid.dCoefLo = 300;

        pid.pCoefHi = 300;
        pid.iCoefHi = 10;
        pid.dCoefHi = 300;

        pid.iPartLimit = 1000;

        phase.electricalMotorNilPosition = -1400 * 4;
        phase.mechToElectrCoef = 1;
        phase.phaseDivider = 6553 * 4;
        phase.maxPhase = 1638 * 4;
    }

    if (DEVICE_NUMBER == BOTTOM_191004)  // (прошивка образца 181001)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)175076;
        position.stopper2 = (Uint32)43724;
        position.positionOfArretir = 239042;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 1600;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0.0001);
        SpeedRef2 = _IQ24(0.0002);
        SpeedRef3 = _IQ24(0.0005);
        SpeedRef4 = _IQ24(0.001);
        SpeedRef5 = _IQ24(0.005);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.18);
        VqTestingCold = _IQ(0.23);
        VqTestingVeryCold = _IQ(0.25);
        VqTestingHot = _IQ(0.12);

        Amplitude.Heating = _IQ(0.2);
        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = 1000;
        pid.iCoefLo = 20;
        pid.dCoefLo = 2000;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 20;
        pid.dCoefHi = 2000;

        pid.iPartLimit = 5000;

        phase.electricalMotorNilPosition = -1500;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = (long)8192 * 4;
        phase.maxPhase = 2048 * 4;
    }

    if (DEVICE_NUMBER == BOTTOM_201002)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)44230 * 4;
        position.stopper2 = (Uint32)11410 * 4;
        position.positionOfArretir = 60740 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0005);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.08);
        VqTestingCold = _IQ(0.09);
        VqTestingVeryCold = _IQ(0.11);
        VqTestingHot = _IQ(0.05);
        Amplitude.Heating = _IQ(0.11);

        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_201003)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)44307 * 4;
        position.stopper2 = (Uint32)11570 * 4;
        position.positionOfArretir = 60715 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;
        //
        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0005);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.08);
        VqTestingCold = _IQ(0.08);
        VqTestingVeryCold = _IQ(0.08);
        VqTestingHot = _IQ(0.05);
        Amplitude.Heating = _IQ(0.33);

        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_201004)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)44018 * 4;
        position.stopper2 = (Uint32)11111 * 4;
        position.positionOfArretir = 60585 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0005);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);


        VqTestingNormal = _IQ(0.08);
        VqTestingCold = _IQ(0.08);
        VqTestingVeryCold = _IQ(0.08);
        VqTestingHot = _IQ(0.05);
        Amplitude.Heating = _IQ(0.33);

        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_201005)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)50000 * 4;
        position.stopper2 = (Uint32)10000 * 4;
        position.positionOfArretir = 60000 * 4;

        Amplitude.Max = _IQ(0.15);
        Amplitude.Heating = _IQ(0.6);

        pid.pCoefLo = 500;
        pid.iCoefLo = 0;
        pid.dCoefLo = 1000;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 30;
        pid.dCoefHi = 1000;

        pid.iPartLimit = 2000;

        phase.electricalMotorNilPosition = -3300 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = (int32)8192 * 4;
        phase.maxPhase = 2048 * 4;
    }

    if (DEVICE_NUMBER == BOTTOM_201008)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43923 * 4;
        position.stopper2 = (Uint32)11137 * 4;
        position.positionOfArretir = 60372 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;
        //
        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0005);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.15);
        VqTestingCold = _IQ(0.15);
        VqTestingVeryCold = _IQ(0.15);
        VqTestingHot = _IQ(0.12);

        Amplitude.Max = _IQ(0.14);
        Amplitude.Heating = _IQ(0.6);
        Amplitude.HoldPosition = _IQ(0.14);

        pid.pCoefLo = 500;
        pid.iCoefLo = 30;
        pid.dCoefLo = 2000;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 30;
        pid.dCoefHi = 2000;

        pid.iPartLimit = 5000;

        phase.electricalMotorNilPosition = -1300 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = (int32)8192 * 4;
        phase.maxPhase = 2048 * 4;
    }

    if (DEVICE_NUMBER == BOTTOM_201007)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43836 * 4;
        position.stopper2 = (Uint32)11039 * 4;
        position.positionOfArretir = 60259 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0005);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.16);
        VqTestingCold = _IQ(0.16);
        VqTestingVeryCold = _IQ(0.16);
        VqTestingHot = _IQ(0.12);
        Amplitude.Heating = _IQ(0.7);

        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_201006)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43724 * 4;
        position.stopper2 = (Uint32)10845 * 4;
        position.positionOfArretir = 60663 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;
        //
        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0003);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.1);
        VqTestingCold = _IQ(0.1);
        VqTestingVeryCold = _IQ(0.1);
        VqTestingHot = _IQ(0.1);
        Amplitude.Heating = _IQ(0.1);

        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_201009)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43790 * 4;
        position.stopper2 = (Uint32)10951 * 4;
        position.positionOfArretir = 60330 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;
        //
        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0003);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.15);
        VqTestingCold = _IQ(0.15);
        VqTestingVeryCold = _IQ(0.15);
        VqTestingHot = _IQ(0.12);
        Amplitude.Heating = _IQ(0.7);

        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_201010)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43784 * 4;
        position.stopper2 = (Uint32)10964 * 4;
        position.positionOfArretir = 60250 * 4;

        PositioningAccuracy1 = 5;
        PositioningAccuracy2 = 300;
        PositioningAccuracy3 = 700;
        PositioningAccuracy4 = 5000;
        PositioningAccuracy5 = 5000;

        SpeedRef1 = _IQ24(0);
        SpeedRef2 = _IQ24(0.0003);
        SpeedRef3 = _IQ24(0.001);
        SpeedRef4 = _IQ24(0.003);
        SpeedRef5 = _IQ24(0.013);
        SpeedRef6 = _IQ24(0.02);

        VqTestingNormal = _IQ(0.15);
        VqTestingCold = _IQ(0.15);
        VqTestingVeryCold = _IQ(0.15);
        VqTestingHot = _IQ(0.12);
        Amplitude.Heating = _IQ(0.7);

        Amplitude.Max = _IQ(0.15);

        pid.pCoefLo = _IQ(10);
        pid.iCoefLo = _IQ(0.001);
        pid.dCoefLo = 100;

        pid.pCoefHi = _IQ(10);
        pid.iCoefHi = _IQ(0.05);
        pid.dCoefHi = 100;
    }

    if (DEVICE_NUMBER == BOTTOM_201011)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43784 * 4;
        position.stopper2 = (Uint32)10964 * 4;
        position.positionOfArretir = 60143 * 4;

        Amplitude.Max = _IQ(0.15);
        Amplitude.Heating = _IQ(0.6);

        pid.pCoefLo = 300;
        pid.iCoefLo = 1;
        pid.dCoefLo = 2000;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 0;
        pid.dCoefHi = 300;

        phase.electricalMotorNilPosition = -2080 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = 8192;
        phase.maxPhase = 2048;
    }

    if (DEVICE_NUMBER == BOTTOM_201012)
    {
        sign = 1;

        rotationMode = RotationMode.Reversed;

        position.stopper1 = (Uint32)22000 * 4;
        position.stopper2 = (Uint32)54580 * 4;
        position.positionOfArretir = 38344 * 4;

        Amplitude.Max = _IQ(0.15);
        Amplitude.Heating = _IQ(0.6);

        pid.pCoefLo = 500;
        pid.iCoefLo = 1;
        pid.dCoefLo = 2000;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 0;
        pid.dCoefHi = 300;

        phase.electricalMotorNilPosition = -2080 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = 8192;
        phase.maxPhase = 2048;
    }

    if (DEVICE_NUMBER == BOTTOM_201013)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)43784 * 4;
        position.stopper2 = (Uint32)10964 * 4;
        position.positionOfArretir = 60067 * 4;

        Amplitude.Max = _IQ(0.15);
        Amplitude.Heating = _IQ(0.6);


        pid.pCoefLo = 500;
        pid.iCoefLo = 1;
        pid.dCoefLo = 2000;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 0;
        pid.dCoefHi = 300;

        phase.electricalMotorNilPosition = -2550 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = 8192;
        phase.maxPhase = 2048;
    }

    if (DEVICE_NUMBER == BOTTOM_201014)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;


        position.stopper1 = (Uint32)43784 * 4;
        position.stopper2 = (Uint32)10964 * 4;
        position.positionOfArretir = 60067 * 4;

        Amplitude.Max = _IQ(0.15);
        Amplitude.Heating = _IQ(0.6);

        pid.pCoefLo = 500;
        pid.iCoefLo = 1;
        pid.dCoefLo = 2000;

        pid.pCoefHi = 1000;
        pid.iCoefHi = 0;
        pid.dCoefHi = 300;

        phase.electricalMotorNilPosition = -1200 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = 8192;
        phase.maxPhase = 2048;
    }

    if (DEVICE_NUMBER == BOTTOM_201015)
    {
        sign = -1;

        rotationMode = RotationMode.Normal;

        position.stopper1 = (Uint32)50000 * 4;
        position.stopper2 = (Uint32)10000 * 4;
        position.positionOfArretir = 60072 * 4;

        Amplitude.Max = _IQ(0.15);
        Amplitude.Heating = _IQ(0.6);

        pid.pCoefLo = 500;
        pid.iCoefLo = 0;
        pid.dCoefLo = 2000;

        pid.pCoefHi = 1500;
        pid.iCoefHi = 0;
        pid.dCoefHi = 1400;

        phase.electricalMotorNilPosition = -1950 * 4;
        phase.mechToElectrCoef = -1;
        phase.phaseDivider = 8192;
        phase.maxPhase = 2048;
    }
}
