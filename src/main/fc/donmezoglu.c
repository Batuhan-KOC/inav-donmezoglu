#include "donmezoglu.h"

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/compass/compass.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "drivers/serial_uart.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "io/beeper.h"

int FUZE_STATUS = 0;
serialPortIdentifier_e FUZE_PORT_IDENTIFIER = SERIAL_PORT_UART5;

/* 
  * TAPA STATUS
  *
  * -1 : UNKNOWN
  * 0  : Z (ERROR) 
  * 1  : C (DISABLED)
  * 2  : L (ACTIVE AND LOW SIGNAL)
  * 3  : T (ACTIVE AND HIGH SIGNAL)
 */
int TAPA_STATUS = -1;

bool CHARGE_DISPLAYING = false;

static serialPort_t* dSerialPort = NULL;
static bool dInitializationCompleted = false;
static bool dRcConnection = false;

static bool dInitialSafety = false;

static int dLastAux3State = -1;

static float AUX_EDGE_VALUE_MIN = 1300;
static float AUX_MID_VALUE = 1500;
static float AUX_EDGE_VALUE = 1750;

static float AUX2Value = 0; // KONTROL
static float AUX3Value = 0; // ŞARJ
static float AUX4Value = 0; // GÜVENLİK
static float AUX5Value = 0; // PATLATMA

static bool ControlHigh = false;
static bool SafetyHigh = false;
static bool ExplosionHigh = false;

static bool fuseConnectedMessageReceived = false;
static bool fuseConnected = false;

static bool CHARGE_HIGH_REQUESTED = false;
static bool SHOCK_SENSOR_ACTIVE_REQUESTED = false;
static bool CHARGE_HIGH = false;
static bool SHOCK_SENSOR_ACTIVE = true;

enum FUSE_MESSAGE{
    FM_NONE,
    FM_CONTROL,
    FM_CHARGE,
    FM_SAFETY,
    FM_EXPLOSION
};

enum SHOCK_SENSOR_MESSAGE{
    SSM_NONE,
    SSM_ENABLE,
    SSM_DISABLE
};

static enum FUSE_MESSAGE LAST_SEND_FUSE_MESSAGE = FM_NONE;
static enum SHOCK_SENSOR_MESSAGE LAST_SEND_SHOCK_SENSOR_MESSAGE = SSM_NONE;

static float oldAUX2 = 1500;
static float oldAUX3 = 1500;
static float oldAUX4 = 1500;
static float oldAUX5 = 1500;

void setFuzeData(int value){
    FUZE_STATUS = value;

    if(value == 2){
        CHARGE_DISPLAYING = true;
    }
    else{
        CHARGE_DISPLAYING = false;
    }
}

void waitRcData(void){
    if(dInitializationCompleted && !dRcConnection){
        oldAUX2 = rcData[AUX2];
        oldAUX3 = rcData[AUX3];
        oldAUX4 = rcData[AUX4];
        oldAUX5 = rcData[AUX5];

        // At least one of them should be not equal to default mid value. AUX5 is in failsafe. Do not read it
        if(oldAUX2 != AUX_MID_VALUE || 
           oldAUX3 != AUX_MID_VALUE || 
           oldAUX4 != AUX_MID_VALUE //|| 
            //oldAUX5 != AUX_MID_VALUE
        ){
            dRcConnection = true;
        }
        else{
            //FUZE_STATUS = 8; // Connect Rc Controller
            setFuzeData(8); // Connect Rc Controller
        }
    }

    return;
}

void checkSafety(void){
    static bool firstFuzeDataReceived = false;

    if(dInitializationCompleted && dRcConnection && !dInitialSafety){
        float aux2Val = rcData[AUX2];
        float aux3Val = rcData[AUX3];
        float aux4Val = rcData[AUX4];
        float aux5Val = rcData[AUX5];

        if(!firstFuzeDataReceived){
            static uint32_t bytesWaiting = 0;

            if (dSerialPort->rxBufferHead >= dSerialPort->rxBufferTail) {
                bytesWaiting = dSerialPort->rxBufferHead - dSerialPort->rxBufferTail;
            } else {
                bytesWaiting = dSerialPort->rxBufferSize + dSerialPort->rxBufferHead - dSerialPort->rxBufferTail;
            }

            if(bytesWaiting > 0){
                firstFuzeDataReceived = true;
            }
        }

        if(aux4Val > AUX_EDGE_VALUE && 
           aux2Val < AUX_EDGE_VALUE && 
           aux3Val < AUX_EDGE_VALUE && 
           aux5Val < AUX_EDGE_VALUE &&
           firstFuzeDataReceived){
            dInitialSafety = true;
            setFuzeData(0);
        }
        else{
            if(aux4Val < AUX_EDGE_VALUE){
                setFuzeData(3); // Güvenliği aç
            }
            else if(aux3Val > AUX_EDGE_VALUE){
                setFuzeData(4); // Şarjı kapat
            }
            else if(aux5Val > AUX_EDGE_VALUE_MIN){
                setFuzeData(5); // Patlatmaya basma
            }
            else if(aux2Val > AUX_EDGE_VALUE){
                setFuzeData(6); // Kontrole basma
            }
            else if(!firstFuzeDataReceived){
                setFuzeData(7); // Anahtarı Aç
            }
        }
    }

    return;
}

void initializationTask(void){
    if(!dInitializationCompleted){

        serialPortUsage_t* usage = findSerialPortUsageByIdentifier(FUZE_PORT_IDENTIFIER);

        if(usage != NULL){
            dSerialPort = usage->serialPort;

            dInitializationCompleted = true;
        }
    }

    // Wait a remote controller connect to drone
    waitRcData();

    checkSafety();

    return;
}

// Read new rc data to local variables
void readRcData(void){
    AUX2Value = rcData[AUX2];
    AUX3Value = rcData[AUX3];
    AUX4Value = rcData[AUX4];
    AUX5Value = rcData[AUX5];
}

// Examine readed rc values and assign them as high and low and set edge values if necessary
void parseRcData(void){
    if(AUX2Value > AUX_EDGE_VALUE){
        ControlHigh = true;
    }
    else{
        ControlHigh = false;
    }

    if(AUX5Value > AUX_EDGE_VALUE){
        ExplosionHigh = true;
    }
    else{
        ExplosionHigh = false;
    }

    if(AUX4Value > AUX_EDGE_VALUE){
        SafetyHigh = true;
    }
    else{
        SafetyHigh = false;
    }

    // HIGH
    if(AUX3Value > AUX_EDGE_VALUE){
        if(dLastAux3State != 3){
            CHARGE_HIGH_REQUESTED = true;
            SHOCK_SENSOR_ACTIVE_REQUESTED = true;
        }

        dLastAux3State = 3;
    }
    // MID
    else if(AUX3Value > AUX_EDGE_VALUE_MIN){
        if(dLastAux3State != 2){
            CHARGE_HIGH_REQUESTED = true;
            SHOCK_SENSOR_ACTIVE_REQUESTED = false;
        }

        dLastAux3State = 2;
    }
    // LOW
    else{
        if(dLastAux3State != 1){
            CHARGE_HIGH_REQUESTED = false;
            SHOCK_SENSOR_ACTIVE_REQUESTED = true;
        }

        dLastAux3State = 1;
    }
}

void sendFuzeData(void){
    if(SHOCK_SENSOR_ACTIVE_REQUESTED){
        if(!CHARGE_HIGH_REQUESTED && CHARGE_HIGH){
            // pass
        }
        else{
            if(LAST_SEND_SHOCK_SENSOR_MESSAGE != SSM_ENABLE){
                donmezogluSerialPrintC('O');

                LAST_SEND_SHOCK_SENSOR_MESSAGE = SSM_ENABLE;
            }
        }
    }
    else{
        if(LAST_SEND_SHOCK_SENSOR_MESSAGE != SSM_DISABLE){
            donmezogluSerialPrintC('C');

            LAST_SEND_SHOCK_SENSOR_MESSAGE = SSM_DISABLE;
        }
    }

    if(SafetyHigh){
        if(LAST_SEND_FUSE_MESSAGE != FM_SAFETY){
            donmezogluSerialPrintC('G');

            LAST_SEND_FUSE_MESSAGE = FM_SAFETY;
        }
    }
    else{
        if(!CHARGE_HIGH_REQUESTED && CHARGE_HIGH && LAST_SEND_FUSE_MESSAGE != FM_SAFETY){
            donmezogluSerialPrintC('G');

            LAST_SEND_FUSE_MESSAGE = FM_SAFETY;
        }
        else if(ControlHigh){
            if(LAST_SEND_FUSE_MESSAGE != FM_CONTROL){
                donmezogluSerialPrintC('K');

                LAST_SEND_FUSE_MESSAGE = FM_CONTROL;
            }
        }
        else if(ExplosionHigh && fuseConnected){
            if(CHARGE_DISPLAYING){
                if(LAST_SEND_FUSE_MESSAGE != FM_EXPLOSION){
                    donmezogluSerialPrintC('P');

                    LAST_SEND_FUSE_MESSAGE = FM_EXPLOSION;
                }
            }
        }
        else if(CHARGE_HIGH_REQUESTED && fuseConnected){
            if(!SHOCK_SENSOR_ACTIVE_REQUESTED && SHOCK_SENSOR_ACTIVE){
                // pass
            }
            else{
                if(LAST_SEND_FUSE_MESSAGE != FM_CHARGE){
                    donmezogluSerialPrintC('S');    

                    LAST_SEND_FUSE_MESSAGE = FM_CHARGE;
                }
            }
        }
        else{
            LAST_SEND_FUSE_MESSAGE = FM_NONE;
        }
    }
}

void serialDataReceivedF(void){
    // Zaten fünye kontrol edilmiş ve zaten bağlı olduğu biliniyorken tekrar status değiştirme yoksa E-E gösteremem
    if(!fuseConnectedMessageReceived || !fuseConnected){
        fuseConnectedMessageReceived = true;
        fuseConnected = true;
        setFuzeData(1);
    }
}

void serialDataReceivedH(void){
    fuseConnectedMessageReceived = true;
    fuseConnected = false;
    setFuzeData(0);
}

void serialDataReceivedE(void){
    setFuzeData(2);
    CHARGE_HIGH = true;
}

void serialDataReceivedW(void){
    CHARGE_HIGH = false;

    if(fuseConnected){
        setFuzeData(1);
    }
    else{
        setFuzeData(0);
    }
}

void serialDataReceivedN(void){
    CHARGE_HIGH = false;

    if(fuseConnected){
        setFuzeData(1);
    }
    else{
        setFuzeData(0);
    }
}

void serialDataReceivedZ(void){
    TAPA_STATUS = 0;
    SHOCK_SENSOR_ACTIVE = false;
}

void serialDataReceivedC(void){
    TAPA_STATUS = 1;
    SHOCK_SENSOR_ACTIVE = false;
}

void serialDataReceivedL(void){
    TAPA_STATUS = 2;
    SHOCK_SENSOR_ACTIVE = true;
}

void serialDataReceivedT(void){
    TAPA_STATUS = 3;
    SHOCK_SENSOR_ACTIVE = true;
}

void awaitFuzeData(void){
    static uint32_t bytesWaiting = 0;

    if (dSerialPort->rxBufferHead >= dSerialPort->rxBufferTail) {
        bytesWaiting = dSerialPort->rxBufferHead - dSerialPort->rxBufferTail;
    } else {
        bytesWaiting = dSerialPort->rxBufferSize + dSerialPort->rxBufferHead - dSerialPort->rxBufferTail;
    }

    if(bytesWaiting > 0){
        for(uint32_t i = 1; i <= bytesWaiting; ++i){
            static uint8_t dataReaded;
            dataReaded = serialRead(dSerialPort);

            switch (dataReaded)
            {
            case 'F':
            case 'f':
                serialDataReceivedF();
                break;
            case 'H':
            case 'h':
                serialDataReceivedH();
                break;
            case 'W':
            case 'w':
                serialDataReceivedW();
                break;
            case 'N':
            case 'n':
                serialDataReceivedN();
                break;
            case 'E':
            case 'e':
                serialDataReceivedE();
                break;
            case 'Z':
            case 'z':
                serialDataReceivedZ();
                break;
            case 'C':
            case 'c':
                serialDataReceivedC();
                break;
            case 'L':
            case 'l':
                serialDataReceivedL();
                break;
            case 'T':
            case 't':
                serialDataReceivedT();
                break;
            default:
                break;
            }
        }
    }
}

/*
void chargeTimeControl(timeUs_t currentTimeUs){
    static timeUs_t chargeStartingTimePoint = 0;

    if(lastSendMessage == LSM_CHARGE){
        if(lastSendMessagePrev != LSM_CHARGE){
            chargeStartingTimePoint = currentTimeUs;
        }

        // After one seconds of charging display as ready
        if(currentTimeUs - chargeStartingTimePoint > 1000000){
            FUZE_STATUS = 2;
        }
    }
    else{
        chargeStartingTimePoint = currentTimeUs;
    }
}
*/

#define UNUSED(x) (void)(x)

void periodicTask(timeUs_t currentTimeUs){
    UNUSED(currentTimeUs);

    if(!dInitializationCompleted || !dRcConnection || !dInitialSafety){
        return;
    }

    readRcData();

    parseRcData();

    sendFuzeData();

    awaitFuzeData();

    return;
}

void donmezogluUpdate(timeUs_t currentTimeUs){
    UNUSED(currentTimeUs);

    // Complete initialization if it is not completed yet
    initializationTask();

    // Perform periodic task after initializations
    periodicTask(currentTimeUs);
}

void donmezogluSerialPrintS(const char* str){
    if(dSerialPort != NULL){
        // Iterate through each character in the string until '\0' (null terminator) is encountered
        for (int i = 0; str[i] != '\0'; i++) {
            // Call serialWrite for each character in the string
            donmezogluSerialPrintC((uint8_t)str[i]);
        }
    }
}

void donmezogluSerialPrintC(uint8_t ch){
    if(dSerialPort != NULL){
        serialWrite(dSerialPort, ch);
    }
}