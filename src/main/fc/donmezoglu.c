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

#include "io/serial.h"
#include "drivers/serial_uart.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "io/beeper.h"

int FUZE_STATUS = 0;

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

static serialPort_t* dSerialPort = NULL;
static bool dInitializationCompleted = false;
static bool dRcConnection = false;

static bool dInitialSafety = false;

static int dLastAux5State = -1;

static float AUX_EDGE_VALUE_MIN = 1300;
static float AUX_MID_VALUE = 1500;
static float AUX_EDGE_VALUE = 1750;

static float AUX2Value = 0; // KONTROL
static float AUX3Value = 0; // ŞARJ
static float AUX4Value = 0; // GÜVENLİK
static float AUX5Value = 0; // PATLATMA

static bool ControlHigh = false;
static bool ChargeHigh = false;
static bool SafetyHigh = false;
static bool ExplosionHigh = false;

static bool SendTapaEnable = false;
static bool SendTapaDisable = false;

static bool ControlPositiveEdge = false;
static bool ChargePositiveEdge = false;
static bool SafetyPositiveEdge = false;

static bool fuseConnectedMessageReceived = false;
static bool fuseConnected = false;

enum LastSendMessage{
    LSM_NONE,
    LSM_CONTROL,
    LSM_CHARGE,
    LSM_SAFETY,
    LSM_EXPLOSION
};

static enum LastSendMessage lastSendMessagePrev = LSM_NONE;
static enum LastSendMessage lastSendMessage = LSM_NONE;

static float oldAUX2 = 1500;
static float oldAUX3 = 1500;
static float oldAUX4 = 1500;
static float oldAUX5 = 1500;

void waitRcData(void){
    if(dInitializationCompleted && !dRcConnection){
        oldAUX2 = rcData[AUX2];
        oldAUX3 = rcData[AUX3];
        oldAUX4 = rcData[AUX4];
        oldAUX5 = rcData[AUX5];

        // At least one of them should be not equal to default mid value
        if(oldAUX2 != AUX_MID_VALUE || 
           oldAUX3 != AUX_MID_VALUE || 
           oldAUX4 != AUX_MID_VALUE || 
           oldAUX5 != AUX_MID_VALUE){
            dRcConnection = true;
        }
        else{
            FUZE_STATUS = 8; // Connect Rc Controller
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
            FUZE_STATUS = 0;
        }
        else{
            if(aux4Val < AUX_EDGE_VALUE){
                FUZE_STATUS = 3; // Güvenliği aç
            }
            else if(aux3Val > AUX_EDGE_VALUE){
                FUZE_STATUS = 4; // Şarjı kapat
            }
            else if(aux5Val > AUX_EDGE_VALUE_MIN){
                FUZE_STATUS = 5; // Patlatmaya basma
            }
            else if(aux2Val > AUX_EDGE_VALUE){
                FUZE_STATUS = 6; // Kontrole basma
            }
            else if(!firstFuzeDataReceived){
                FUZE_STATUS = 7; // Anahtarı Aç
            }
        }
    }

    return;
}

void initializationTask(void){
    if(!dInitializationCompleted){

        serialPortUsage_t* usage = findSerialPortUsageByIdentifier(SERIAL_PORT_UART5);

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
        if(!ControlHigh){
            ControlPositiveEdge = true;
        }

        ControlHigh = true;
    }
    else{
        ControlHigh = false;
    }

    if(AUX3Value > AUX_EDGE_VALUE){
        if(!ChargeHigh){
            ChargePositiveEdge = true;
        }

        ChargeHigh = true;
    }
    else{
        ChargeHigh = false;
    }

    if(AUX4Value > AUX_EDGE_VALUE){
        if(!SafetyHigh){
            SafetyPositiveEdge = true;
        }

        SafetyHigh = true;
    }
    else{
        SafetyHigh = false;
    }

    if(AUX5Value > AUX_EDGE_VALUE){
        if(dLastAux5State != 3){
            SendTapaEnable = false;
            SendTapaDisable = false;
            ExplosionHigh = true;
        }

        dLastAux5State = 3;
    }
    else if(AUX5Value > AUX_EDGE_VALUE_MIN){
        if(dLastAux5State != 2){
            SendTapaEnable = true;
            SendTapaDisable = false;
            ExplosionHigh = false;
        }

        dLastAux5State = 2;
    }
    else{
        if(dLastAux5State != 1){
            SendTapaEnable = false;
            SendTapaDisable = true;
            ExplosionHigh = false;
        }

        dLastAux5State = 1;
    }
}

void sendFuzeData(void){
    if(SendTapaEnable){
        donmezogluSerialPrintC('O');
        SendTapaEnable = false;
    }

    if(SendTapaDisable){
        donmezogluSerialPrintC('C');
        SendTapaDisable = false;
    }

    if(SafetyHigh){
        if(lastSendMessage != LSM_SAFETY){
            donmezogluSerialPrintC('G');

            lastSendMessage = LSM_SAFETY;

            if(fuseConnected){
                FUZE_STATUS = 1;
            }
            else{
                FUZE_STATUS = 0;
            }
        }
    }
    else{
        if(ControlHigh){
            if(lastSendMessage != LSM_CONTROL){
                dSerialPort->rxBufferHead = 0;
                dSerialPort->rxBufferTail = 0;

                donmezogluSerialPrintC('K');

                lastSendMessage = LSM_CONTROL;
            }
        }
        else if(ExplosionHigh && fuseConnected){
            if(lastSendMessage != LSM_EXPLOSION){
                donmezogluSerialPrintC('P');

                lastSendMessage = LSM_EXPLOSION;
            }
        }
        else if(ChargeHigh && fuseConnected){
            if(lastSendMessage != LSM_CHARGE){
                donmezogluSerialPrintC('S');

                lastSendMessage = LSM_CHARGE;
            }
        }
        else{
            lastSendMessage = LSM_NONE;
        }
    }
}

void serialDataReceivedF(){
    // Zaten fünye kontrol edilmiş ve zaten bağlı olduğu biliniyorken tekrar status değiştirme yoksa E-E gösteremem
    if(!fuseConnectedMessageReceived || !fuseConnected){
        fuseConnectedMessageReceived = true;
        fuseConnected = true;
        FUZE_STATUS = 1;
    }
}

void serialDataReceivedH(){
    fuseConnectedMessageReceived = true;
    fuseConnected = false;
    FUZE_STATUS = 0;
}

void serialDataReceivedE(){
    FUZE_STATUS = 2;
}

void serialDataReceivedZ(){
    TAPA_STATUS = 0;
}

void serialDataReceivedC(){
    TAPA_STATUS = 1;
}

void serialDataReceivedL(){
    TAPA_STATUS = 2;
}

void serialDataReceivedT(){
    TAPA_STATUS = 3;
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

void periodicTask(timeUs_t currentTimeUs){
    if(!dInitializationCompleted || !dRcConnection || !dInitialSafety){
        return;
    }

    readRcData();

    parseRcData();

    sendFuzeData();

    awaitFuzeData();

    //chargeTimeControl(currentTimeUs);

    lastSendMessagePrev = lastSendMessage;

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