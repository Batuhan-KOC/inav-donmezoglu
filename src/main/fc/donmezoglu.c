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
#include "fc/rc_controls.h"
#include "rx/rx.h"

static serialPort_t* dSerialPort = NULL;
static bool dInitializationCompleted = false;
static bool dRcConnection = false;

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

static bool ControlPositiveEdge = false;
static bool ChargePositiveEdge = false;
static bool SafetyPositiveEdge = false;
static bool ExplosionPositiveEdge = false;

static bool ExpectingControlMessage = false;

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
    }

    return;
}

void initializationTask(void){
    if(!dInitializationCompleted){

        serialPortUsage_t* usage = findSerialPortUsageByIdentifier(SERIAL_PORT_UART5);

        if(usage != NULL){
            dSerialPort = usage->serialPort;
        }

        dInitializationCompleted = true;
    }

    // Wait a remote controller connect to drone
    waitRcData();

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
void readFuzeData(void){
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
        if(!ExplosionHigh){
            ExplosionPositiveEdge = true;
        }

        ExplosionHigh = true;
    }
    else{
        ExplosionHigh = false;
    }
}

void sendFuzeData(void){
    if(SafetyHigh){
        if(lastSendMessage != LSM_SAFETY){
            donmezogluSerialPrintC('G');

            lastSendMessage = LSM_SAFETY;
        }
    }
    else{
        if(ControlHigh){
            if(lastSendMessage != LSM_CONTROL){
                donmezogluSerialPrintC('K');

                lastSendMessage = LSM_CONTROL;

                ExpectingControlMessage = true;
            }
        }
        else if(ExplosionHigh){
            if(lastSendMessage != LSM_EXPLOSION){
                donmezogluSerialPrintC('P');

                lastSendMessage = LSM_EXPLOSION;
            }
        }
        else if(ChargeHigh){
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

void periodicTask(void){
    readRcData();

    readFuzeData();

    sendFuzeData();

    lastSendMessagePrev = lastSendMessage;
}

void donmezogluUpdate(timeUs_t currentTimeUs){
    UNUSED(currentTimeUs);

    // Complete initialization if it is not completed yet
    initializationTask();

    // Perform periodic task after initializations
    periodicTask();
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