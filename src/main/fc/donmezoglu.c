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

static serialPort_t* dSerialPort = NULL;
static bool dInitializationCompleted = false;
static timeUs_t dTimePoint = 0;

void donmezogluUpdate(timeUs_t currentTimeUs){
    // Complete initialization if it is not completed yet
    if(!dInitializationCompleted){
 
        serialPortUsage_t* usage = findSerialPortUsageByIdentifier(SERIAL_PORT_UART5);

        if(usage != NULL){
            dSerialPort = usage->serialPort;
        }

        dInitializationCompleted = true;
    }

    if(currentTimeUs - dTimePoint > 1000000){
        if(dSerialPort != NULL){
            donmezogluSerialPrintS("DENEME");
        }

        dTimePoint = currentTimeUs;
    }
}

void donmezogluSerialPrintS(const char* str){
    // Iterate through each character in the string until '\0' (null terminator) is encountered
    for (int i = 0; str[i] != '\0'; i++) {
        // Call serialWrite for each character in the string
        donmezogluSerialPrintC((uint8_t)str[i]);
    }
}

void donmezogluSerialPrintC(uint8_t ch){
    serialWrite(dSerialPort, ch);
}