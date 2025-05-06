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
    UNUSED(currentTimeUs);

    // Complete initialization if it is not completed yet
    if(!dInitializationCompleted){
 
        serialPortUsage_t* usage = findSerialPortUsageByIdentifier(SERIAL_PORT_UART5);

        if(usage != NULL){
            dSerialPort = usage->serialPort;
        }
        else{
            dSerialPort = openSerialPortSafe(
            SERIAL_PORT_UART5,
            FUNCTION_NONE,
            NULL,
            NULL,
            BAUD_9600,
            MODE_RXTX,
            SERIAL_NOT_INVERTED | SERIAL_BIDIR
            );
        }

        dInitializationCompleted = true;
    }

    if(currentTimeUs - dTimePoint > 1000000){
        if(dSerialPort != NULL){
            serialWrite(dSerialPort, 'R');
        }

        dTimePoint = currentTimeUs;
    }
}