#pragma once

#include <stdbool.h>
#include <time.h>

#include "common/time.h"

/*
  * Periodic task with low priority and 33 Hertz frequency.
  *
  * Note : Higher priorities cause error on runtime. Remain low priority.
 */
void donmezogluUpdate(timeUs_t currentTimeUs);