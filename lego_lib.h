#pragma once

/**
 * @author Jan Mrázek
 * This small piece of code includes the ev3 library and ensures
 * that if your app fail (uncaught exception or bad pointer
 * dereference), all the motors are stopped.
 */

#include "ev3dev.h"
#include <iostream>
#include <csignal>

/**
 * Safe replacement of the main function. All exceptions are caught,
 * all signals are processed and the motors are always stopped in the
 * end of program.
 */
int run();

/**
 * Signal handler which disables all outputs of the lego
 */
void stop_motors(int);
