#pragma once
#include "KincoNamespace.h"

#define XSTR(x) STR(x)
#define STR(x) #x

#define SIDEREAL_RATE_MTR_RPM 6.25
#define SR_MULT 10

#define UPDATE_RATE_HZ 30

const bool TERMINAL_DRIVE_A = true;
const bool TERMINAL_DRIVE_B = true;


#define AZ_DRIVER_ID_A 1
#define AZ_DRIVER_ID_B 2
#define EL_DRIVER_ID_A 3
#define EL_DRIVER_ID_B 4

#define RX_BUFF_SIZE 128

#define CONSOLE_DOWNSAMPLE 10