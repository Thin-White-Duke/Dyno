/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com
This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _CONFIG_h
#define _CONFIG_h

#define DEBUG                // if defined additional output is given
#define DEBUGSERIAL Serial
#define SERIALIO Serial1     // serial port for VESC-UART connection

// HX711 load cell
#define HX711_DOUT A1        // DOUT assignment pin #A1
#define HX711_PD_SCK A0      // PD_SCK assignment pin #A0
#define LEN_LEV 200          // length of the lever arm in mm

// Prior to any measurement the regression line for the calibration have to be determined
// by several measurements. The function has the generic form m = SCALE_M * measure + SCALE_B.
// For the cheap 10 kg measurement cell the equation reads m [gram] = -0.00454 * x - 1664.235837
#define WEIGHT_MAX 10000     // maximum weight in gram which must not be exceeded
#define SCALE_M_CALIB -0.00454
#define SCALE_B_CALIB -1664.235837

// LCD interface pins with arduino pin numbers connected
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

// dyno switches and controls
#define STARTSTOP_PIN 2      // digital pin (internal pullup) for start/stop switches
#define PWM_PIN 3            // interrup pin for PWM modulation of constant current source
#define CONTROL_PIN A8       // poti control
#define CURR_TORQUE_MAX 682  // maximum int current allowed for torque brake e.g. 400mA/600mA*1023
#define M_NULL 0.777         // zero torque [Ncm] of the magtrol brake without field current

// motor parameters
#define POLE_PAIR 7

#endif
