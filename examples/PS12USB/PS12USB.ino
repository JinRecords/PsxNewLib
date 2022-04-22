/*******************************************************************************
 * This file is a modified version of PSX2USB                                  *
 * This modification was made for Dualshock 1                                  *
 * Modified by JinRecords 2022                                                 *
 *                                                                             *
 * Copyright (C) 2019-2020 by SukkoPera <software@sukkology.net>               *
 *                                                                             *
 * PsxNewLib is free software: you can redistribute it and/or                  *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * PsxNewLib is distributed in the hope that it will be useful,                *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with PsxNewLib. If not, see http://www.gnu.org/licenses.              *
 *******************************************************************************
 *
 * This sketch shows how the library can be used to turn a PSX controller into
 * an USB one, using an Arduino Leonardo or Micro and the excellent
 * ArduinoJoystickLibrary.
 *
 * For details on ArduinoJoystickLibrary, see
 * https://github.com/MHeironimus/ArduinoJoystickLibrary.
 */

#include <PsxControllerBitBang.h>
#include <Joystick.h>

/* We must use the bit-banging interface, as SPI pins are only available on the
   ICSP header on the Leonardo.

   Note that we use pins 9-12 so that 13 can be used with the built-in LED.
*/
const byte PIN_PS2_ATT = 9;
const byte PIN_PS2_CMD = 10;
const byte PIN_PS2_DAT = 11;
const byte PIN_PS2_CLK = 12;

const unsigned long POLLING_INTERVAL = 1000U / 50U;

// Send debug messages to serial port
//~ #define ENABLE_SERIAL_DEBUG

PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;

Joystick_ usbStick (
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS,
  14,			// buttonCount
  1,			// hatSwitchCount (0-2)
  true,		// includeXAxis
  true,		// includeYAxis
  true,		// includeZAxis
  true,		// includeRxAxis
  true,		// includeRyAxis
  true,		// includeRzAxis
  false,		// includeRudder
  false,		// includeThrottle
  false,		// includeAccelerator
  false,		// includeBrake
  false		// includeSteering
);


#ifdef ENABLE_SERIAL_DEBUG
#define dstart(spd) do {Serial.begin (spd); while (!Serial) {digitalWrite (LED_BUILTIN, (millis () / 500) % 2);}} while (0);
#define debug(...) Serial.print (__VA_ARGS__)
#define debugln(...) Serial.println (__VA_ARGS__)
#else
#define dstart(...)
#define debug(...)
#define debugln(...)
#endif

boolean haveController = false;


#define	toDegrees(rad) (rad * 180.0 / PI)

#define deadify(var, thres) (abs (var) > thres ? (var) : 0)


/** \brief Dead zone for analog sticks

   If the analog stick moves less than this value from the center position, it
   is considered still.

   \sa ANALOG_IDLE_VALUE
*/
const byte ANALOG_DEAD_ZONE = 50U;


void setup () {
  // Lit the builtin led whenever buttons are pressed
  pinMode (LED_BUILTIN, OUTPUT);

  // Init Joystick library
  usbStick.begin (false);		// We'll call sendState() manually to minimize lag

  // This way we can output the same range of values we get from the PSX controller
  usbStick.setXAxisRange (ANALOG_MIN_VALUE, ANALOG_MAX_VALUE);
  usbStick.setYAxisRange (ANALOG_MIN_VALUE, ANALOG_MAX_VALUE);
  usbStick.setRxAxisRange (ANALOG_MIN_VALUE, ANALOG_MAX_VALUE);
  usbStick.setRyAxisRange (ANALOG_MIN_VALUE, ANALOG_MAX_VALUE);

  dstart (115200);

  debugln (F("Ready!"));
}

void loop () {
  static unsigned long last = 0;

  if (millis () - last >= POLLING_INTERVAL) {
    last = millis ();

    if (!haveController) {
      if (psx.begin ()) {
        debugln (F("Controller found!"));
        if (!psx.enterConfigMode ()) {
          debugln (F("Cannot enter config mode"));
        } else {
          // Try to enable analog sticks
          if (!psx.enableAnalogSticks ()) {
            debugln (F("Cannot enable analog sticks"));
          }

          if (!psx.exitConfigMode ()) {
            debugln (F("Cannot exit config mode"));
          }
        }

        haveController = true;
      }
    } else {
      if (!psx.read ()) {
        debugln (F("Controller lost :("));
        haveController = false;
      } else {
        byte x, y;

        /* Flash led with buttons, I like this but it introduces a bit of
           lag, so let's keep it disabled by default
        */
        //~ digitalWrite (LED_BUILTIN, !!psx.getButtonWord ());

        // Read was successful, so let's make up data for Joystick

        // Buttons first!
        usbStick.setButton (0, psx.buttonPressed (PSB_SQUARE));
        usbStick.setButton (1, psx.buttonPressed (PSB_CROSS));
        usbStick.setButton (2, psx.buttonPressed (PSB_CIRCLE));
        usbStick.setButton (3, psx.buttonPressed (PSB_TRIANGLE));
        usbStick.setButton (4, psx.buttonPressed (PSB_L1));
        usbStick.setButton (5, psx.buttonPressed (PSB_R1));
        usbStick.setButton (6, psx.buttonPressed (PSB_L2));
        usbStick.setButton (7, psx.buttonPressed (PSB_R2));
        usbStick.setButton (13, psx.buttonPressed (PSB_SELECT));
        usbStick.setButton (9, psx.buttonPressed (PSB_START));
        usbStick.setButton (10, psx.buttonPressed (PSB_L3));		// Only available on DualShock and later controllers
        usbStick.setButton (11, psx.buttonPressed (PSB_R3));		// Ditto

        // D-Pad makes up the X/Y axes
        if (psx.buttonPressed (PSB_PAD_UP)) {
          usbStick.setHatSwitch (0, 0);
        }

        if (psx.buttonPressed (PSB_PAD_DOWN)) {
          usbStick.setHatSwitch (0, 180);
        }

        if (psx.buttonPressed (PSB_PAD_LEFT)) {
          usbStick.setHatSwitch (0, 270);
        }

        if (psx.buttonPressed (PSB_PAD_RIGHT)) {
          usbStick.setHatSwitch (0, 90);
        }


        // Left analog gets mapped to the X/Y rotation axes
        if (psx.getLeftAnalog (x, y)) {
          usbStick.setXAxis (x + 10);
          if ((y <= 95) || (y >= 111)) {
          }
          usbStick.setYAxis (y + 16);
        } else {
        }
        
        if (psx.getRightAnalog (x, y)) {
          usbStick.setRzAxis (map( y, -255, 255, -1023, 1023) + 110);
          usbStick.setZAxis (map( x, -255, 255, -1023, 1023) + 5);
        }
        // All done, send data for real!
        usbStick.sendState ();
      }
    }
  }
}
