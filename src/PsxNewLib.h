/*******************************************************************************
 * This file is part of PsxNewLib.                                             *
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
 ******************************************************************************/
/**
 * \file PsxNewLib.h
 * \author SukkoPera <software@sukkology.net>
 * \date 27 Jan 2020
 * \brief Playstation controller interface library for Arduino
 * 
 * Please refer to the GitHub page and wiki for any information:
 * https://github.com/SukkoPera/PsxNewLib
 */

#ifndef PSXNEWLIB_H_
#define PSXNEWLIB_H_

/** \brief Command timeout (ms)
 * 
 * Commands are sent to the controller repeatedly, until they succeed or time
 * out. This is the length of that timeout.
 * 
 * \sa COMMAND_RETRY_INTERVAL
 */
const unsigned long COMMAND_TIMEOUT = 250;

/** \brief Command Retry Interval (ms)
 * 
 * When sending a command to the controller, if it does not succeed, it is
 * retried after this amount of time.
 */
const unsigned long COMMAND_RETRY_INTERVAL = 10;

/** \brief Mode switch delay (ms)
 * 
 * After a command has been issued successfully to the controller, this amount
 * of time is waited to allow it to complete any internal procedures required to
 * execute the command.
 * 
 * \todo This is probably unnecessary.
 */
const unsigned long MODE_SWITCH_DELAY = 500;


/** \brief Type that is used to represent a single button in most places
 */
enum PsxButton {
	PSB_NONE       = 0x0000,
	PSB_SELECT     = 0x0001,
	PSB_L3         = 0x0002,
	PSB_R3         = 0x0004,
	PSB_START      = 0x0008,
	PSB_PAD_UP     = 0x0010,
	PSB_PAD_RIGHT  = 0x0020,
	PSB_PAD_DOWN   = 0x0040,
	PSB_PAD_LEFT   = 0x0080,
	PSB_L2         = 0x0100,
	PSB_R2         = 0x0200,
	PSB_L1         = 0x0400,
	PSB_R1         = 0x0800,
	PSB_TRIANGLE   = 0x1000,
	PSB_CIRCLE     = 0x2000,
	PSB_CROSS      = 0x4000,
	PSB_SQUARE     = 0x8000
};

/** \brief Type that is used to represent a single button when retrieving
 *         analog pressure data
 *
 * \sa getAnalogButton()
 */
enum PsxAnalogButton {
	PSAB_PAD_RIGHT  = 0,
	PSAB_PAD_LEFT   = 1,
	PSAB_PAD_UP     = 2,
	PSAB_PAD_DOWN   = 3,
	PSAB_TRIANGLE   = 4,
	PSAB_CIRCLE     = 5,
	PSAB_CROSS      = 6,
	PSAB_SQUARE     = 7,
	PSAB_L1         = 8,
	PSAB_R1         = 9,
	PSAB_L2         = 10,
	PSAB_R2         = 11
};

/** \brief Number of digital buttons
 *
 * Includes \a everything, i.e.: 4 directions, Square, Cross, Circle, Triangle,
 * L1/2/3, R1/2/3, Select and Start.
 *
 * This is the number of entries in #PsxButton.
 */
const byte PSX_BUTTONS_NO = 16;

/** \brief Type that is used to report button presses
 */
typedef uint16_t PsxButtons;

/** \brief Size of buffer holding analog button data
 *
 * This is the size of the array returned by getAnalogButtonData().
 */
const byte PSX_ANALOG_BTN_DATA_SIZE = 12;

//! \name Controller Commands
//! @{
/** \brief Enter Configuration Mode
 * 
 * Command used to enter the controller configuration (also known as \a escape)
 * mode
 */
static const byte enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
static const byte exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
/* These shorter versions of enter_ and exit_config are accepted by all
 * controllers I've tested, even in analog mode, EXCEPT SCPH-1200, so let's use
 * the longer ones
 */
//~ static byte enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
//~ static byte exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x00};

/** \brief Read Controller Type
 * 
 * Command used to read the controller type.
 * 
 * This does not seem to be 100% reliable, or at least we don't know how to tell
 * all the various controllers apart.
 */
static const byte type_read[] = {0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
static const byte set_mode[] = {0x01, 0x44, 0x00, /* enabled */ 0x01, /* locked */ 0x03, 0x00, 0x00, 0x00, 0x00};
static const byte set_pressures[] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
//~ static byte enable_rumble[] = {0x01, 0x4D, 0x00, 0x00, 0x01};

/** \brief Poll all buttons
 * 
 * Command used to read the status of all buttons.
 */
static const byte poll[] = {0x01, 0x42, 0x00, 0xFF, 0xFF};
//! @}

/** \brief Controller Type
 *
 * This is somehow derived from the reply to the #type_read command. It is NOT
 * much trustworthy, so it might be removed in the future.
 *
 * \sa getControllerType
 */
enum PsxControllerType {
	PSCTRL_UNKNOWN = 0,			//!< No idea
	PSCTRL_DUALSHOCK,			//!< DualShock or compatible
	PSCTRL_DSWIRELESS,			//!< Sony DualShock Wireless
	PSCTRL_GUITHERO,			//!< Guitar Hero controller
};

/** \brief Number of different controller types recognized
 *
 * This is the number of entries in #PsxControllerType.
 */
const byte PSCTRL_MAX = static_cast<byte> (PSCTRL_GUITHERO) + 1;


/** \brief Controller Protocol
 *
 * Identifies the protocol the controller uses to report axes positions and
 * button presses. It's quite more reliable than #PsxControllerType, so use this
 * if you must.
 *
 * \sa getProtocol
 */
enum PsxControllerProtocol {
	PSPROTO_UNKNOWN = 0,		//!< No idea
	PSPROTO_DIGITAL,			//!< Original controller (SCPH-1010) protocol (8 digital buttons + START + SELECT)
	PSPROTO_DUALSHOCK,			//!< DualShock (has analog axes)
	PSPROTO_DUALSHOCK2,			//!< DualShock 2 (has analog axes and buttons)
	PSPROTO_FLIGHTSTICK,		//!< Green-mode (like DualShock but missing SELECT, L3 and R3)
	PSPROTO_NEGCON,				//!< Namco neGcon (has 1 analog X axis and analog Square, Circle and L1 buttons)
	PSPROTO_JOGCON				//!< Namco Jogcon (Wheel is mapped to analog X axis, half a rotation in each direction)
};

/** \brief Number of different protocols supported
 *
 * This is the number of entries in #PsxControllerProtocol.
 */
const byte PSPROTO_MAX = static_cast<byte> (PSPROTO_JOGCON) + 1;

/** \brief Analog sticks minimum value
 * 
 * Minimum value reported by analog sticks. This usually means that the stick is
 * fully either at the top or left position. Note that some sticks might not get
 * fully down to this value.
 *
 * \sa ANALOG_MAX_VALUE
 * \sa ANALOG_IDLE_VALUE
 */
const byte ANALOG_MIN_VALUE = 0U;

/** \brief Analog sticks maximum value
 * 
 * Maximum value reported by analog sticks. This usually means that the stick is
 * fully either at the bottom or right position. Note that some sticks might not
 * get fully up to this value.
 *
 * \sa ANALOGI_MAX_VALUE
 * \sa ANALOG_IDLE_VALUE
 */
const byte ANALOG_MAX_VALUE = 255U;

/** \brief Analog sticks idle value
 * 
 * Value reported when an analog stick is in the (ideal) center position. Note
 * that old and worn-out sticks might not self-center perfectly when released,
 * so you should never rely on this precise value to be reported.
 *
 * Also note that the up/down and left/right ranges are off by one, since values
 * 0-127 represent up/left and 129-255 mean down/right. The former interval
 * contains 128 different values, while the latter only 127. Sometimes you will
 * need to take this in consideration.
 */
const byte ANALOG_IDLE_VALUE = 128U;

/** \brief neGcon I/II-button press threshold
 *
 * The neGcon does not report digital button press data for its analog buttons,
 * so we have to make it up. The Square, Cross digital buttons will be
 * reported as pressed when the analog value of the II and I buttons
 * (respectively), goes over this threshold.
 *
 * \sa NEGCON_L_BUTTON_THRESHOLD
 */
const byte NEGCON_I_II_BUTTON_THRESHOLD = 128U;

/** \brief neGcon L-button press threshold
 *
 * The neGcon does not report digital button press data for its analog buttons,
 * so we have to make it up. The L1 digital button will be reported as pressed
 * when the analog value of the L buttons goes over this threshold.
 *
 * This value has been tuned so that the L button gets digitally triggered at
 * about the same point as the non-analog R button. This is done "empirically"
 * and might need tuning on a different controller than the one I actually have.
 * 
 * \sa NEGCON_I_II_BUTTON_THRESHOLD
 */
const byte NEGCON_L_BUTTON_THRESHOLD = 240U;

#include "PsxDriver.h"


/** \brief PSX Controller Interface
 * 
 * This is the base class implementing interactions with PSX controllers. It is
 * partially abstract, so it is not supposed to be instantiated directly.
 */
class PsxController {
protected:
	PsxDriver *driver;
	
	/** \brief Previous (Digital) Button Status
	 * 
	 * The individual bits can be identified through #PsxButton.
	 */
	PsxButtons previousButtonWord;

	/** \brief (Digital) Button Status
	 * 
	 * The individual bits can be identified through #PsxButton.
	 */
	PsxButtons buttonWord;

	/** \brief Controller Protocol
	 *
	 * The protocol controller data was interpreted with at the last call to
	 * read()
	 *
	 * \sa getProtocol
	 */
	PsxControllerProtocol protocol;

	//! \name Analog Stick Data
	//! @{
	byte lx;		//!< Horizontal axis of left stick [0-255, L to R]
	byte ly;		//!< Vertical axis of left stick [0-255, U to D]
	byte rx;		//!< Horizontal axis of right stick [0-255, L to R]
	byte ry;		//!< Vertical axis of right stick [0-255, U to D]
	
	boolean analogSticksValid;	//!< True if the above were valid at the last call to read()
	//! @}
	
	/** \brief Analog Button Data
	 * 
	 * \todo What's the meaning of every individual byte?
	 */
	byte analogButtonData[PSX_ANALOG_BTN_DATA_SIZE];

	/** \brief Analog Button Data Validity
	 * 
	 * True if the #analogButtonData were valid in last call to read()
	 */
	boolean analogButtonDataValid;

	// Green Mode controllers
	inline boolean isFlightstickReply (const byte *status) {
		return (status[1] & 0xF0) == 0x50;
	}

	inline boolean isDualShockReply (const byte *status) {
		return (status[1] & 0xF0) == 0x70;
	}

	inline boolean isDualShock2Reply (const byte *status) {
		return status[1] == 0x79;
	}

	inline boolean isDigitalReply (const byte *status) {
		return (status[1] & 0xF0) == 0x40;
	}

	inline boolean isConfigReply (const byte *status) {
		return (status[1] & 0xF0) == 0xF0;
	}

	inline boolean isNegconReply (const byte *status) {
		return status[1] == 0x23;
	}

	inline boolean isJogconReply (const byte *status) {
		return (status[1] & 0xF0) == 0xE0;
	}

public:
	/** \brief Initialize library
	 * 
	 * This function shall be called before any others, it will initialize the
	 * communication and return if a supported controller was found. It shall
	 * also be called to reinitialize the communication whenever the controller
	 * is unplugged.
	 * 
	 * Derived classes can override this function if they need to perform
	 * additional initializations, but shall call it on return.
	 * 
	 * \return true if a supported controller was found, false otherwise
	 */
	virtual boolean begin (PsxDriver& drv) {
		driver = &drv;
		
		// Start with all analog axes at midway position
		lx = ANALOG_IDLE_VALUE;		
		ly = ANALOG_IDLE_VALUE;
		rx = ANALOG_IDLE_VALUE;
		ry = ANALOG_IDLE_VALUE;

		analogSticksValid = false;
		memset (analogButtonData, 0, sizeof (analogButtonData));

		protocol = PSPROTO_UNKNOWN;

		// Some disposable readings to let the controller know we are here
		for (byte i = 0; i < 5; ++i) {
			read ();
			delay (1);
		}

		return read ();
	}

	//! \name Configuration Mode Functions
	//! @{
	
	/** \brief Enter Configuration Mode
	 * 
	 * Some controllers can be configured in several aspects. For instance,
	 * DualShock controllers can return analog stick data. This function puts
	 * the controller in configuration mode.
	 * 
	 * Note that <i>Configuration Mode</i> is sometimes called <i>Escape Mode</i>.
	 * 
	 * \return true if Configuration Mode was entered successfully
	 */
	boolean enterConfigMode () {
		boolean ret = false;

		unsigned long start = millis ();
		do {
			driver -> attention ();
			byte *in = driver -> autoShift (enter_config, 4);
			driver -> noAttention ();

			ret = in != NULL && isConfigReply (in);

			if (!ret) {
				delay (COMMAND_RETRY_INTERVAL);
			}
		} while (!ret && millis () - start <= COMMAND_TIMEOUT);
		delay (MODE_SWITCH_DELAY);

		return ret;
	}

	/** \brief Enable (or disable) analog sticks
	 * 
	 * This function enables or disables the analog sticks that were introduced
	 * with DualShock controllers. When they are enabled, the getLeftAnalog()
	 * and getRightAnalog() functions can be used to retrieve their positions.
	 * Also, button presses for L3 and R3 will be available through the
	 * buttonPressed() and similar functions.
	 * 
	 * When analog sticks are enabled, the \a ANALOG led will light up (in red)
	 * on the controller.
	 * 
	 * Note that on some third-party controllers, when analog sticks are
	 * disabled the analog levers will "emulate" the D-Pad and possibly the
	 * []/^/O/X buttons. This does not happen on official Sony controllers.
	 * 
	 * This function will only work if when the controller is in Configuration
	 * Mode.
	 * 
	 * \param[in] enabled true to enable, false to disable
	 * \param[in] locked If true, the \a ANALOG button on the controller will be
	 *                   disabled and the user will not be able to turn off the
	 *                   analog sticks.
	 * \return true if the command was ackowledged by the controller. Note that
	 *         this does not fully guarantee that the analog sticks were enabled
	 *         as this can only be checked after Configuration Mode is exited.
	 */
	boolean enableAnalogSticks (bool enabled = true, bool locked = false) {
		boolean ret = false;
		byte out[sizeof (set_mode)];

		memcpy (out, set_mode, sizeof (set_mode));
		out[3] = enabled ? 0x01 : 0x00;
		out[4] = locked ? 0x03 : 0x00;

		unsigned long start = millis ();
		byte cnt = 0;
		do {
			driver -> attention ();
			byte *in = driver -> autoShift (out, 5);
			driver -> noAttention ();

			/* We can't know if we have successfully enabled analog mode until
			 * we get out of config mode, so let's just be happy if we get a few
			 * consecutive valid replies
			 */
			if (in != nullptr) {
				++cnt;
			}
			ret = cnt >= 3;

			if (!ret) {
				delay (COMMAND_RETRY_INTERVAL);
			}
		} while (!ret && millis () - start <= COMMAND_TIMEOUT);
		delay (MODE_SWITCH_DELAY);

		return ret;
	}

	/** \brief Enable (or disable) analog buttons
	 * 
	 * This function enables or disables the analog buttons that were introduced
	 * with DualShock 2 controllers. When they are enabled, the
	 * getAnalogButton() functions can be used to retrieve how deep/strongly
	 * they are pressed. This applies to the D-Pad buttons, []/^/O/X, L1/2 and
	 * R1/2
	 * 
	 * This function will only work if when the controller is in Configuration
	 * Mode.
	 * 
	 * \param[in] enabled true to enable, false to disable
	 * \return true if the command was ackowledged by the controller. Note that
	 *         this does not fully guarantee that the analog sticks were enabled
	 *         as this can only be checked after Configuration Mode is exited.
	 */
	boolean enableAnalogButtons (bool enabled = true) {
		boolean ret = false;
		byte out[sizeof (set_mode)];

		memcpy (out, set_pressures, sizeof (set_pressures));
		if (!enabled) {
			out[3] = 0x00;
			out[4] = 0x00;
			out[5] = 0x00;
		}

		unsigned long start = millis ();
		byte cnt = 0;
		do {
			driver -> attention ();
			byte *in = driver -> autoShift (out, sizeof (set_pressures));
			driver -> noAttention ();

			/* We can't know if we have successfully enabled analog mode until
			 * we get out of config mode, so let's just be happy if we get a few
			 * consecutive valid replies
			 */
			if (in != nullptr) {
				++cnt;
			}
			ret = cnt >= 3;

			if (!ret) {
				delay (COMMAND_RETRY_INTERVAL);
			}
		} while (!ret && millis () - start <= COMMAND_TIMEOUT);
		delay (MODE_SWITCH_DELAY);

		return ret;
	}

	/** \brief Retrieve the controller type
	 * 
	 * This function retrieves the controller type. It is not 100% reliable, so
	 * do not rely on it for anything other than a vague indication (for
	 * instance, the DualShock SCPH-1200 controller gets reported as the Guitar
	 * Hero controller...).
	 * 
	 * This function will only work if when the controller is in Configuration
	 * Mode.
	 * 
	 * \return The (tentative) controller type
	 */
	PsxControllerType getControllerType () {
		PsxControllerType ret = PSCTRL_UNKNOWN;

		driver -> attention ();
		byte *in = driver -> autoShift (type_read, 3);
		driver -> noAttention ();

		if (in != nullptr) {
			const byte& controllerType = in[3];
			if (controllerType == 0x03) {
				ret = PSCTRL_DUALSHOCK;
			//~ } else if (controllerType == 0x01 && in[1] == 0x42) {
				//~ return 4;		// ???
			}  else if (controllerType == 0x01 && in[1] != 0x42) {
				ret = PSCTRL_GUITHERO;
			} else if (controllerType == 0x0C) {
				ret = PSCTRL_DSWIRELESS;
			}
		}

		return ret;
	}

	boolean exitConfigMode () {
		boolean ret = false;

		unsigned long start = millis ();
		do {
			driver -> attention ();
			//~ shiftInOut (poll, in, sizeof (poll));
			//~ shiftInOut (exit_config, in, sizeof (exit_config));
			byte *in = driver -> autoShift (exit_config, 4);
			driver -> noAttention ();

			ret = in != nullptr && !isConfigReply (in);

			if (!ret) {
				delay (COMMAND_RETRY_INTERVAL);
			}
		} while (!ret && millis () - start <= COMMAND_TIMEOUT);
		delay (MODE_SWITCH_DELAY);

		return ret;
	}

	//! @}		// Configuration Mode Functions
	
	//! \name Polling Functions
	//! @{

	/** \brief Retrieve the controller protocol
	 * 
	 * This function retrieves the protocol that was used to interpret
	 * controller data at the last call to read().
	 * 
	 * \return The controller protocol
	 */
	PsxControllerProtocol getProtocol () const {
		return protocol;
	}

	/** \brief Poll the controller
	 * 
	 * This function polls the controller for button and stick data. It self-
	 * adapts to all the supported controller types and populates internal
	 * variables with the retrieved information, which can be later accessed
	 * through the inspection functions.
	 * 
	 * This function must be called quite often in order to keep the controller
	 * alive. Most controllers have some kind of watchdog that will reset them
	 * if they don't get polled at least every so often (like a couple dozen
	 * times per seconds).
	 * 
	 * If this function fails repeatedly, it can safely be assumed that the
	 * controller has been disconnected (or that it is not supported if it
	 * failed right from the beginning).
	 * 
	 * \return true if the read was successful, false otherwise
	 */
	boolean read () {
		boolean ret = false;

		analogSticksValid = false;
		analogButtonDataValid = false;

		driver -> attention ();
		byte *in = driver -> autoShift (poll, 3);
		driver -> noAttention ();

		if (in != NULL) {
			if (isConfigReply (in)) {
				// We're stuck in config mode, try to get out
				exitConfigMode ();
			} else {
				// We surely have buttons
				previousButtonWord = buttonWord;
				buttonWord = ((PsxButtons) in[4] << 8) | in[3];

				// See if we have anything more to read
				if (isDualShock2Reply (in)) {
					protocol = PSPROTO_DUALSHOCK2;
				} else if (isDualShockReply (in)) {
					protocol = PSPROTO_DUALSHOCK;
				} else if (isFlightstickReply (in)) {
					protocol = PSPROTO_FLIGHTSTICK;
				} else if (isNegconReply (in)) {
					protocol = PSPROTO_NEGCON;
				} else if (isJogconReply (in)) {
					protocol = PSPROTO_JOGCON;
				} else {
					protocol = PSPROTO_DIGITAL;
				}

				switch (protocol) {
					case PSPROTO_DUALSHOCK2:
						// We also have analog button data
						analogButtonDataValid = true;
						for (int i = 0; i < PSX_ANALOG_BTN_DATA_SIZE; ++i) {
							analogButtonData[i] = in[i + 9];
						}
						/* Now fall through to DualShock case, the next line
						 * avoids GCC warning
						 */
						/* FALLTHRU */
					case PSPROTO_DUALSHOCK:
					case PSPROTO_FLIGHTSTICK:
						// We have analog stick data
						analogSticksValid = true;
						rx = in[5];
						ry = in[6];
						lx = in[7];
						ly = in[8];
						break;
					case PSPROTO_NEGCON:
						// Map the twist axis to X axis of left analog
						analogSticksValid = true;
						lx = in[5];

						// Map analog button data to their reasonable counterparts
						analogButtonDataValid = true;
						analogButtonData[PSAB_CROSS] = in[6];
						analogButtonData[PSAB_SQUARE] = in[7];
						analogButtonData[PSAB_L1] = in[8];

						// Make up "missing" digital data
						if (analogButtonData[PSAB_SQUARE] >= NEGCON_I_II_BUTTON_THRESHOLD) {
							buttonWord &= ~PSB_SQUARE;
						}
						if (analogButtonData[PSAB_CROSS] >= NEGCON_I_II_BUTTON_THRESHOLD) {
							buttonWord &= ~PSB_CROSS;
						}
						if (analogButtonData[PSAB_L1] >= NEGCON_L_BUTTON_THRESHOLD) {
							buttonWord &= ~PSB_L1;
						}
						break;
					case PSPROTO_JOGCON:
						/* Map the wheel X axis of left analog, half a rotation
						 * per direction: byte 5 has the wheel position, it is
						 * 0 at startup, then we have 0xFF down to 0x80 for
						 * left/CCW, and 0x01 up to 0x80 for right/CW
						 *
						 * byte 6 is the number of full CW rotations
						 * byte 7 is 0 if wheel is still, 1 if it is rotating CW
						 *        and 2 if rotation CCW
						 * byte 8 seems to stay at 0
						 *
						 * We'll want to cap the movement halfway in each
						 * direction, for ease of use/implementation.
						 */
						analogSticksValid = true;
						if (in[6] < 0x80) {
							// CW up to half
							lx = in[5] < 0x80 ? in[5] : (0x80 - 1);
						} else {
							// CCW down to half
							lx = in[5] > 0x80 ? in[5] : (0x80 + 1);
						}

						// Bring to the usual 0-255 range
						lx += 0x80;
						break;
					default:
						// We are already done
						break;
				}
				
				ret = true;
			}
		}

		return ret;
	}

	/** \brief Check if any button has changed state
	 * 
	 * \return true if any button has changed state with regard to the previous
	 *         call to read(), false otherwise
	 */
	boolean buttonsChanged () const {
		return ((previousButtonWord ^ buttonWord) > 0);
	}

	/** \brief Check if a button has changed state
	 * 
	 * \return true if \a button has changed state with regard to the previous
	 *         call to read(), false otherwise
	 */
	boolean buttonChanged (const PsxButtons button) const {
		return (((previousButtonWord ^ buttonWord) & button) > 0);
	}

	/** \brief Check if a button is currently pressed
	 * 
	 * \param[in] button The button to be checked
	 * \return true if \a button was pressed in last call to read(), false
	 *         otherwise
	 */
	boolean buttonPressed (const PsxButton button) const {
		return buttonPressed (~buttonWord, button);
	}

	/** \brief Check if a button is pressed in a Button Word
	 * 
	 * \param[in] buttons The button word to check in
	 * \param[in] button The button to be checked
	 * \return true if \a button is pressed in \a buttons, false otherwise
	 */
	boolean buttonPressed (const PsxButtons buttons, const PsxButton button) const {
		return ((buttons & static_cast<const PsxButtons> (button)) > 0);
	}

	/** \brief Check if a button has just been pressed
	 * 
	 * \param[in] button The button to be checked
	 * \return true if \a button was not pressed in the previous call to read()
	 *         and is now, false otherwise
	 */
	boolean buttonJustPressed (const PsxButton button) const {
		return (buttonChanged (button) & buttonPressed (button));
	}

	/** \brief Check if a button has just been released
	 * 
	 * \param[in] button The button to be checked
	 * \return true if \a button was pressed in the previous call to read() and
	 *         is not now, false otherwise
	 */
	boolean buttonJustReleased (const PsxButton button) const {
		return (buttonChanged (button) & ((~previousButtonWord & button) > 0));
	}

	/** \brief Check if NO button is pressed in a Button Word
	 * 
	 * \param[in] buttons The button word to check in
	 * \return true if all buttons in \a buttons are released, false otherwise
	 */
	boolean noButtonPressed (const PsxButtons buttons) const {
		return buttons == PSB_NONE;
	}

	/** \brief Check if NO button is currently pressed
	 * 
	 * \return true if all buttons were released in the last call to read(),
	 *         false otherwise
	 */
	boolean noButtonPressed (void) const {
		return buttonWord == ~PSB_NONE;
	}
	
	/** \brief Retrieve the <em>Button Word</em>
	 * 
	 * The button word contains the status of all digital buttons and can be
	 * retrieved so that it can be inspected later.
	 * 
	 * \sa buttonPressed
	 * \sa noButtonPressed
	 * 
	 * \return the Button Word
	 */
	PsxButtons getButtonWord () const {
		return ~buttonWord;
	}

	/** \brief Retrieve button pressure depth/strength
	 * 
	 * This function will return how deeply/strongly a button is pressed. It
	 * will only work on DualShock 2 controllers after enabling this feature
	 * with enableAnalogButtons().
	 * 
	 * Note that button pressure depth/strength is only available for the D-Pad
	 * buttons, []/^/O/X, L1/2 and R1/2.
	 *
	 * \param[in] button the button the retrieve the pressure depth/strength of
	 * \return the pressure depth/strength [0-255, Fully released to fully
	 *         pressed]
	 */
	byte getAnalogButton (const PsxAnalogButton button) const {
		byte ret = 0;
		
		if (analogButtonDataValid) {
			ret = analogButtonData[button];
		//~ } else if (buttonPressed (button)) {		// FIXME
			//~ // No analog data, assume fully pressed or fully released
			//~ ret = 0xFF;
		}

		return ret;
	}

	/** \brief Retrieve all analog button data
	 */
	const byte* getAnalogButtonData () const {
		return analogButtonDataValid ? analogButtonData : NULL;
	}

	/** \brief Retrieve position of the \a left analog stick
	 * 
	 * This function will return the absolute position of the left analog stick.
	 * 
	 * Note that not all controllers have analog sticks, in which case this
	 * function will return false.
	 * 
	 * \param[in] x A variable where the horizontal position will be stored
	 *              [0-255, L to R]
	 * \param[in] y A variable where the vertical position will be stored
	 *              [0-255, U to D]
	 * \return true if the returned position is valid, false otherwise
	 */
	boolean getLeftAnalog (byte& x, byte& y) const {
		x = lx;
		y = ly;

		return analogSticksValid;
	}

	/** \brief Retrieve position of the \a right analog stick
	 * 
	 * This function will return the absolute position of the right analog
	 * stick.
	 * 
	 * Note that not all controllers have analog sticks, in which case this
	 * function will return false.
	 * 
	 * \param[in] x A variable where the horizontal position will be stored
	 *              [0-255, L to R]
	 * \param[in] y A variable where the vertical position will be stored
	 *              [0-255, U to D]
	 * \return true if the returned position is valid, false otherwise
	 */
	boolean getRightAnalog (byte& x, byte& y) {
		x = rx;
		y = ry;

		return analogSticksValid;
	}
	
	//! @}		// Polling Functions
};

#endif
