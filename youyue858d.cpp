/*
 * V1.48.3S (based on RaiHei V1.47 - For Technet_858D Board v14 )
 *
 * 2015/16	- Robert Spitzenpfeil (madworm)
 * 2015		- Moritz Augsburger
 * V1.48 (based on RaiHei V1.47 )
 * 2016    - RaiHei
 * 2018    - Gabs'e
 * 2021	   - Vic Severov
 * 
 * License: GNU GPL v2
 *
 */

/* Technet_858D_V14
 *
 * PB0: Active Beeper
 * PB1: SW1 (Top Button)
 * PB2: SW2 (Bottom Button)
 * PB3: 7-seg digit 0 - Common Anode 
 * PB4: 7-seg digit 1 - Common Anode 
 * PB5: 7-seg digit 2 - Common Anode 
 * PB6: TRIAC control  (!!! DANGEROUS !!!)
 *
 * PC0: TIP122.base --> FAN Control
 * PC1: FAN purge MOD
 * PC3: ADC <-- Termocouple (A3 in Arduino IDE)
 * PC4: Reed Switch in Wand
 * PC5: FAN speed MOD (A5  in Arduino IDE)
 * #21: AREF <--- Analog Reference Voltage for ADC
 *
 * PD0: 7-seg bottom 			- D
 * PD1: 7-seg bottom left 		- E
 * PD2: 7-seg top 				- A
 * PD3: 7-seg middle 			- G
 * PD4: 7-seg bottom right 		- C
 * PD5: 7-seg dot 				- DP
 * PD6: 7-seg top right 		- B
 * PD7: 7-seg top left 			- F
 *
 */

#include "Arduino.h"
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>
#include "youyue858d.h"

uint8_t fb[3] = { 0xFF, 0xFF, 0xFF };	// dig0, dig1, dig2
framebuffer_t framebuffer = { 0x00, 0x00, 0x00, 0, 0, 0, 0 };

CPARAM p_gain = { 0, 999, P_GAIN_DEFAULT, P_GAIN_DEFAULT, 2, 3 };	// min, max, default, value, eep_addr_high, eep_addr_low
CPARAM i_gain = { 0, 999, I_GAIN_DEFAULT, I_GAIN_DEFAULT, 4, 5 };
CPARAM d_gain = { 0, 999, D_GAIN_DEFAULT, D_GAIN_DEFAULT, 6, 7 };
CPARAM i_thresh = { 0, 100, I_THRESH_DEFAULT, I_THRESH_DEFAULT, 8, 9 };
CPARAM temp_offset_corr = { -100, 100, TEMP_OFFSET_CORR_DEFAULT, TEMP_OFFSET_CORR_DEFAULT, 10, 11 };
CPARAM temp_setpoint = { 50, 500, TEMP_SETPOINT_DEFAULT, TEMP_SETPOINT_DEFAULT, 12, 13 };
CPARAM temp_averages = { 100, 999, TEMP_AVERAGES_DEFAULT, TEMP_AVERAGES_DEFAULT, 14, 15 };
CPARAM slp_timeout = { 0, 30, SLP_TIMEOUT_DEFAULT, SLP_TIMEOUT_DEFAULT, 16, 17 };
CPARAM fan_only = { 0, 1, 0, 0, 26, 27 };
CPARAM display_adc_raw = { 0, 1, 0, 0, 28, 29 };
CPARAM fan_timeout = { 0, 60, FAN_TIMEOUT_DEFAULT, FAN_TIMEOUT_DEFAULT, 30, 31 };
CPARAM sound_on = { 0, 1, SOUND_ON_DEFAULT, SOUND_ON_DEFAULT, 32, 33 };

#ifdef CURRENT_SENSE_MOD
CPARAM fan_current_min = { 0, 999, FAN_CURRENT_MIN_DEFAULT, FAN_CURRENT_MIN_DEFAULT, 22, 23 };
CPARAM fan_current_max = { 0, 999, FAN_CURRENT_MAX_DEFAULT, FAN_CURRENT_MAX_DEFAULT, 24, 25 };
#else
CPARAM fan_speed_min = { 120, 180, FAN_SPEED_MIN_DEFAULT, FAN_SPEED_MIN_DEFAULT, 18, 19 };
CPARAM fan_speed_max = { 300, 400, FAN_SPEED_MAX_DEFAULT, FAN_SPEED_MAX_DEFAULT, 20, 21 };
#endif

volatile uint8_t key_state;	// debounced and inverted key state: bit = 1: key pressed
volatile uint8_t key_press;	// key press detect
volatile uint8_t key_rpt;	// key long press and repeat

volatile uint8_t display_blink;

bool cradlebeep = true;
bool timeoutbeep = true;
bool hasheated = false;
bool firstbeep = true;
int tcounter = 0; // timer for fan off delay
int scounter = 0; // timer to debounce beep when putting gun in cradle


//// Main ///////////////////////////////////////////////////////////////////

int main(void)
{

	init();			// make sure the Arduino-specific stuff is up and running (timers... see 'wiring.c')
	setup_858D();	

#ifdef DISPLAY_MCUSR
	HEATER_OFF;
	FAN_ON;
	display_number(_mcusr);
	MCUSR = 0;
	//
	// ATmega168 MCUSR (MSB to LSB): x-x-x-x-WDRF-BORF-EXTRF-PORF
	//
	delay(1000);
#endif

//

#ifdef USE_WATCHDOG

	_mcusr = MCUSR;
	MCUSR = 0;	

	if (!(_mcusr & _BV(EXTRF))) { 		//External Reset?
		if (_mcusr & _BV(WDRF)) {		// there was a watchdog reset - should never ever happen
			HEATER_OFF;
			FAN_ON;

		#ifdef FAN_PURGE_MOD
			FAN_MAX_ON;
		#endif		
			watchdog_off();
			while (1) {
				beep(1);
				display_string("RST");
				delay(300);
				clear_display();
				delay(300);
			}
		}
	}
#endif

#ifdef USE_WATCHDOG
	test_F_CPU_with_watchdog();
#endif

	fan_test();
	beep(2);

#ifdef USE_WATCHDOG
	watchdog_on();
#endif	
	
#ifdef DEBUG
	Serial.begin(2400);
	Serial.println("\nRESET");
#endif

	// Main Loop
	while (1) {
#ifdef DEBUG
		int32_t start_time = micros();
#endif
		static int16_t temp_inst = 0;
		static int32_t temp_accu = 0;
		static int16_t temp_average = 0;
		static int16_t temp_average_previous = 0;

		static int32_t adc_raw_accu = 0;
		static int16_t adc_raw_average = 0;

		static int32_t button_input_time = 0;

		static int16_t heater_ctr = 0;
		static int16_t heater_duty_cycle = 0;
		static int16_t error = 0;
		static int32_t error_accu = 0;
		static int16_t velocity = 0;
		static float PID_drive = 0;

		static uint8_t temp_setpoint_saved = 1;
		static int32_t temp_setpoint_saved_time = 0;

		static uint32_t heater_start_time = 0;

		uint16_t adc_raw = analogRead(A3); 	// need raw value later, store it here and avoid 2nd ADC read

		// Orig
		//temp_inst = adc_raw + temp_offset_corr.value;	
		
		// For samsung MCU
		//temp_inst =(adc_raw/1.9) + temp_offset_corr.value; 

		temp_inst = adc_raw/1.9 + temp_offset_corr.value;  

		// For MK1841D3 MCU
		//temp_inst =(adc_raw/2.4) + temp_offset_corr.value;  

		if (temp_inst < 0) {
			temp_inst = 0;
		}
		// pid loop / heater handling
		if (fan_only.value == 1 || REEDSW_CLOSED) {
			HEATER_OFF;
			heater_start_time = millis();
			clear_dot();
		} else if (REEDSW_OPEN && (temp_setpoint.value >= temp_setpoint.value_min) && (temp_average < MAX_TEMP_ERR) && ((millis() - heater_start_time) < ((uint32_t) (slp_timeout.value) * 60 * 1000))) {

			FAN_ON;
			
			error = temp_setpoint.value - temp_average;
			velocity = temp_average_previous - temp_average;

			if (abs(error) < i_thresh.value) {
				// if close enough to target temperature use PID control
				error_accu += error;
			} else {
				// otherwise only use PD control (avoids issues with error_accu growing too large
				error_accu = 0;
			}

			PID_drive =
			    error * (p_gain.value / P_GAIN_SCALING) + error_accu * (i_gain.value / I_GAIN_SCALING) +
			    velocity * (d_gain.value / D_GAIN_SCALING);

			heater_duty_cycle = (int16_t) (PID_drive);

			if (heater_duty_cycle > HEATER_DUTY_CYCLE_MAX) {
				heater_duty_cycle = HEATER_DUTY_CYCLE_MAX;
			}

			if (heater_duty_cycle < 0) {
				heater_duty_cycle = 0;
			}

			if (heater_ctr < heater_duty_cycle) {
				set_dot();
				HEATER_ON;
			} else {
				HEATER_OFF;
				clear_dot();
			}

			heater_ctr++;
			if (heater_ctr == PWM_CYCLES) {
				heater_ctr = 0;
			}
		} else {
			HEATER_OFF;
			clear_dot();
		}

		static uint16_t temp_avg_ctr = 0;

		temp_accu += temp_inst;
		temp_avg_ctr++;

		adc_raw_accu += adc_raw;  // re-uses the counter temp_avg_ctr

		if (temp_avg_ctr == (uint16_t) (temp_averages.value)) {
			temp_average_previous = temp_average;
			temp_average = temp_accu / temp_averages.value;
			temp_accu = 0;
			temp_avg_ctr = 0;

			adc_raw_average = adc_raw_accu / temp_averages.value;
			adc_raw_accu = 0;
		}

		// fan/cradle handling

		if (cradlebeep == false && REEDSW_CLOSED) {
			if (scounter < 300) {
				scounter++;
				delay(1);
			} else {
				scounter = 0;
				beep(1);
				cradlebeep = true;
			}
		}

		if (cradlebeep == true && REEDSW_OPEN) {
			cradlebeep = false;
			firstbeep = true;
			timeoutbeep = false;
		}
		
		if ((millis() - heater_start_time) > ((uint32_t) (slp_timeout.value) * 60 * 1000) && timeoutbeep == false) {
			beep(2);
			timeoutbeep = true;
		}

		if (REEDSW_OPEN && temp_average > 100) {
			hasheated = true;
		}

		if (REEDSW_CLOSED && fan_only.value == 0 && (temp_average > FAN_ON_TEMP)) {                  // In cradle cool down heating mode cooling
			FAN_ON;
			FAN_MAX_ON;
		} else if (REEDSW_CLOSED && fan_only.value == 1 && (temp_average > FAN_OFF_TEMP_FANONLY)) {   // In cradle cool down FAN mode cooling
			FAN_ON;
			FAN_MAX_ON;
		} else if (REEDSW_CLOSED && fan_only.value == 1 && (temp_average <= FAN_OFF_TEMP_FANONLY)) { // In cradle cool down FAN mode cold
			FAN_OFF;
			FAN_MAX_OFF;
		} else if (REEDSW_CLOSED && fan_only.value == 0 && (temp_average <= FAN_ON_TEMP)) {        // In cradle cool down heating mode cold
			if (tcounter < (fan_timeout.value*1000) && hasheated == true){
				tcounter++;
				delay(1);
			} else {
				tcounter = 0;
				hasheated = false;
				FAN_OFF;
				FAN_MAX_OFF;
			}
		
		} else {             //  Outside cradle
			FAN_ON;
			FAN_MAX_OFF;
		}

		// menu key handling
		if (get_key_short(1 << KEY_UP)) {
			button_input_time = millis();
			if (temp_setpoint.value < temp_setpoint.value_max) {
				temp_setpoint.value++;
		        firstbeep = true;

			}
			temp_setpoint_saved = 0;
		} else if (get_key_short(1 << KEY_DOWN)) {
			button_input_time = millis();
			if (temp_setpoint.value > temp_setpoint.value_min) {
				temp_setpoint.value--;
		        firstbeep = true;
			}
			temp_setpoint_saved = 0;
		} else if (get_key_long_r(1 << KEY_UP) || get_key_rpt_l(1 << KEY_UP)) {
			button_input_time = millis();
			if (temp_setpoint.value < (temp_setpoint.value_max - 10)) {
				temp_setpoint.value += 10;
		        firstbeep = true;				
			} else {
				temp_setpoint.value = temp_setpoint.value_max;
			}
			temp_setpoint_saved = 0;

		} else if (get_key_long_r(1 << KEY_DOWN) || get_key_rpt_l(1 << KEY_DOWN)) {
			button_input_time = millis();

			if (temp_setpoint.value > (temp_setpoint.value_min + 10)) {
				temp_setpoint.value -= 10;
		        firstbeep = true;				
			} else {
				temp_setpoint.value = temp_setpoint.value_min;
			}

			temp_setpoint_saved = 0;
		} else if (get_key_common_l(1 << KEY_UP | 1 << KEY_DOWN)) {
			HEATER_OFF;	// security reasons, delay below!
			clear_dot();

#ifdef USE_WATCHDOG
		watchdog_off();
#endif

			delay(uint16_t(20.48 * (REPEAT_START - 3) + 1));
			if (get_key_long_r(1 << KEY_UP | 1 << KEY_DOWN)) {
				change_config_parameter(&temp_offset_corr, "TOF");
				change_config_parameter(&temp_averages, "AVG");
				change_config_parameter(&slp_timeout, "SLP");
				change_config_parameter(&fan_timeout, "FAN");
				change_config_parameter(&sound_on, "SND");
				change_config_parameter(&display_adc_raw, "ADC");
				change_config_parameter(&p_gain, "P");
				change_config_parameter(&i_gain, "I");
				change_config_parameter(&d_gain, "D");
				change_config_parameter(&i_thresh, "ITH");
#ifdef CURRENT_SENSE_MOD
				change_config_parameter(&fan_current_min, "FCL");
				change_config_parameter(&fan_current_max, "FCH");
#else
//				change_config_parameter(&fan_speed_min, "FSL");
//				change_config_parameter(&fan_speed_max, "FSH");
#endif
			} else {
				get_key_press(1 << KEY_UP | 1 << KEY_DOWN);	// clear inp state
				beep(1);
				fan_only.value ^= 0x01;
				temp_setpoint_saved = 0;
				if (fan_only.value == 0) {
					button_input_time = millis();	// show set temp after disabling fan only mode
				}
				display_blink = 0;	// make sure we start displaying "FAN" or set temp
			}
#ifdef USE_WATCHDOG
		watchdog_on();
#endif			
		}
		// security first!
		if (temp_average >= MAX_TEMP_ERR) {
			// something might have gone terribly wrong
			beep(3);
			HEATER_OFF;
			clear_dot();
			FAN_ON;
			FAN_MAX_ON;

#ifdef USE_WATCHDOG
			watchdog_off();
#endif
			while (1) {
				// stay here until the power is cycled
				// make sure the user notices the error by blinking "FAN"
				// and don't resume operation if the error goes away on its own
				//
				// possible reasons to be here:
				//
				// * wand is not connected (false temperature reading)
				// * thermo couple has failed
				// * true over-temperature condition
				//
				beep(3);
				display_string("*C");
				delay(1000);
				display_string("ERR");
				delay(1000);
				clear_display();
				delay(1000);
			}
		}
		// Display Output
		if ((millis() - button_input_time) < SHOW_SETPOINT_TIMEOUT) {
			if (display_blink < 5) {
				clear_display();
			} else {
				display_number(temp_setpoint.value);	// Show Setpoint Temperature
			}
		} else {
			if (temp_setpoint_saved == 0) {
				set_eeprom_saved_dot();
				eep_save(&temp_setpoint);
				eep_save(&fan_only);
				temp_setpoint_saved_time = millis();
				temp_setpoint_saved = 1;
			} else if (temp_average <= SAFE_TO_TOUCH_TEMP) {
				if (fan_only.value == 1) {
					display_string("FAN");
				} else {
					display_string("---");
				}
			} else if (fan_only.value == 1) {
				if (display_blink < 20) {
					display_string("FAN");
				} else {
					display_number(temp_average);
				}
			} else if (display_adc_raw.value == 1) {
				display_number(adc_raw_average);
			} else if (abs((int16_t) (temp_average) - (int16_t) (temp_setpoint.value)) < TEMP_REACHED_MARGIN) {
				if (firstbeep == true) {  				// beep one time when set temperature is reached
					beep(1);
        			firstbeep = false;
				}
				display_number(temp_setpoint.value);	// avoid showing insignificant fluctuations on the display (annoying)
			} else {
				display_number(temp_average);
			}
		}

		if ((millis() - temp_setpoint_saved_time) > 500) {
			clear_eeprom_saved_dot();
		}

		fb_update();

#if defined(WATCHDOG_TEST) && defined(USE_WATCHDOG)
		// watchdog test
		if (temp_average > 100) {
			delay(3000);
		}
#endif		

#ifdef USE_WATCHDOG
		wdt_reset();
#endif

#ifdef DEBUG
		int32_t stop_time = micros();
		Serial.println(stop_time - start_time);
#endif
	}

}

void setup_858D(void)
{
	HEATER_OFF;
	clear_dot();
	DDRB |= _BV(PB6);							// Triac Control (Input)

#ifdef FAN_PURGE_MOD
	DDRC |= _BV(PB7);							// Fan Max Speed Relay (Output)
#endif

	DDRB |= _BV(PB0);							// Beeper (Output)

	DDRB &= ~(_BV(PB1) | _BV(PB2));				// Buttons (Input)

	DDRC &= ~_BV(PC4);							// Reed Switch (Input)

	PORTC |= _BV(PC4);							// Reed Switch Pullup Resistor is On
	PORTB |= (_BV(PB1) | _BV(PB2));				// Buttons Pullup Resistor is On

	FAN_OFF;
	DDRC |= _BV(PC0);							// Fan control (Output)

#ifdef FAN_PURGE_MOD	
	DDRC |= _BV(PC1);							// FAN PURGE MOD (Output)
#endif

	DDRD |= 0xFF;								// Port D - 7-seg Indicator Control (Output segments)
	DDRB |= (_BV(PB3) | _BV(PB4) | _BV(PB5));	// Digits 1,2,3 7-seg Indicator (Output)

#ifndef CURRENT_SENSE_MOD
//	DDRC &= ~_BV(PC1);							
#endif

	setup_timer1_ctc();							// Display refresh timer

	analogReference(EXTERNAL);					// Analog Reference Voltage = 2.5V

	if (EEPROM.read(0) != 0x22) {
		// check if the firmware was just flashed and the EEPROM is therefore empty
		// assumption: full chip erase with ISP programmer (preserve eeprom fuse NOT set!)
		// if so, restore default parameter values & write a 'hint' to address 0
		restore_default_conf();
		EEPROM.write(0, 0x22);
	}
	
	// Press SW0 and SW1 simultaneously, and turn on device for reset all settings to default values
	if (SW0_PRESSED && SW1_PRESSED) {
		beep(4);
		restore_default_conf();
	// Press SW0 and turn on device for fan Speed Test
	} else if (SW0_PRESSED) {
		display_string("FAN");
		delay(1000);
		display_string("TST");
		delay(1000);
		FAN_ON;
		while (1) {
			uint16_t fan;
			delay(500);
#ifdef CURRENT_SENSE_MOD
			fan = analogRead(A5);
#else				//CURRENT_SENSE_MOD
			fan = analogRead(A5);			// Read Fan Speed (Arduino A5 = ATmega PC5)
#endif				//CURRENT_SENSE_MOD
			display_number(fan);
		}
	}

	eep_load(&p_gain);
	eep_load(&i_gain);
	eep_load(&d_gain);
	eep_load(&i_thresh);
	eep_load(&temp_offset_corr);
	eep_load(&temp_setpoint);
	eep_load(&temp_averages);
	eep_load(&slp_timeout);
	eep_load(&fan_only);
	eep_load(&display_adc_raw);
	eep_load(&fan_timeout);
	eep_load(&sound_on);
#ifdef CURRENT_SENSE_MOD
	eep_load(&fan_current_min);
	eep_load(&fan_current_max);
#else
	eep_load(&fan_speed_min);
	eep_load(&fan_speed_max);
#endif
}

void clear_display(void)
{
	framebuffer.digit[0] = 255;
	framebuffer.digit[1] = 255;
	framebuffer.digit[2] = 255;
	framebuffer.dot[0] = 0;
	framebuffer.dot[1] = 0;
	framebuffer.dot[2] = 0;
	framebuffer.changed = 1;
	fb_update();
}

void display_string(const char *string)
{
	framebuffer.digit[0] = 255;
	framebuffer.digit[1] = 255;
	framebuffer.digit[2] = 255;
	framebuffer.dot[0] = 0;
	framebuffer.dot[1] = 0;
	framebuffer.dot[2] = 0;

	uint8_t ctr;

	for (ctr = 0; ctr <= 2; ctr++) {
		// read the first 3 characters of the string
		if (string[ctr] == '\0') {
			break;
		} else {
			framebuffer.digit[ctr] = string[ctr];
		}
	}
	framebuffer.changed = 1;
	fb_update();
}

void change_config_parameter(CPARAM * param, const char *string)
{
	beep(1);
	display_string(string);
	delay(1000);		

	uint8_t loop = 1;

	while (loop == 1) {
		if (get_key_short(1 << KEY_UP)) {
			if (param->value < param->value_max) {
				param->value++;
			}
		} else if (get_key_short(1 << KEY_DOWN)) {
			if (param->value > param->value_min) {
				param->value--;
			}
		} else if (get_key_long_r(1 << KEY_UP) || get_key_rpt_l(1 << KEY_UP)) {
			if (param->value < param->value_max - 10) {
				param->value += 10;
			}
		} else if (get_key_long_r(1 << KEY_DOWN) || get_key_rpt_l(1 << KEY_DOWN)) {
			if (param->value > param->value_min + 10) {
				param->value -= 10;
			}
		} else if (get_key_common(1 << KEY_UP | 1 << KEY_DOWN)) {
			loop = 0;
		}

		display_number(param->value);
	}
	set_eeprom_saved_dot();
	eep_save(param);
	beep(2);
	delay(750);
	clear_eeprom_saved_dot();
}

void eep_save(CPARAM * param)
{
	// make sure NOT to save invalid parameter values
	if ((param->value >= param->value_min) && (param->value <= param->value_max)) {
		// nothing to do
	} else {
		// reset to sensible minimum
		param->value = param->value_default;
	}
	EEPROM.update(param->eep_addr_high, highByte(param->value));
	EEPROM.update(param->eep_addr_low, lowByte(param->value));
}

void eep_load(CPARAM * param)
{
	int16_t tmp = (EEPROM.read(param->eep_addr_high) << 8) | EEPROM.read(param->eep_addr_low);

	// make sure NOT to restore invalid parameter values
	if ((tmp >= param->value_min) && (tmp <= param->value_max)) {
		// the value was good, so we use it
		param->value = tmp;
	} else {
		// reset to sensible value
		param->value = param->value_default;
	}
}

void restore_default_conf(void)
{
	p_gain.value = p_gain.value_default;
	i_gain.value = i_gain.value_default;
	d_gain.value = d_gain.value_default;
	i_thresh.value = i_thresh.value_default;
	temp_offset_corr.value = temp_offset_corr.value_default;
	temp_setpoint.value = temp_setpoint.value_default;
	temp_averages.value = temp_averages.value_default;
	slp_timeout.value = slp_timeout.value_default;
	fan_only.value = 0;
	display_adc_raw.value = 0;
	fan_timeout.value = fan_timeout.value_default;
	sound_on.value = sound_on.value_default;
#ifdef CURRENT_SENSE_MOD
	fan_current_min.value = fan_current_min.value_default;
	fan_current_max.value = fan_current_max.value_default;
#else
	fan_speed_min.value = fan_speed_min.value_default;
	fan_speed_max.value = fan_speed_max.value_default;
#endif

	eep_save(&p_gain);
	eep_save(&i_gain);
	eep_save(&d_gain);
	eep_save(&i_thresh);
	eep_save(&temp_offset_corr);
	eep_save(&temp_setpoint);
	eep_save(&temp_averages);
	eep_save(&slp_timeout);
	eep_save(&fan_only);
	eep_save(&display_adc_raw);
	eep_save(&fan_timeout);
	eep_save(&sound_on);
	
#ifdef CURRENT_SENSE_MOD
	eep_save(&fan_current_min);
	eep_save(&fan_current_max);
#else
	eep_save(&fan_speed_min);
	eep_save(&fan_speed_max);
#endif
}

void set_dot(void)
{
	framebuffer.dot[2] = 1;
	framebuffer.changed = 1;
	fb_update();
}

void clear_dot(void)
{
	framebuffer.dot[2] = 0;
	framebuffer.changed = 1;
	fb_update();
}

void set_eeprom_saved_dot(void)
{
	framebuffer.dot[0] = 1;
	framebuffer.dot[1] = 1;
	framebuffer.changed = 1;
	fb_update();
}

void clear_eeprom_saved_dot(void)
{
	framebuffer.dot[0] = 0;
	framebuffer.dot[1] = 0;
	framebuffer.changed = 1;
	fb_update();
}

void display_number(int16_t number)
{
	if (number < 0) {
		framebuffer.dot[0] = 1;
		framebuffer.dot[1] = 1;
		framebuffer.dot[2] = 1;
		number = -number;
	} else {
		framebuffer.dot[0] = 0;
		framebuffer.dot[1] = 0;
	}
	
	framebuffer.digit[2] = (uint8_t) (number % 10);
	number /= 10;
	framebuffer.digit[1] = (uint8_t) (number % 10);
	number /= 10;
	framebuffer.digit[0] = (uint8_t) (number % 10);
	framebuffer.changed = 1;
	fb_update();
}

void display_char(uint8_t digit, uint8_t character, uint8_t dot)
{
	uint8_t portout = 0xFF;

	switch (character) {
	case 0:
		portout = (uint8_t) (~0xD7);	// activate segments for displaying a '0'
		break;
	case 1:
		portout = (uint8_t) (~0x50);	// '1'
		break;
	case 2:
		portout = (uint8_t) (~0x4F);	// '2'
		break;
	case 3:
		portout = (uint8_t) (~0x5D);	// '3'
		break;
	case 4:
		portout = (uint8_t) (~0xD8);	// '4'
		break;
	case 5:
		portout = (uint8_t) (~0x9D);	// '5'
		break;
	case 6:
		portout = (uint8_t) (~0x9F);	// '6'
		break;
	case 7:
		portout = (uint8_t) (~0x54);	// '7'
		break;
	case 8:
		portout = (uint8_t) (~0xDF);	// '8'
		break;
	case 9:
		portout = (uint8_t) (~0xDC);	// '9'
		break;
	case '-':
		portout = (uint8_t) (~0x08);	// '-'
		break;
	case '.':
		portout = (uint8_t) (~0x20);	// '.'
		break;
	case 'A':
		portout = (uint8_t) (~0xDE);	// 'A'
		break;
	case 'C':
		portout = (uint8_t) (~0x87);	// 'C'
		break;
	case 'D':
		portout = (uint8_t) (~0x5B);	// 'd'
		break;
	case 'E':
		portout = (uint8_t) (~0x8F);	// 'E'
		break;
	case 'F':
		portout = (uint8_t) (~0x8E);	// 'F'
		break;
	case 'G':
		portout = (uint8_t) (~0x9F);	// 'G'
		break;
	case 'H':
		portout = (uint8_t) (~0x9A);	// 'h'
		break;
	case 'I':
		//portout = (uint8_t) (~0x10);	// 'i'
		portout = (uint8_t) (~0x50);	// 'i'
		break;
	case 'L':
		portout = (uint8_t) (~0x83);	// 'L'
		break;
	case 'N':
		portout = (uint8_t) (~0xD6);	// 'N'
		break;
	case 'O':
		portout = (uint8_t) (~0x1B);	// 'o'
		break;
	case 'P':
		portout = (uint8_t) (~0xCE);	// 'P'
		break;
	case 'R':
		portout = (uint8_t) (~0x0A);	// 'r'
		break;
	case 'S':
		portout = (uint8_t) (~0x9D);	// 'S'
		break;
	case 'T':
		portout = (uint8_t) (~0x8B);	// 't'
		break;
	case 'U':
		portout = (uint8_t) (~0x13);	// 'u'          
		break;
	case 'V':
		portout = (uint8_t) (~0x13);	// 'v'
		break;
	case '*':
		portout = (uint8_t) (~0xCC);	// '°'
		break;
	case 255:
		portout = (uint8_t) (0xFF);	// All Segments OFF
		break;
	default:
		portout = (uint8_t) (~0x20);	// '.'
		break;
	}

	if (dot)
		portout &= (~0x20);	// '.'

	fb[digit] = portout;
}

void fan_test(void)
{
	HEATER_OFF;
	clear_dot();

	// if the wand is not in the cradle when powered up, go into a safe mode
  	// and display an error
	while (!REEDSW_CLOSED) {
		beep(3);
		display_string("CRA");
		delay(1000);
		display_string("DLE");
		delay(1000);
		clear_display();
		delay(1000);
	}

#ifdef CURRENT_SENSE_MOD
	uint16_t fan_current;
	FAN_ON;
	delay(3000);
	fan_current = analogRead(A5);

	if ((fan_current < (uint16_t) (fan_current_min.value)) || (fan_current > (uint16_t) (fan_current_max.value))) {
#else				//CURRENT_SENSE_MOD
	uint16_t fan_speed;
	FAN_ON;
	delay(3000);
	fan_speed = analogRead(A5);

			//display_number(fan_speed);
			delay(1000);

	// the fan is not working as it should
	if ((fan_speed < (uint16_t) (fan_speed_min.value)) || (fan_speed > (uint16_t) (fan_speed_max.value))) {
#endif				//CURRENT_SENSE_MOD
		FAN_OFF;
		while (1) {
			beep(3);
			display_string("FAN");
			delay(1000);
#ifdef CURRENT_SENSE_MOD
			display_string("CUR");
#else				//CURRENT_SENSE_MOD
			display_string("SPD");
#endif				//CURRENT_SENSE_MOD
			delay(2000);
			clear_display();
			delay(1000);
		}
	}

	FAN_OFF;

}

void setup_timer1_ctc(void)
{
	// ATmega168 running at 8MHz internal RC oscillator
	// Timer1 (16bit) Settings:
	// prescaler (frequency divider) values:   CS12    CS11   CS10
	//                                           0       0      0    stopped
	//                                           0       0      1      /1  
	//                                           0       1      0      /8  
	//                                           0       1      1      /64
	//                                           1       0      0      /256 
	//                                           1       0      1      /1024
	//                                           1       1      0      external clock on T1 pin, falling edge
	//                                           1       1      1      external clock on T1 pin, rising edge
	//
	uint8_t _sreg = SREG;	/* save SREG */
	cli();			/* disable all interrupts while messing with the register setup */

	/* set prescaler to 256 */
	TCCR1B &= ~(_BV(CS11) | _BV(CS10));
	TCCR1B |= _BV(CS12);

	/* set WGM mode 4: CTC using OCR1A */
	TCCR1A &= ~(_BV(WGM10) | _BV(WGM11));
	TCCR1B |= _BV(WGM12);
	TCCR1B &= ~_BV(WGM13);

	/* normal operation - disconnect PWM pins */
	TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0));

	/* set top value for TCNT1 */
	OCR1A = 640;		// key debouncing every 20.48ms
	OCR1B = 8;			// new segment every 256µs, complete display update every 6ms <=> 160Hz

	/* enable COMPA and COMPB isr */
	TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);

	/* restore SREG with global interrupt flag */
	SREG = _sreg;
}

ISR(TIMER1_COMPB_vect)
{
	static uint8_t digit = 0;

	digit++;

	if (digit == 24) {
		digit = 0;
	}

	uint8_t bm;
	// explicit switch is faster than variable shifting
	switch (digit & 0x07) {
	case 0:
		bm = ~(1 << 0);
		break;
	case 1:
		bm = ~(1 << 1);
		break;
	case 2:
		bm = ~(1 << 2);
		break;
	case 3:
		bm = ~(1 << 3);
		break;
	case 4:
		bm = ~(1 << 4);
		break;
	case 5:
		bm = ~(1 << 5);
		break;
	case 6:
		bm = ~(1 << 6);
		break;
	case 7:
		bm = (uint8_t) ~ (1 << 7);
		break;
	}
	// all segments OFF (set HIGH, as current sinks)
	SEGS_OFF;
    
	switch (digit / 8) {
	case 0:
		DIG0_ON;			// turn on digit #0
		PORTD = fb[0] | bm;
		DIG1_OFF;
		DIG2_OFF;
		break;
	case 1:
		DIG1_ON;			// turn on digit #1
		PORTD = fb[1] | bm;
		DIG0_OFF;
		DIG2_OFF;
		break;
	case 2:
		DIG2_ON;			// turn on digit #2
		PORTD = fb[2] | bm;
		DIG0_OFF;
		DIG1_OFF;
		break;
	default:
		DIG0_OFF;
		DIG1_OFF;
		DIG2_OFF;
		break;
	}

	if (OCR1B == 640) {
		OCR1B = 8;
	} else {
		OCR1B += 8;
	}
}

ISR(TIMER1_COMPA_vect)
{
	// explained in https://www.mikrocontroller.net/articles/Entprellung#Komfortroutine_.28C_f.C3.BCr_AVR.29
	static uint8_t ct0, ct1, rpt;
	uint8_t i;

	i = key_state ^ ~KEY_PIN;			// key changed ?
	ct0 = ~(ct0 & i);					// reset or count ct0
	ct1 = ct0 ^ (ct1 & i);				// reset or count ct1
	i &= ct0 & ct1;						// count until roll over ?
	key_state ^= i;						// then toggle debounced state
	key_press |= key_state & i;			// 0->1: key press detect

	if ((key_state & REPEAT_MASK) == 0)	// check repeat function
		rpt = REPEAT_START;				// start delay
	if (--rpt == 0) {
		rpt = REPEAT_NEXT;				// repeat delay
		key_rpt |= key_state & REPEAT_MASK;
	}

	if (++display_blink > 50)
		display_blink = 0;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint8_t get_key_press(uint8_t key_mask)
{
	cli();			// read and clear atomic !
	key_mask &= key_press;	// read key(s)
	key_press ^= key_mask;	// clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint8_t get_key_rpt(uint8_t key_mask)
{
	cli();			// read and clear atomic !
	key_mask &= key_rpt;	// read key(s)
	key_rpt ^= key_mask;	// clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state(uint8_t key_mask)
{
	key_mask &= key_state;
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_short(uint8_t key_mask)
{
	cli();			// read key state and key press atomic !
	return get_key_press(~key_state & key_mask);
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_long(uint8_t key_mask)
{
	return get_key_press(get_key_rpt(key_mask));
}

uint8_t get_key_long_r(uint8_t key_mask)
{				// if repeat function needed
	return get_key_press(get_key_rpt(key_press & key_mask));
}

uint8_t get_key_rpt_l(uint8_t key_mask)
{				// if long function needed
	return get_key_rpt(~key_press & key_mask);
}

uint8_t get_key_common(uint8_t key_mask)
{
	return get_key_press((key_press & key_mask) == key_mask ? key_mask : 0);
}

uint8_t get_key_common_l(uint8_t key_mask)
{
	return get_key_state((key_press & key_mask) == key_mask ? key_mask : 0);
}

#ifdef USE_WATCHDOG
void watchdog_off(void)
{
	wdt_reset();
	MCUSR &= ~_BV(WDRF);	// clear WDRF, as it overrides WDE-bit in WDTCSR-reg and leads to endless reset-loop (15ms timeout after wd-reset)
	wdt_disable();
}

void watchdog_on(void)
{
	wdt_reset();
	MCUSR &= ~_BV(WDRF);	// clear WDRF, as it overrides WDE-bit in WDTCSR-reg and leads to endless reset-loop (15ms timeout after wd-reset)
	wdt_enable(WDTO_500MS);
}
/*
void watchdog_off_early(void)
{
	wdt_reset();
	_mcusr = MCUSR;
	MCUSR &= ~_BV(WDRF);	// clear WDRF, as it overrides WDE-bit in WDTCSR-reg and leads to endless reset-loop (15ms timeout after wd-reset)
	wdt_disable();
}
*/

void test_F_CPU_with_watchdog(void)
{
/*
 * Hopefully cause a watchdog reset if the CKDIV8 FUSE is set (F_CPU 1MHz instead of 8MHz)
 *
 */
	wdt_reset();
	MCUSR &= ~_BV(WDRF);	// clear WDRF, as it overrides WDE-bit in WDTCSR-reg and leads to endless reset-loop (15ms timeout after wd-reset)
	wdt_enable(WDTO_120MS);
	delay(40);		// IF "CKDIV8" fuse is erroneously set, this should delay by 8x40 = 320ms & cause the dog to bite!

	watchdog_off();		// IF we got to here, F_CPU is OK.
}
#endif


void fb_update()
{
	if (!framebuffer.changed)
		return;

	uint8_t _sreg = SREG;	/* save SREG */
	cli();					/* disable all interrupts to avoid half-updated screens */

	for (uint8_t digit = 0; digit < 3; digit++) {
		display_char(digit, framebuffer.digit[digit], framebuffer.dot[digit]);
	}
	framebuffer.changed = 0;

	SREG = _sreg;
}

void beep(uint8_t beep_count)
{
	if(sound_on.value != 1)		
		return;

	if (beep_count < 1 || beep_count > 4)
		beep_count = 1;

	for (uint8_t i=0; i<beep_count; i++){
		BEEPER_ON;
		delay(100);
		BEEPER_OFF;
		delay(100);
	}
}
