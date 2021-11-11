#ifndef youyoue858d_h
#define youyoue858d_h

//#define DEBUG						
//#define CURRENT_SENSE_MOD			

#define USE_WATCHDOG				

//#define WATCHDOG_TEST
//#define DISPLAY_MCUSR
				
#define FAN_PURGE_MOD

typedef struct CPARAM {
	int16_t value_min;
	int16_t value_max;
	int16_t value_default;
	int16_t value;
	uint8_t eep_addr_high;
	uint8_t eep_addr_low;
} CPARAM;

typedef struct {
	char digit[3];
	bool dot[3];
	bool changed:1;
} framebuffer_t;

void change_config_parameter(CPARAM * param, const char *string);
void char_test(void);
void clear_display(void);
void clear_dot(void);
void clear_eeprom_saved_dot(void);
void display_char(uint8_t digit, uint8_t character);
void display_number(int16_t number);
void display_set_temp(int16_t number);
void display_string(const char *string);
void eep_load(CPARAM * param);
void eep_save(CPARAM * param);
void fan_test(void);
int main(void);
void restore_default_conf(void);
void segm_test(void);
void set_dot(void);
void set_eeprom_saved_dot(void);
void setup_858D(void);
void setup_timer1_ctc(void);
void fb_update(void);
void beep(uint8_t beep_count);
#ifdef USE_WATCHDOG
uint8_t _mcusr __attribute__ ((section(".noinit")));
//void watchdog_off_early(void) __attribute__ ((naked)) __attribute__ ((section(".init3")));	// requires R1 to be zero! (do NOT use .init1)
void watchdog_off(void);
void watchdog_on(void);
void test_F_CPU_with_watchdog(void);
#endif
uint8_t get_key_press(uint8_t key_mask);
uint8_t get_key_rpt(uint8_t key_mask);
uint8_t get_key_state(uint8_t key_mask);
uint8_t get_key_short(uint8_t key_mask);
uint8_t get_key_long(uint8_t key_mask);
uint8_t get_key_long_r(uint8_t key_mask);
uint8_t get_key_rpt_l(uint8_t key_mask);
uint8_t get_key_common(uint8_t key_mask);
uint8_t get_key_common_l(uint8_t key_mask);

#define FAN_OFF ( PORTC |= _BV(PC0) )		// Set "1" on PС0 (Fan)
#define FAN_ON  ( PORTC &= ~_BV(PC0) )		// Set "0" on PС0
#ifdef FAN_PURGE_MOD
#define FAN_MAX_ON ( PORTC |= _BV(PC1) )	// Set "1" на PC1 (fan Purge)
#define FAN_MAX_OFF  ( PORTC &= ~_BV(PC1) )	// Set "0" on PC1
#endif
#define FAN_IS_ON ( !(PINC & _BV(PC0)) )	
#define FAN_IS_OFF ( PINC & _BV(PC0) )		

#define DIG0_OFF ( PORTB &= ~_BV(PB3) )
#define DIG1_OFF ( PORTB &= ~_BV(PB4) )
#define DIG2_OFF ( PORTB &= ~_BV(PB5) )

#define DIG0_ON ( PORTB |= _BV(PB3) )
#define DIG1_ON ( PORTB |= _BV(PB4) )
#define DIG2_ON ( PORTB |= _BV(PB5) )

#define SEGS_OFF ( PORTD = 0xFF )			// Disable all Segments

#define TRIAC_ON ( PORTB |= _BV(PB6) )		// Set "1" on PB6 (Triac)
#define HEATER_ON TRIAC_ON					
#define TRIAC_OFF ( PORTB &= ~_BV(PB6) )	// Set "0" on PB6 (Triac)
#define HEATER_OFF TRIAC_OFF				

#define BEEPER_ON ( PORTB |= _BV(PB0) )		// Set "1"
#define BEEPER_OFF  ( PORTB &= ~_BV(PB0) )	// Set "0"


#define SW0_PRESSED ( !(PINB & _BV(PB1)) )	
#define SW1_PRESSED ( !(PINB & _BV(PB2)) )	

#define REEDSW_CLOSED ( !(PINC & _BV(PC4)) ) // Reed Switch Closed (Wand on a Handle)  PC4 - "0"
#define REEDSW_OPEN ( PINC & _BV(PC4) )		

#define SHOW_SETPOINT_TIMEOUT 2000L

#define HEATER_DUTY_CYCLE_MAX 512L
#define PWM_CYCLES 512L

#define P_GAIN_DEFAULT 650
#define I_GAIN_DEFAULT 15
#define D_GAIN_DEFAULT 500
#define I_THRESH_DEFAULT 45
#define P_GAIN_SCALING 100.0
#define I_GAIN_SCALING 10000.0
#define D_GAIN_SCALING 25.0

//#define TEMP_OFFSET_CORR_DEFAULT 33
#define TEMP_OFFSET_CORR_DEFAULT 0
#define TEMP_SETPOINT_DEFAULT 200

#define TEMP_AVERAGES_DEFAULT 250L
#define TEMP_REACHED_MARGIN 3

#define MAX_TEMP_ERR 550L
#define SAFE_TO_TOUCH_TEMP 45

#define FAN_OFF_TEMP 45
#define FAN_ON_TEMP 65
#define FAN_OFF_TEMP_FANONLY (SAFE_TO_TOUCH_TEMP - 2)

#define FAN_TIMEOUT_DEFAULT 20

#define SOUND_ON_DEFAULT 1


 #define FAN_SPEED_MIN_DEFAULT 70UL
 #define FAN_SPEED_MAX_DEFAULT 900UL

//
//
// CPARAM fan_speed_min = { 0, 999, FAN_SPEED_MIN_DEFAULT, FAN_SPEED_MIN_DEFAULT, 18, 19 };
// CPARAM fan_speed_max = { 0, 999, FAN_SPEED_MAX_DEFAULT, FAN_SPEED_MAX_DEFAULT, 20, 21 };
//

// #define FAN_CURRENT_MIN_DEFAULT 30UL
// #define FAN_CURRENT_MAX_DEFAULT 71UL

#define SLP_TIMEOUT_DEFAULT 15

#define KEY_DDR         DDRB
#define KEY_PORT        PORTB
#define KEY_PIN         PINB
#define KEY_UP          1
#define KEY_DOWN        2
#define ALL_KEYS        (1<<KEY_DOWN | 1<<KEY_UP)

#define REPEAT_MASK     (1<<KEY_DOWN | 1<<KEY_UP)
#define REPEAT_START    20	// after 20*20.48ms = 409.6ms
#define REPEAT_NEXT     8	// every 6*20.48ms = 122.88ms

#endif

