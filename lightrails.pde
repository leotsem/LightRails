/*

 LightRails 1.0
   A dynamic external exposure control system with
   integrated intervalometer for time-lapse
   photography

 Copyright (c) 2008-2009 C. A. Church drone< a_t >dronecolony.com

 This program is free software: you can redistribute it
 and/or modify it under the terms of the GNU General
 Public License as published by the Free Software
 Foundation, either version 3 of the License, or (at
 your option) any later version.

 This program is distributed in the hope that it will be
 useful, but WITHOUT ANY WARRANTY; without even the
 implied warranty of MERCHANTABILITY or FITNESS FOR A
 PARTICULAR PURPOSE. See the GNU General Public License
 for more details. 

*/

#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <MsTimer2.h>

#define TSL_FREQ_PIN 2
#define TSL_S0         6
#define TSL_S1         5
#define TSL_S2         3
#define TSL_S3       4

#define CAMERA_PIN   13

 // buttons

#define B_LT   9
#define B_RT   10
#define B_UP   7
#define B_DN   8
#define B_CT   11

  // max number of setup steps available

#define MAX_SETUP_STEPS   12

  // read frequency for how many milliseconds
  // note: this code only works for 1000 any other value
  // and you need to adjust your frequency calculation

#define READ_TM      1000

  // low and high thresholds for adjusting sensitivity
  // of the TSL230R.  If freq is below LO thresh, it will
  // automatically increase sensitivity.  If it increases
  // beyond the HI thresh, it will decrease sensitivity

#define SENS_THRESH_LO 101
#define SENS_THRESH_HI 7999

  // LCD print buffer
  // set to LCD width + 1

#define SIZE_OF_LCD_BUF 17

 // threshold to accept a frequency change
 // the frequency read off the chip must change by
 // at least this many Hz to accept the change
 // this prevents fluttering.  However, it can cause
 // problems with low-light level changes.  You may need
 // to adjust according to types of shooting you
 // typically do (higher for more day shooting
 // and lower for more night shooting)

#define FREQ_CHG_THRESH  20

 // the minimum exposure gap is the minimum time
 // between triggering exposures (for the cases where
 // exposure_time exceeds interval_time).  At a bare
 // minimum, it should be the minimum amount of time
 // required by your camera to consistently register
 // the completion of one exposure and execution of
 // another.  this time is in milliseconds

#define MIN_EXP_GAP 1000

 // some info about exposure data
 // need iso and f/stop to calculate
 // exposure time.  These are just
 // defaults - they are adjusted via the UI

int   iso_rating = 100;
float f_stop     = 8.0;

 // Maximum Diff % in EV Value calculated from reading to reading
byte ev_diff_ceiling  = 0; 

 // for through-a-lens metering - f-stop of the lens
 // (light read must be calculated)

float ttl_stop = 0.0;

 // default times
unsigned int camera_delay       = 5;
unsigned int min_exp_tm         = 10;
byte         max_exp_tm         = 20;

 // actual delay time in mS

unsigned long real_camera_delay = camera_delay * 1000;

  // running variables

float         uwcm2          = 0; // uw/cm2 calculated 
float         lux            = 0; // Lux caclulated
float         ev             = 0; // EV value calculated
float         exp_tm         = 0; // camera exposure time calculated in 1/x value
float         pre_exp_tm     = 0; // for latching, need previous reading
unsigned long frequency      = 0; // frequency read
unsigned long cur_exp_tm     = 0; // current exposure time in mS
unsigned int  shots_fired    = 0; // how many exposures taken since interval started

bool camera_engaged          = false; // camera currently exposing?
bool light_type              = false; // which spd graph to use - d65 [0] or ilA [1]

 // this is used by the pulse counter.

volatile unsigned long pulse_cnt = 0;

 // for calculating time differences

unsigned long pre_tm         = millis();

unsigned int  freq_tm_diff   = 0; // how much time has passed since last frequency reading
                                  // -- note the int size limit
unsigned long camera_tm_diff = 0; // how much time has passed since last exposure

 // this is used by the sensitivity adjustment
 // to record the amount to divide the freq by
 // to get the uW/cm2 value at 420nm

unsigned int  calc_sensitivity   = 10;

 // this is the frequency calculation multiplier
 // based on what scaling factor is chosen. 
 // set_scaling() will adjust it for you.

byte freq_mult = 1;

 // our previous frequency count
 // -- we set it to fifty so it doesn't swing
 // auto-sensitivity adjustment either way on the first
 // reading

unsigned long last_cnt  = 50;

  // some EV tracking variables

float prev_ev = 0;
float set_ev  = 0;

  // our wavelengths (nm) we're willing to calculate illuminance for (lambda)
int wavelengths[18] = { 380, 400, 420, 440, 460, 480, 500, 520, 540, 560, 580, 600, 620, 640, 660, 680, 700, 720 };
  // the CIE V(l) for photopic vision - CIE Vm(l) 1978 - mapping to the same (l) above
float v_lambda[18]  = { 0.0002, 0.0028, 0.0175, 0.0379, 0.06, 0.13902, 0.323, 0.71, 0.954, 0.995, 0.87, 0.631, 0.381, 0.175, 0.061, 0.017, 0.004102, 0.001047 };

  // CIE SPD graphs for D65 and Illuminant A light sources, again mapping to same lambda as included in wavelengths
float spd_graphs[2][18] = {
  { 49.975500, 82.754900, 93.431800, 104.865000, 117.812000, 115.923000, 109.354000, 104.790000, 104.405000, 100.000000, 95.788000, 90.006200, 87.698700, 83.699200, 80.214600, 78.284200, 71.609100, 61.604000 },
  { 9.795100, 14.708000, 20.995000, 28.702700, 37.812100, 48.242300, 59.861100, 72.495900, 85.947000, 100.000000, 114.436000, 129.043000, 143.618000, 157.979000, 171.963000, 185.429000, 198.261000, 210.365000 }
};

/*

  Let's setup all of our display strings for the LCD

  These are stored as PROGMEM so they don't take up valuable
  SRAM

*/
  // main display strings

// setup strings

const prog_char setup_str1[] PROGMEM  = "ISO";
const prog_char setup_str2[] PROGMEM  = "Target F/Stop";
const prog_char setup_str3[] PROGMEM  = "EV Adjust";
const prog_char setup_str4[] PROGMEM  = "EV Steps";
const prog_char setup_str5[] PROGMEM  = "Max Exp. - Sec";
const prog_char setup_str6[] PROGMEM  = "Exposure Cycle";
const prog_char setup_str7[] PROGMEM  = "Latch";
const prog_char setup_str8[] PROGMEM  = "Min Exp. - mSec";
const prog_char setup_str9[] PROGMEM  = "Scale Read";
const prog_char setup_str10[] PROGMEM = "TTL F/Stop";
const prog_char setup_str11[] PROGMEM = "Light Source";
const prog_char setup_str12[] PROGMEM = "EV Change Max";

PGM_P setup_strings[MAX_SETUP_STEPS] PROGMEM =
 {
   setup_str1,
   setup_str2, 
   setup_str3,
   setup_str4, 
   setup_str6,
   setup_str7,
   setup_str8,
   setup_str5,
   setup_str9,
   setup_str10,
   setup_str11,
   setup_str12
 };

 // buffer for above setup strings
char lcd_print_buffer[SIZE_OF_LCD_BUF];

 // ev adjust can be negative
 //
 // how many EV steps to add or subtract
 // to each EV calculation - UI moves in 1/4EV
 // increments.

float ev_adjust     = 0.0;

 // how many steps per EV to deal with -
 // calculations round to the nearest step
 // and adjustments happen in these steps,
 // e.g.: 10 steps = .1 EV per step,
 // 1 step = 1 EV per step

byte ev_steps     = 100;

 // status flags:
 // B0 = intervalometer enable
 // B1 = latch up
 // B2 = latch down
 // B3 = in setup
 // B4 = latched (exp tm not moved due to latch)
 // B5 = latched to min/max exp tm setting
 // B6 = ev adjust enabled
 // B7 = update setup display

byte status = B00000000;

 // current setup step

byte setup_step  = 0;

 // display type flags
 // 0 = 1/x shutter speed/time (calculated)
 // 1 = ms shutter speed (calculated)
 // 2 = frequency
 // 3 = EV (calculated)
 // 4 = W/m2 (calculated)
 // 5 = sensitivity (10, 100, 1000)
 // 6 = lux (calculated)
 // 7 = scale

byte disp_enable = B10000000;

 // button hit flags (plus a few others)
 // B0 = up
 // B1 = dn
 // B2 = lt
 // B3 = rt
 // B4 = ct
 // B5
 // B6 =
 // B7 = camera exposing (use this to save a var elsewhere)

byte buttons = B00000000;

 // time last button was hit
unsigned long button_hit = 0;

 // init lcd display

LiquidCrystal lcd(14, 14, 15, 16, 17, 18, 19);

void setup() {

 lcd.clear();
 lcd.print("LightRails/1.0"); 

  // attach interrupt to pin8, send output pin of TSL230R to arduino 8
  // call handler on each rising pulse

 attachInterrupt(0, add_pulse, RISING);

 pinMode(TSL_FREQ_PIN, INPUT);

 pinMode(TSL_S0, OUTPUT);
 pinMode(TSL_S1, OUTPUT);
 pinMode(TSL_S2, OUTPUT);
 pinMode(TSL_S3, OUTPUT);

 pinMode(CAMERA_PIN, OUTPUT);

   // set input button pins

 pinMode(B_LT, INPUT);
 pinMode(B_RT, INPUT);
 pinMode(B_UP, INPUT);
 pinMode(B_DN, INPUT);
 pinMode(B_CT, INPUT);

   // enable pullup resistors
 digitalWrite(B_LT, HIGH);
 digitalWrite(B_RT, HIGH);
 digitalWrite(B_UP, HIGH);
 digitalWrite(B_DN, HIGH);
 digitalWrite(B_CT, HIGH);

  // write TSL sensitivity 1x

 digitalWrite(TSL_S0, HIGH);
 digitalWrite(TSL_S1, LOW);

  // set frequency scaling

 tsl_set_scaling(2); 

}

void loop() {

  // operate camera
  // handle user input
  // update light sensor data

    // calculate how much time has passed
    // both for camera firing and light reading

    // this code requires arduino 012 or any version with
    // compatible millis() overflow and behavior

 freq_tm_diff   += millis() - pre_tm;
 camera_tm_diff += millis() - pre_tm;

 pre_tm          = millis();

  // if intervalometer enabled, enough time has passed,
  // and the camera isn't already engaged
  // - fire camera (camera delay is in seconds)

 if( status & B10000000 && camera_tm_diff >= real_camera_delay &&  camera_engaged == false ) {

       // re-set camera difference timer
     camera_tm_diff = 0;

       // trigger camera remote
     fire_camera();

      // convert sec to msec for next exposure check
      // always try and use the configured value first
      // (see next operation)

     real_camera_delay = camera_delay * 1000;

       // assure MIN_EXP_GAP threshold is enforced
       // between exposure triggers
       //
       // if the exposure time leaves less than the
       // minimum gap between exposures (MIN_EXP_GAP
       // must be <= real_camera_delay), add the minimum gap
       // to the current exposure time, and make that the
       // interval time before the next shot

     if( cur_exp_tm >= ( real_camera_delay - MIN_EXP_GAP ) )
       real_camera_delay = cur_exp_tm + MIN_EXP_GAP;

       // update count of shots fired
     shots_fired++;

 } // end if( status...   

   // see if the user has pressed any buttons, and handle
   // them if need be

 check_input();

   // if a second has passed, we need to
   // update our calculations

 if( freq_tm_diff >= READ_TM ) {

     // reset time counter

   freq_tm_diff = 0;

    // get current frequency

   frequency = tsl_get_freq(); 

    // ignore changes in frequency below FREQ_CHG_THRESH

   if( last_cnt > 0 && abs( (long) ( frequency - last_cnt ) ) < FREQ_CHG_THRESH )
     frequency = last_cnt;

     // set other needed values

   if ( frequency > 0 ) {  

       // chip has given us a positive reading

       // calculate power   
     uwcm2     = tsl_calc_uwatt_cm2(frequency);   

     if( uwcm2 <= 0 ) {
         // handle no actual power reading available
       uwcm2 = 0;
       lux   = 0;
     }
      else {

        // there was a positive power reading...

        // calculate lx value using gaussian formula

       lux = calc_lux_gauss(uwcm2);
      }
   }
   else {
       // 0 frequency read from chip - set all values to zero
     uwcm2 = 0;
     lux   = 0;
   }

   if ( lux <= 0 ) {
       // don't try calculating nonsense - we cant calculate
       // an EV with no light.
     lux = 0;
     ev = -6.0;
   }
   else {
       // we have positive lux reading
       // calculate EV from lux reading
     ev = calc_ev(lux);
   }

      // calculate exposure time value

   exp_tm     = calc_exp_tm( ev, f_stop );

      // determine if we need to latch on to a particular exposure
      // as lowest or highest allowed

   if( status & B10000000 && status & B01100000) {
     // latch high or low enabled - and intervalometer on     

     if( status & B01000000 ) {
         // latch low is enabled (don't increase exposure time)
         // remember that exp_tm is a divisor - so a higher number == faster exposure
       if( exp_tm > pre_exp_tm) {
         exp_tm = pre_exp_tm;
           // indicate that exposure is latched
         status |= B00001000;
       }
       else {
           // reset latch indicator
         status &= B11110111;
           // set new exposure value to keep exp from
           // getting longer than current exp. (new
           // ceiling)
         pre_exp_tm = exp_tm;
       }
     }
     else if( status & B00100000 ) {
         // latch high enabled (don't decrease exposure time)
       if( exp_tm < pre_exp_tm ) {
          exp_tm = pre_exp_tm;
          status |= B00001000;
       }
       else {
         status &= B11110111;
           // new floor
         pre_exp_tm = exp_tm;
       }
     }
   } // end if latch high or low enabled

     // calculate exposure ms
     // we can only go as short as 1ms, so don't
     // try and calculate below that.  Check for
     // 1/1000 ceiling

   if( exp_tm >= 1000 ) {
    cur_exp_tm = 1;
   }
   else {
     cur_exp_tm = calc_exp_ms( exp_tm );
   }

    // check for minimum/max exposure time

   if( cur_exp_tm < min_exp_tm ) {
       // bring time up to min. exposure time
     cur_exp_tm = min_exp_tm;
       // set exposure time ceiling/floor engaged
     status |= B00000100;
   }
     // max exposure time is in seconds
   else if( max_exp_tm > 0 && cur_exp_tm > ( max_exp_tm * 1000 ) ) {
     cur_exp_tm = max_exp_tm * 1000;
       // set exposure time ceiling/floor engaged
     status |= B00000100;
   }
   else {
       // at neither min nor max exp. time
       // reset exposure time ceiling/floor flag
     status &= B11111011;
   }

         // determine if we need to change sensitivity --
         // two readings in a row must pass our thresholds

   if( frequency < SENS_THRESH_LO && last_cnt < SENS_THRESH_LO && calc_sensitivity < 1000 ) {
      tsl_sensitivity( HIGH );
   }
   else if( frequency > SENS_THRESH_HI && last_cnt > SENS_THRESH_HI && calc_sensitivity > 10) {
      tsl_sensitivity( LOW );
   }

     // save off current reading so we can see if we need to adjust sensitivity
     // on the next pass

   last_cnt = frequency;

     // set display to update

   status |= B00000001;   

 } // end if(freq_tm_diff > READ_TM)

    //
    // update user interface as the last step in the main
    // loop...
    // 

 if(  status & B00010000 && status & B00000001 ) {
     // in setup and screen needs updating

       // clear update flag
     status &= B11111110;

       // display setup info

     print_setup_display();
 }
  else if( status & B00000001 ) {
    // on main screen and need to update display

       // clear update flag
     status &= B11111110;

     print_info();

  }

} // end main loop

 // interrupt handler to function as a pulse-counter

void add_pulse() {

  // increase pulse count
 pulse_cnt++;
 return;
}

float float_abs_diff( float f1, float f2 ) {

  float diff = f1 > f2 ? f1 - f2 : f2 - f1;

  if( diff < 0 )
    diff *= 1;

  return(diff);
}