/*

 LightRails - TSL230R Control and Conversion Functions

   Copyright (c) 2008-2009 C. A. Church drone< a_t >dronecolony.com

 unsigned long tsl_get_freq()
 float tsl_calc_uwatt_cm2(unsigned long freq)
 void tsl_sensitivity( bool dir )
 void tsl_set_scaling( int what )

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

unsigned long tsl_get_freq() {

    // we have to scale out the frequency - the only
    // 1:1 frequency pulse we get is 100x scale.  Smaller
    // scaling on the TSL230R requires us to multiply by a factor
    // to get actual frequency

  unsigned long freq = pulse_cnt * freq_mult;
  pulse_cnt = 0;
  return(freq);
}

float tsl_calc_uwatt_cm2(unsigned long freq) {

  // get uW observed - assume 640nm wavelength
  // calc_sensitivity is our divide-by to map to a given signal strength
  // for a given sensitivity (each level of greater sensitivity reduces the signal
  // (uW) by a factor of 10)
  //
  // note: this function does not take sensor size into account
  //       other functions handle this

  float uw_cm2 = (float) freq / (float) calc_sensitivity;

  return(uw_cm2);

}

void tsl_sensitivity( bool dir ) {

  // adjust sensitivity of TSL230R in 3 steps of 10x either direction

  int pin_0 = false;
  int pin_1 = false;

  if( dir == true ) {

      // increasing sensitivity

      // -- already as high as we can get
    if( calc_sensitivity == 1000 )
      return;

    if( calc_sensitivity == 100 ) {
        // move up to max sensitivity
      pin_0 = true;
      pin_1 = true;
    }
    else {
        // move up to med. sesitivity
      pin_1 = true;
    }

      // increase sensitivity divider
    calc_sensitivity *= 10;
  }
  else {
      // reducing sensitivity

      // already at lowest setting

    if( calc_sensitivity == 10 )
      return;

    if( calc_sensitivity == 100 ) {
        // move to lowest setting
      pin_0 = true;
    }
    else {
        // move to medium sensitivity
      pin_1 = true;
    }

      // reduce sensitivity divider
    calc_sensitivity = calc_sensitivity / 10;
  }

    // make any necessary changes to pin states

 digitalWrite(TSL_S0, pin_0);
 digitalWrite(TSL_S1, pin_1);

 return;
}

void tsl_set_scaling ( int what ) {

  // set output frequency scaling for TSL230R
  // when increasing the scaling for divide-by-output, you reduce the
  // freq multiplier by a factor of ten. 
  // e.g.:
  // scale = 2 == freq_mult = 100
  // scale = 10 == freq_mult = 10
  // scale = 100 == freq_mult = 1

  byte pin_2 = HIGH;
  byte pin_3 = HIGH;

  switch( what ) {
    case 2:
      pin_3     = LOW;
      freq_mult = 2;
      break;
    case 10:
      pin_2     = LOW;
      freq_mult = 10;
      break;
    case 100:
      freq_mult = 100;
      break;
    default:
      return;
  }

  digitalWrite(TSL_S2, pin_2);
  digitalWrite(TSL_S3, pin_3);

  return;
}