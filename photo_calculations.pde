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

/* 

 Functions to calculate photographic values
 from light readings, the following functions are
 found here:

 calc_lux_single*
 calc_lux_gauss
 calc_ev
 calc_exp_tm
 calc_exp_ms

 Last Modified: 2/29/2009, c.a. church
 Original:      10/15/2008, c.a. church
*/ 

/* 

  We always use multiple wavelengths.  This function is
  if you happen to be erm, you know, filming something illuminated
  by a single wavelength light source.  Given the rarity of this
  situation, you'll have to provide the luminous efficiency function
  value yourself.

float calc_lux_single(float uw_cm2, float efficiency) {

    // calculate lux (lm/m^2) for single wavelength, using standard formula:
    // Xv = Xl * V(l) * Km
    // Xl is W/m^2 (calculate actual receied uW/cm^2, extrapolate from sensor size (0.0136cm^2)
    // to whole cm size, then convert uW to W)
    // V(l) = efficiency function (provided via argument)
    // Km = constant, L/W @ 555nm = 683 (555nm has efficiency function of nearly 1.0)
    //
    // Only a single wavelength is calculated - you'd better make sure that your
    // source is of a single wavelength...  Otherwise, you should be using
    // calc_lux_gauss() for multiple wavelengths

  return( ( ( uw_cm2 * ( (float) 1 / (float) 0.0136) ) / (float) 1000000 ) * (float) 100   * efficiency * (float) 683 );
}

*/

float calc_lux_gauss( float uw_cm2 ) {

    // # of wavelengths mapped to V(l) values - better have
    // enough V(l) values!

  int nm_cnt = sizeof(wavelengths) / sizeof(int);

    // watts/m2

  float w_m2 =  ( uw_cm2 * ( (float) 1 / (float) 0.0136 ) / (float) 1000000 ) * (float) 100;

  float result = 0;

    // integrate XlV(l) dl
    // Xl = uW-m2-nm caclulation weighted by the CIE lookup for the given light
    //   temp
    // V(l) = standard luminous efficiency function

  for( int i = 0; i < nm_cnt; i++) {

    if( i > 0) {
      result +=  ( spd_graphs[light_type][i] / (float) 1000000)  * (wavelengths[i] - wavelengths[i - 1]) * w_m2  * v_lambda[i];
    }
    else {
      result += ( spd_graphs[light_type][i] / (float) 1000000) * wavelengths[i] * w_m2 * v_lambda[i];
    }

  }

    // multiply by constant Km and return

  return(result * (float) 683);
}

float calc_ev( float lux ) {

    // calculate EV using APEX method:
    // Ev = Av + Tv = Bv + Sv
    // Bv = log2( B/NK )
    // Sv = log2( NSx )
    // K  = Meter Calibration Constant
    //      14 = Pentax standard
    // N  = constant relationship between Sx and Sv

  float ev = ( log( (float)  0.3 * (float) iso_rating ) / log(2) ) + ( log( lux / ( (float) 0.3 * (float) 14.0 ) ) / log(2) );

   // round down if EV step
   // value is set to whole steps

  if( ev_steps == 1 )
    ev = (float) int(ev);

    // convert to positive value temporarily, if needed
  bool neg_ev = false;

  if( ev < 0 ) {
     neg_ev = true;
     ev *= -1;
   }

   if( ev > int(ev) ) {

      // if ev has a decimal value, determine nearest
      // fraction of EV to round to, based on ev step
      // setting

         // handle rounding to nearest step

       int rem = ( ( ev - int(ev) ) * 100 );    
       int step = 100 / ev_steps;

       for (int i = ev_steps; i > 0; i-- ) {

         if( rem >= step * i) {
           rem = step * i;
           break;
         }
       }

       ev = (float) int(ev) + ((float) rem / 100);

   }

    // reset back to negative EV value
  if( neg_ev == true )
    ev *= -1.0;

    // is there a ceiling set for maximum EV change?
    // if so, handle only moving a maximum amount of <ceiling>
    // steps between readings

  if( ev_diff_ceiling > 0 ) {

    float diff = float_abs_diff( ev, prev_ev );
    float ceiling = (float) ev_diff_ceiling / (float) ev_steps;

    if( diff > ceiling ) {

      if( ev < prev_ev ) {
        ev = prev_ev - ceiling;
      }
      else {
        ev = prev_ev + ceiling;
      }

    }

  }

  prev_ev = ev; 

    // if ev adjust enabled - apply it now 

   ev += ev_adjust;

    // deal with TTL compensation
    // each full aperture step results in one less
    // EV read in, so adjust output
    // up by one EV per stop    

  if( ttl_stop > 0 )
    ev += log( ttl_stop ) / log(sqrt(2));

  return(ev);

}

float calc_exp_tm ( float ev, float aperture  ) {

    // Ev = Av + Tv = Bv + Sv
    // need to determine Tv value, so Ev - Av = Tv
    // Av = log2(Aperture^2)
    // Tv = log2( 1/T ) = log2(T) = 2 ^^ (Ev - Av)

  float exp_tm = ev - ( log( pow(aperture, 2) ) / log(2) );

  float exp_log = pow(2, exp_tm); 

  return( exp_log  );
}

unsigned long calc_exp_ms( float exp_tm ) {

  return( (unsigned long) ( 1000 / exp_tm ) );

    // if you wish to round actual mS for exposure to nearest
    // exposure step (the actual exposure step displayed in 1/x
    // format), un-comment the following code and comment the
    // line above out.
    //
    // Mind you, that doing so will make the device
    // no more accurate than your standard camera's meter.

/*   
      // deal with times less than or equal to half a second
   if( exp_tm >= 2 ) {

     if( exp_tm >= (float) int(exp_tm) + (float) 0.5 ) {
       exp_tm = int(exp_tm) + 1;
     }
     else {
       exp_tm = int(exp_tm);
     }

     return(1000 / exp_tm);

   }
   else if( exp_tm >= 1 ) {
     // deal with times larger than 1/2 second

     float disp_v = 1 / exp_tm;
       // get first significant digit
     disp_v       = int( disp_v * 10 );    
     return( ( 1000 * disp_v ) / 10 );

   }
    else {
      // times larger
     int disp_v = int( (float) 1 / exp_tm);
     return((unsigned long) 1000 * (unsigned long) disp_v);

    }
*/

}