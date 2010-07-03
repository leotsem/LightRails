/*

 -- user input functions

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

void check_input() {

   // this function determines if any button has been hit,
   // and what to do about it

   // up and down do not require a HIGH (unpressed) reading
   // so that you can hold them down and quickly cycle through
   // values (about 3 changes per second)

 if( digitalRead(B_UP) == LOW && millis() - button_hit > 300) {

      button_hit = millis();
      take_button_action(0);
      return;
 }

 if( digitalRead(B_DN) == LOW && millis() - button_hit > 300 ) {
      //pre_dn = true;
      take_button_action(1);
      button_hit = millis();
      return;
 }

 if( digitalRead(B_LT) == LOW) {
  if(buttons & B00100000 && millis() - button_hit > 300) {  
      buttons &= B11011111;

      take_button_action(2);
      button_hit = millis();
      return;
   }

   buttons &= B11011111;

 }
  else {
    buttons  |= B00100000;
  }

 if( digitalRead(B_RT) == LOW ) {
   if( buttons & B00010000 && millis() - button_hit > 300) {
      buttons &= B11101111;
      take_button_action(3);
      button_hit = millis();
      return;
   }
  buttons &= B11101111;
 }
 else {
   buttons |= B00010000;
 }

 if( digitalRead(B_CT) == LOW ) {
   if( buttons & B00001000 && millis() - button_hit > 300) {
      buttons &= B11110111;
      take_button_action(4);
      button_hit = millis();
      return;
   }
   buttons &= B11110111;
 }
 else {
   buttons |= B00001000;
 }

}

void take_button_action( byte button ) {

  // this function operates whatever action is desired when
  // a button is pressed.  called by check_input()

  switch(button) {
    case 0:
      // up hit

      if( status & B00010000 ) {

            // we're in a setup screen...

            // increasing value at current cursor position

          change_setup_value(setup_step, 1);

            // update flag to show change in display value
          status |= B00000001;

          return;
      }
      else {      
        // on main screen - up turns intervalometer on

            // if intervalometer on already, do nothing

          if( status & B10000000 )
            return;

            // turn on intervalometer
          status |= B10000000;
            // save current exposure reading for latch
          pre_exp_tm = exp_tm; 

        }

      break;

    case 1:
      // down hit

      if( status & B00010000 ) {
            // we're in a setup screen     
            // decreasing value at current cursor position

          change_setup_value(setup_step, -1);
          status |= B00000001;

          return;
      }
      else {

           // main menu - turn off intervalometer
           // re-set latch matched (if set)
         status &= B01110111;
         shots_fired = 0;
       }

      break;

   case 2:
     // left button was hit

      // do nothing if in setup menu          
     if( status & B00010000 )
       return;

           // main display, cycle through display values

     if( disp_enable & B10000000 ) {
           // we were already left, move to the furthest right
           // value
         disp_enable = B00000001;
           // set screen to update
         status |= B00000001;
     }
     else {
             // shift current display value left
           disp_enable = disp_enable << 1;
           status |= B00000001;
     }  

     break;

   case 3:
     // right button was hit

      // do nothing if in setup menu    
     if( status & B00010000 )
         return;       

           // main display, cycle through display values

       if( disp_enable & B00000001 ) {
           // we were already at furthest right value,
           // need to move to furthest left value
         disp_enable = B10000000;

           // set screen to update
         status |= B00000001;
       }
       else {
             // shift current display right
           disp_enable = disp_enable >> 1;
             // set screen to update
           status |= B00000001;
       }

     break;

   case 4:

      // center button hit

      if( status & B00010000 ) {
          // in setup mode - we're going to change which
          // setup value we're displaying

          // set display to update   
        status |= B00000001;

        if( setup_step == MAX_SETUP_STEPS - 1 ) {
            // no more steps to go, exit menu 
          setup_step = 0;
            // set that we're no longer in setup
          status &= B11101111;
          return;
        }

        setup_step++;
        return;

      }
      else {
          // not in setup or input -
          // go into setup mode, tell display
          // to refresh
        status |= B00010001;
        return;
      }

      break;
  }

}

void change_setup_value( byte which, int what ) {

  switch( which ) {
    case 0:

      {

        // change iso value, convert to logarithmic
        // scale first, then add or subtract one step
        // and convert back to arithmetic scale 

       float foo = log(iso_rating) / log(10);

       int din = ( 10 * foo ) + 1;

       din += what;

       if( din < 1 )
          din = 1; 

       float bar = pow(10, (float) ( (float) din - 1) / (float) 10 );

       if( bar > int(bar) ) {
          iso_rating = int(bar) + 1;
       }
        else {
          iso_rating = int(bar);
        }

      }

      break;

    case 1:
      {
          // move f_stop value - move one third step
          // either direction

        float steps = log( f_stop ) / log( sqrt(2) );

        if( what > 0 ) {
          steps += 0.3333;
        }
        else {
          steps -= 0.3333;
        }

        if( steps < 0 )
          steps = 0;

        f_stop = pow( sqrt(2), steps );

      }
      break; 

    case 2:
        // ev adjust moes in 1/4 EV increments

      ev_adjust += (float) what * 0.25;
      break;

    case 3:
        // ev steps determines how many divisions of an EV
        // there are when calculating automatic exposure
        // values.  more steps = better granularity in exposure
        // timing.

      ev_steps += what;
      if( ev_steps < 0 )
        ev_steps = 0;

      break;

    case 4:

      camera_delay += what;

      if( camera_delay < 1 )
        camera_delay = 1;

        // ms delay time
      real_camera_delay = camera_delay * 1000;

      break;    

    case 5: 

        // cycle between latch settings

      if( status & B01000000 ) {
        status &= B10111111;
        status |= B00100000;
      }
      else if( status & B00100000 ) {
        status &= B10011111;
      }
      else {
        status |= B01000000;
      }

      break;

    case 6:

      min_exp_tm += what;

        // don't allow min time to be less than 0
      if( min_exp_tm <= 0 )
        min_exp_tm = 0;

      break;

    case 7:

      max_exp_tm += what;

        // don't allow max time to be less than 0
      if( max_exp_tm <= 0 )
        max_exp_tm = 0;

      break;

    case 8:

        // change scaling:
        // scaling value can be 2, 10, or 100
        // allow wrap around from highest to lowest
        // and vice-versa
      freq_mult += what;

        // we only get to 11 by going up from 10 and
        // we only 1 by going down from 2 - next step
        // either way is 100
      if( freq_mult == 11 || freq_mult == 1 ) {
        tsl_set_scaling(100);
      }
      else if( freq_mult == 9 || freq_mult == 101 ) {
          // if we're moving down from 10, or up from 100
          // move to lowest setting
        tsl_set_scaling(2);
      }
      else {
          // if neither of the above cases is true, then our only
          // destination is 10
        tsl_set_scaling(10);
      }

      break;

    case 9:
      {
          // move ttl_stop value - move one third step
          // either direction

        float steps = log( ttl_stop ) / log(sqrt(2));

        if( steps < 0 )
          steps = 0;

        if( what > 0 ) {
          steps += 0.3333;
        }
        else {
          steps -= 0.3333;
        }

          // for ttl stop, we want to get to zero, if possible
          // for non-ttl readings.  We go ahead and skip every stop below
          // 1.0 for the heck of it. (Very few lenses go below 1.0)

        if( steps < 0 ) {
          ttl_stop = 0.0;
        }
         else {
           ttl_stop = pow( sqrt(2), steps );

            // handle rounding for higher f-stops
           if( ttl_stop > 7.1 )
             ttl_stop = int(ttl_stop);
        }

      }
      break; 

    case 10:

      light_type = light_type == 1 ? 0 : 1;
      break;

    case 11:

      ev_diff_ceiling += what;
      break;

    default:
      break;
  }

 return;
}