/*

  -- LCD Display Functions

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

void print_setup_display() {

   // clear out lcd print buffer

 memset(lcd_print_buffer, 0, SIZE_OF_LCD_BUF);

   // get setup string from hash

 strcpy_P(lcd_print_buffer, (char*)pgm_read_word( &(setup_strings[setup_step])) );

 lcd.clear();
 lcd.setCursor(0, 0);

 lcd.print(lcd_print_buffer);

   // go to second line
 lcd.setCursor(0, 1);

   // determine which setup step to display

 switch(setup_step) {
     case 0:
       lcd.print(iso_rating, DEC);
       break;

     case 1:
             // handle rounding for higher f-stops

       if( f_stop > 7.1 ) {
         print_float((float) int(f_stop));
       }
       else {
         print_float(f_stop);
       }

       break;

     case 2:
       print_float(ev_adjust);
       break;

     case 3:
       lcd.print(ev_steps, DEC);
       break;

     case 4:
       lcd.print(camera_delay, DEC);
       break;

     case 5:
       if( status & B01000000 ) {
         lcd.print('-');
       }
       else if( status & B00100000 ) {
         lcd.print('+');
       }
       else {
         lcd.print('X');
       }
       break;

     case 6:

      lcd.print(min_exp_tm, DEC);
      break;

     case 7:

      lcd.print(max_exp_tm, DEC);
      break;

     case 8:

      lcd.print(freq_mult, DEC);
      break;

     case 9:

      print_float(ttl_stop);
      break;

     case 10:

      lcd.print(light_type, DEC);
      break;

     case 11:

      lcd.print(ev_diff_ceiling, DEC);
      break;

     default:
       break;
   }

return;
}

void print_info() {

   lcd.clear();

   lcd.setCursor(0, 0);

    // display set iso and f-stop  
   lcd.print(iso_rating);
   lcd.print(' ');

   lcd.print('f');

             // handle rounding for higher f-stops
   if( f_stop > 7.1 ) {
     print_float((float) int(f_stop));
   }
   else {
     print_float(f_stop);
   }

     // if intervalometer on, show 'I' indicator
     // and count of shots fired

   if( status & B10000000 ) {
     lcd.setCursor(15,0);
     lcd.print('I');

     if( shots_fired > 999 ) {
       lcd.setCursor(11, 0);
     }
     else if( shots_fired > 99 ) {
       lcd.setCursor(12,0);
     }
     else if( shots_fired > 9 ) {
       lcd.setCursor(13,0);
     }
     else {
       lcd.setCursor(14,0);
     }

     lcd.print(shots_fired, DEC);

   }

    // move to second line 
   lcd.setCursor(0,1);   

   if( disp_enable & B10000000 ) {

         // show exposure time in 1/x notation

        if( exp_tm >= 2 ) {
         float disp_v = 0;

         if( exp_tm >= (float) int(exp_tm) + (float) 0.5 ) {
           disp_v = int(exp_tm) + 1;
         }
         else {
           disp_v = int(exp_tm);
         }

         lcd.print("1/");
         lcd.print(exp_tm, DEC);
        }
        else if( exp_tm >= 1 ) {
           // deal with times larger than 1/2 second

         float disp_v = 1 / exp_tm;
           // get first significant digit
         disp_v       = int( disp_v * 10 );     

         lcd.print('.');
         lcd.print(disp_v, DEC);
         lcd.print('"');    
        }
        else {
         int disp_v = int( (float) 1 / exp_tm);

         lcd.print(disp_v, DEC);
         lcd.print('"');

      }
    }

   else if( disp_enable & B01000000 ) {
         // display actual exposure mS
       print_ul(cur_exp_tm);
       lcd.print("mS");
   }

   else if( disp_enable & B00100000 ) {
         // display frequency reading
     lcd.print("Fq ");
     lcd.print(frequency, DEC);

   }

   else if( disp_enable & B00010000 ) {

         // display EV
       lcd.print("EV ");
       print_float(ev);

       if( ev_adjust > 0 || ev_adjust < 0 ) {
           // show any adjustment made
         lcd.print('[');
         print_float(ev_adjust);
         lcd.print(']');
       }

   } 

   else if( disp_enable & B00001000 ) {
       // uW/m2
     print_float(uwcm2);
     lcd.print("uW");

   }
   else if( disp_enable & B00000100 ) {
       // sensitivity level
     lcd.print('S');
     lcd.print(calc_sensitivity, DEC);     

   }

   else if( disp_enable & B00000010 ) {
         // display lux
       lcd.print("Lx ");
       print_float(lux);

   }

   else if( disp_enable & B00000001 ) {
       // divide-by-factor display
     lcd.print('R');
     lcd.print(freq_mult, DEC);
   }

  lcd.setCursor(14,1);

          // minimum/maximum exposure time enabled?
  if( status & B00000100 )
       lcd.print('M');

           // exposure was latched?        
 if( status & B00001000 )
        lcd.print('L');

}

void print_float( float val ){

  if ( val < 0 ) {
    lcd.print('-');
    val *= -1;
  }    

  print_ul( (unsigned long int) int(val) ); 
  lcd.print('.');

    // add 1 digit for hundreths precision
  if( (val * 100) - (int(val) * 100)  < 10 )
    lcd.print('0');

  print_ul( (val * 100) - (int(val) * 100) );

}   

void print_ul( unsigned long val ) {

    // print unsigned long to lcd

   // clear out lcd print buffer

 memset(lcd_print_buffer, 0, SIZE_OF_LCD_BUF);

 ultoa(val, lcd_print_buffer, 10); 
 lcd.print(lcd_print_buffer);

 return;
}