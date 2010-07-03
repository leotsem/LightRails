/*

   -- Camera Control Functions

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

void fire_camera() {

  digitalWrite(CAMERA_PIN, HIGH);
    // start timer to stop camera exposure
  MsTimer2::set(cur_exp_tm, stop_camera);
  MsTimer2::start();

    // update camera currently enaged
  camera_engaged = true;

  return;
}

void stop_camera() {

  digitalWrite(CAMERA_PIN, LOW);
    // turn off timer
  MsTimer2::stop();

    // update camera currently enaged
  camera_engaged = false;
}