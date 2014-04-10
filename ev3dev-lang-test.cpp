/*
 * test program for the ev3dev C++ binding
 *
 * Copyright (c) 2014 - Franz Detro
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "ev3dev.h"

#include <iostream>


using namespace std;
using namespace ev3dev;


template <class S>
void test(const char *name)
{
  S s;
  if (s.connected())
  {
    cout << endl
         << "Found " << name << " sensor with mode " << s.mode()
         << " on port " << s.port() << endl
         << "  Current value is " << s.value() << endl << endl;
  }
  else
    cout << "No " << name << " sensor found" << endl;
}

int main()
{

  test<touch_sensor>("touch");
  test<color_sensor>("color");
  test<ultrasonic_sensor>("ultrasonic");
  test<gyro_sensor>("gyro");
  test<infrared_sensor>("infrared");
  
  cout << "Level of left green led is " << led::green_left.level() << endl;
  cout << "Trigger of right red led is " << led::red_right.trigger() << endl << endl;
  
  cout << "Beeping..." << endl << endl; sound::beep();
  
  cout << "Battery voltage is " << battery::voltage() << " V" << endl;
  cout << "Battery current is " << battery::current() << " mA" <<  endl;
  
  cout << endl;
  
  return 0;
}
