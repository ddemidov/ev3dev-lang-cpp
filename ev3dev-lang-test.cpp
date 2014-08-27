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
void test_sensor(const char *name)
{
  S s;
  if (s.connected())
  {
    cout << endl
         << "Found " << name << " sensor with mode " << s.mode()
         << " on port " << s.port_name() << endl
         << "  Current value is " << s.value() << endl << endl;
  }
  else
    cout << "No " << name << " sensor found" << endl;
}

template <class M>
void test_motor(const char *name)
{
  M m;
  if (m.connected())
  {
    cout << endl
         << "Found " << name << " motor on port " << m.port_name() << endl
         << endl;
    
    cout << "  Current state is " << m.state() << endl
         << "    duty_cycle: " << m.duty_cycle() << endl
         << "    pulses_per_second: " << m.pulses_per_second() << endl << endl
         << "  Current run mode is " << m.run_mode() << endl
         << "    stop mode: " << m.stop_mode() << endl
         << "    regulation mode: " << m.regulation_mode() << endl << endl;
    
    if (m.regulation_mode()==m.mode_on)
      cout << "  pulses_per_second setpoint is " << m.pulses_per_second_setpoint() << endl;
    else
      cout << "  duty_cycle setpoint is " << m.duty_cycle_setpoint() << endl;
    
    if (m.run_mode()==m.run_mode_time)
      cout << "  Time setpoint is " << m.time_setpoint() << endl;
    if (m.run_mode()==m.run_mode_position)
      cout << "  Position setpoint is " << m.position_setpoint() << endl;
    cout << "    ramp up: " << m.ramp_up()  << "   ramp down: " << m.ramp_down() << endl<< endl;
  }
  else
    cout << "No " << name << " motor found" << endl;
}


int main()
{

  test_sensor<touch_sensor>("touch");
  test_sensor<color_sensor>("color");
  test_sensor<ultrasonic_sensor>("ultrasonic");
  test_sensor<gyro_sensor>("gyro");
  test_sensor<infrared_sensor>("infrared");
  
  test_motor<medium_motor>("medium");
  test_motor<medium_motor>("large");
  
  cout << "Level of left green led is " << led::green_left.level() << endl;
  cout << "Trigger of right red led is " << led::red_right.trigger() << endl << endl;
  
  cout << "Beeping..." << endl << endl; sound::beep();
  
  cout << "Battery voltage is " << battery::voltage() << " V" << endl;
  cout << "Battery current is " << battery::current() << " mA" <<  endl;
  
  cout << endl;
  
  return 0;
}
