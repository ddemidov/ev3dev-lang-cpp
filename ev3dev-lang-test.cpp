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
#include <vector>

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
      cout << "  pulses_per_second setpoint is " << m.pulses_per_second_sp() << endl;
    else
      cout << "  duty_cycle setpoint is " << m.duty_cycle_sp() << endl;

    if (m.run_mode()==m.run_mode_time)
      cout << "  Time setpoint is " << m.time_sp() << endl;
    if (m.run_mode()==m.run_mode_position)
      cout << "  Position setpoint is " << m.position_sp() << endl;
    cout << "    ramp up: " << m.ramp_up_sp()  << "   ramp down: " << m.ramp_down_sp() << endl << endl;
  }
  else
    cout << "No " << name << " motor found" << endl;
}

void test_dc_motor()
{
  dc_motor m;
  if (m.connected())
  {
    cout << endl
         << "Found dc motor on port " << m.port_name() << endl
         << endl;

    cout << "  Current parameters" << endl
         << "    duty_cycle:   " << m.duty_cycle() << endl
         << "    ramp_up_ms:   " << m.ramp_up_ms() << endl << endl
         << "    ramp_down_ms: " << m.ramp_down_ms() << endl << endl
         << "    polarity:     " << m.polarity() << endl << endl;
  }
  else
    cout << "No dc motor found" << endl;
}

void test_servo_motor()
{
  servo_motor m;
  if (m.connected())
  {
    cout << endl
         << "Found servo motor on port " << m.port_name() << endl
         << endl;

    cout << "  Current command is " << m.command() << endl
         << "    position:     " << m.position() << endl
         << "    rate:         " << m.rate() << endl
         << "    min_pulse_ms: " << m.min_pulse_ms() << endl << endl
         << "    mid_pulse_ms: " << m.mid_pulse_ms() << endl << endl
         << "    max_pulse_ms: " << m.max_pulse_ms() << endl << endl
         << "    polarity:     " << m.polarity() << endl << endl;
  }
  else
    cout << "No servo motor found" << endl;
}

int main()
{

  test_sensor<touch_sensor>("touch");
  test_sensor<color_sensor>("color");
  test_sensor<ultrasonic_sensor>("ultrasonic");
  test_sensor<gyro_sensor>("gyro");
  test_sensor<infrared_sensor>("infrared");
  test_sensor<i2c_sensor>("i2c");

  cout << endl;

  test_motor<medium_motor>("medium");
  test_motor<large_motor>("large");

  test_dc_motor();
  test_servo_motor();

  const vector<string> outPortLEDs {
    "ev3::outA",
    "ev3::outB",
    "ev3::outC",
    "ev3::outD"
  };
  for (auto const &portName : outPortLEDs)
  {
    led l { portName };
    if (l.connected())
    {
      cout << "Brightness of " << portName << " led is " << l.brightness() << endl;
      cout << "Trigger of " << portName << " led is " << l.trigger() << endl << endl;
    }
  }

  cout << "Brightness of left green led is " << led::green_left.brightness() << endl;
  cout << "Trigger of right red led is " << led::red_right.trigger() << endl << endl;

  cout << "Beeping..." << endl << endl; sound::beep();

  cout << "Battery voltage is " << power_supply::battery.voltage_volts() << " V" << endl;
  cout << "Battery current is " << power_supply::battery.current_amps() << " A" <<  endl;

  cout << endl;

  return 0;
}
