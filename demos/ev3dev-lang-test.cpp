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

//-----------------------------------------------------------------------------
//~autogen autogen-header

// Sections of the following code were auto-generated based on spec v1.2.0.

//~autogen
//-----------------------------------------------------------------------------

#include "ev3dev.h"

#include <iostream>
#include <vector>
#include <string.h>
#include <errno.h>

using namespace std;
using namespace ev3dev;


std::ostream& operator<<(std::ostream &os, const std::set<std::string> &ss) {
  os << "[ ";
  for(const auto &s : ss) os << s << " ";
  return os << "]";
}

template <class S>
void test_sensor(const char *name)
{
  S dev;
  if (dev.connected())
  {
    cout << endl
         << "Found " << name << " sensor" << endl
         << "  Current properties are:" << endl;
//~autogen generic_report_status classes.sensor>currentClass

    cout << "    Address: ";
    try { cout << dev.address() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Commands: ";
    try { cout << dev.commands() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Decimals: ";
    try { cout << dev.decimals() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Driver Name: ";
    try { cout << dev.driver_name() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Mode: ";
    try { cout << dev.mode() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Modes: ";
    try { cout << dev.modes() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Num Values: ";
    try { cout << dev.num_values() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Units: ";
    try { cout << dev.units() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }


//~autogen
    cout << endl;
  }
  else
    cout << "No " << name << " sensor found" << endl;
}

template <class M>
void test_motor(const char *name)
{
  M dev;
  if (dev.connected())
  {
    cout << endl
         << "Found " << name << " motor" << endl
         << "  Current properties are:" << endl;
//~autogen generic_report_status classes.motor>currentClass

    cout << "    Address: ";
    try { cout << dev.address() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Commands: ";
    try { cout << dev.commands() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Count Per Rot: ";
    try { cout << dev.count_per_rot() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Count Per M: ";
    try { cout << dev.count_per_m() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Driver Name: ";
    try { cout << dev.driver_name() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Duty Cycle: ";
    try { cout << dev.duty_cycle() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Duty Cycle SP: ";
    try { cout << dev.duty_cycle_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Full Travel Count: ";
    try { cout << dev.full_travel_count() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Polarity: ";
    try { cout << dev.polarity() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Position: ";
    try { cout << dev.position() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Position P: ";
    try { cout << dev.position_p() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Position I: ";
    try { cout << dev.position_i() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Position D: ";
    try { cout << dev.position_d() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Position SP: ";
    try { cout << dev.position_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Max Speed: ";
    try { cout << dev.max_speed() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed: ";
    try { cout << dev.speed() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed SP: ";
    try { cout << dev.speed_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Ramp Up SP: ";
    try { cout << dev.ramp_up_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Ramp Down SP: ";
    try { cout << dev.ramp_down_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed P: ";
    try { cout << dev.speed_p() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed I: ";
    try { cout << dev.speed_i() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed D: ";
    try { cout << dev.speed_d() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    State: ";
    try { cout << dev.state() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Stop Action: ";
    try { cout << dev.stop_action() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Stop Actions: ";
    try { cout << dev.stop_actions() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Time SP: ";
    try { cout << dev.time_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }


//~autogen
    cout << endl;
  }
  else
    cout << "No " << name << " motor found" << endl;
}

void test_dc_motor()
{
  dc_motor dev;
  if (dev.connected())
  {
    cout << endl
         << "Found dc motor" << endl
         << "  Current properties are:" << endl;
//~autogen generic_report_status classes.dcMotor>currentClass

    cout << "    Address: ";
    try { cout << dev.address() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Commands: ";
    try { cout << dev.commands() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Driver Name: ";
    try { cout << dev.driver_name() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Duty Cycle: ";
    try { cout << dev.duty_cycle() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Duty Cycle SP: ";
    try { cout << dev.duty_cycle_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Polarity: ";
    try { cout << dev.polarity() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Ramp Down SP: ";
    try { cout << dev.ramp_down_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Ramp Up SP: ";
    try { cout << dev.ramp_up_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    State: ";
    try { cout << dev.state() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Stop Actions: ";
    try { cout << dev.stop_actions() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Time SP: ";
    try { cout << dev.time_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }


//~autogen
    cout << endl;
  }
  else
    cout << "No dc motor found" << endl;
}

void test_servo_motor()
{
  servo_motor dev;
  if (dev.connected())
  {
    cout << endl
         << "Found servo motor" << endl
         << "  Current properties are:" << endl;
//~autogen generic_report_status classes.servoMotor>currentClass

    cout << "    Address: ";
    try { cout << dev.address() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Driver Name: ";
    try { cout << dev.driver_name() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Max Pulse SP: ";
    try { cout << dev.max_pulse_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Mid Pulse SP: ";
    try { cout << dev.mid_pulse_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Min Pulse SP: ";
    try { cout << dev.min_pulse_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Polarity: ";
    try { cout << dev.polarity() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Position SP: ";
    try { cout << dev.position_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Rate SP: ";
    try { cout << dev.rate_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    State: ";
    try { cout << dev.state() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }


//~autogen
    cout << endl;
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

  cout << "Battery voltage is " << power_supply::battery.measured_volts() << " V" << endl;
  cout << "Battery current is " << power_supply::battery.measured_amps() << " A" <<  endl;

  cout << endl;

  return 0;
}
