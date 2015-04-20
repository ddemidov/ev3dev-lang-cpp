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
    // Sections of the following code were auto-generated based on spec v0.9.2-pre, rev 3. 
//~autogen
//-----------------------------------------------------------------------------

#include "ev3dev.h"

#include <iostream>
#include <vector>

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
         << "  Current properties are:" << endl
//~autogen cpp_generic_report_status classes.sensor>currentClass

         << "    Commands: " << dev.commands() << endl
         << "    Decimals: " << dev.decimals() << endl
         << "    Driver Name: " << dev.driver_name() << endl
         << "    Mode: " << dev.mode() << endl
         << "    Modes: " << dev.modes() << endl
         << "    Num Values: " << dev.num_values() << endl
         << "    Port Name: " << dev.port_name() << endl
         << "    Units: " << dev.units() << endl


//~autogen
         << endl;
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
         << "  Current properties are:" << endl
//~autogen cpp_generic_report_status classes.motor>currentClass

         << "    Commands: " << dev.commands() << endl
         << "    Count Per Rot: " << dev.count_per_rot() << endl
         << "    Driver Name: " << dev.driver_name() << endl
         << "    Duty Cycle: " << dev.duty_cycle() << endl
         << "    Duty Cycle SP: " << dev.duty_cycle_sp() << endl
         << "    Encoder Polarity: " << dev.encoder_polarity() << endl
         << "    Polarity: " << dev.polarity() << endl
         << "    Port Name: " << dev.port_name() << endl
         << "    Position: " << dev.position() << endl
         /* These are broken, see https://github.com/ev3dev/ev3dev/issues/314
         << "    Position P: " << dev.position_p() << endl
         << "    Position I: " << dev.position_i() << endl
         << "    Position D: " << dev.position_d() << endl
         */
         << "    Position SP: " << dev.position_sp() << endl
         << "    Speed: " << dev.speed() << endl
         << "    Speed SP: " << dev.speed_sp() << endl
         << "    Ramp Up SP: " << dev.ramp_up_sp() << endl
         << "    Ramp Down SP: " << dev.ramp_down_sp() << endl
         << "    Speed Regulation Enabled: " << dev.speed_regulation_enabled() << endl
         << "    Speed Regulation P: " << dev.speed_regulation_p() << endl
         << "    Speed Regulation I: " << dev.speed_regulation_i() << endl
         << "    Speed Regulation D: " << dev.speed_regulation_d() << endl
         << "    State: " << dev.state() << endl
         << "    Stop Command: " << dev.stop_command() << endl
         << "    Stop Commands: " << dev.stop_commands() << endl
         << "    Time SP: " << dev.time_sp() << endl


//~autogen
         << endl;

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
         << "  Current properties are:" << endl
//~autogen cpp_generic_report_status classes.dcMotor>currentClass

         << "    Command: " << dev.command() << endl
         << "    Commands: " << dev.commands() << endl
         << "    Driver Name: " << dev.driver_name() << endl
         << "    Duty Cycle: " << dev.duty_cycle() << endl
         << "    Duty Cycle SP: " << dev.duty_cycle_sp() << endl
         << "    Polarity: " << dev.polarity() << endl
         << "    Port Name: " << dev.port_name() << endl
         << "    Ramp Down MS: " << dev.ramp_down_ms() << endl
         << "    Ramp Up MS: " << dev.ramp_up_ms() << endl


//~autogen
         << endl;
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
         << "  Current properties are:" << endl
//~autogen cpp_generic_report_status classes.servoMotor>currentClass

         << "    Command: " << dev.command() << endl
         << "    Driver Name: " << dev.driver_name() << endl
         << "    Max Pulse MS: " << dev.max_pulse_ms() << endl
         << "    Mid Pulse MS: " << dev.mid_pulse_ms() << endl
         << "    Min Pulse MS: " << dev.min_pulse_ms() << endl
         << "    Polarity: " << dev.polarity() << endl
         << "    Port Name: " << dev.port_name() << endl
         << "    Position: " << dev.position() << endl
         << "    Rate: " << dev.rate() << endl


//~autogen
         << endl;
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
