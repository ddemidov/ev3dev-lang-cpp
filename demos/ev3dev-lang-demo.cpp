/*
 * demo program for the ev3dev C++ binding
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
#include <thread>
#include <string.h>
#include <errno.h>

using namespace std;
using namespace ev3dev;

std::ostream& operator<<(std::ostream &os, const std::set<std::string> &ss) {
  os << "[ ";
  for(const auto &s : ss) os << s << " ";
  return os << "]";
}


void print_values(sensor &s)
{
  auto dp = s.decimals();
  unsigned m = s.num_values();
  for (unsigned i=0; i<m; ++i)
  {
    if (i) cout << "; ";
    if (dp)
      cout << s.float_value(i);
    else
      cout << s.value(i);
  }
  cout << " " << s.units();
}

void sensor_action(sensor &s)
{
  char c = 0;
  do
  {
    cout << endl
         << "*** " << s.type_name() << " (" << s.mode() << ") actions ***" << endl
         << endl
         << "(s)how modes"  << endl
         << "(c)hange mode" << endl
         << "(v)alue"       << endl
         << "(m)onitor"     << endl
         << endl
         << "(b)ack"        << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 's':
      {
        cout << "available modes are ";
        const mode_set &m = s.modes();
        for (mode_set::const_iterator it = m.begin(); it!=m.end(); ++it)
        {
          cout << *it << " ";
        }
        cout << endl;
      }
      break;
    case 'c':
      {
        string mode;
        cout << endl << "new mode: "; cin >> mode; cout << endl;
        s.set_mode(mode);
      }
      break;
    case 'v':
      cout << endl << "value is "; print_values(s); cout << endl;
      break;
    case 'm':
      {
        bool bStop = false;
        thread t([&] () {
          char cc; cin >> cc;
          bStop = true;
        });

        int value, lastValue = -99999;
        while (!bStop)
        {
          value = s.value();
          if (value != lastValue)
          {
            lastValue = value;
            print_values(s);
            cout << endl;
          }
          this_thread::sleep_for(chrono::milliseconds(100));
        }
        t.join();
      }
      break;
    }
  }
  while (c != 'b');
}

void sensor_menu()
{
  sensor arrSensors[4] {
    { INPUT_1 },
    { INPUT_2 },
    { INPUT_3 },
    { INPUT_4 }
  };

  char c = 0;
  do
  {
    cout << endl
         << "*** sensor menu ***" << endl
         << endl;

    for (unsigned i=0; i<4; ++i)
    {
      sensor &s = arrSensors[i];
      if (s.connected())
      {
        cout << "(" << i+1 << ") " << s.type_name() << " (device " << s.driver_name()
             << ", port " << s.address() << ", mode " << s.mode() << ")" << endl;
      }
    }
    cout << endl;
    cout << "(b)ack"  << endl;
    cout << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case '1':
    case '2':
    case '3':
    case '4':
      sensor_action(arrSensors[c-'1']);
      break;
    }
  }
  while (c != 'b');
}

void motor_action(motor &dev)
{
  char c = 0;
  int new_value = 0;
  std::string answer;
  bool running = false;

  do
  {
    cout << endl
         << "*** " << dev.driver_name() << " motor (" << dev.address() << ") actions ***" << endl
         << endl
         << "(i)nfo" << endl
         << "(c)ommand" << endl
         << "st(o)p command      [" << dev.stop_command()             << "]" << endl
         << "speed r(e)gulation  [" << dev.speed_regulation_enabled() << "]" << endl;

    if (dev.speed_regulation_enabled()==dev.speed_regulation_on)
      cout << "speed (s)etpoint (" << dev.speed_sp() << ")" << endl;
    else
      cout << "duty cycle (s)etpoint (" << dev.duty_cycle_sp() << ")" << endl;

    cout << "(p)osition setpoint  (" << dev.position_sp()   << ")" << endl
         << "ramp (u)p setpoint   (" << dev.ramp_up_sp()    << ")" << endl
         << "ramp (d)own setpoint (" << dev.ramp_down_sp()  << ")" << endl
         << "(t)ime setpoint      (" << dev.time_sp()       << ")" << endl
         << endl
         << "(0) reset position" << endl
         << endl
         << "(b)ack" << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 'i':
      cout << endl;
//~autogen generic_report_status classes.motor>currentClass

    cout << "    Commands: ";
    try { cout << dev.commands() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Count Per Rot: ";
    try { cout << dev.count_per_rot() << endl; }
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
    cout << "    Encoder Polarity: ";
    try { cout << dev.encoder_polarity() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Polarity: ";
    try { cout << dev.polarity() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Address: ";
    try { cout << dev.address() << endl; }
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
    cout << "    Speed Regulation Enabled: ";
    try { cout << dev.speed_regulation_enabled() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed Regulation P: ";
    try { cout << dev.speed_regulation_p() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed Regulation I: ";
    try { cout << dev.speed_regulation_i() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Speed Regulation D: ";
    try { cout << dev.speed_regulation_d() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    State: ";
    try { cout << dev.state() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Stop Command: ";
    try { cout << dev.stop_command() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Stop Commands: ";
    try { cout << dev.stop_commands() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }
    cout << "    Time SP: ";
    try { cout << dev.time_sp() << endl; }
    catch(...) { cout << "[" << strerror(errno) << "]" << endl; }


//~autogen
      cout << endl;
      break;
    case 'c':
      cout << "command " << dev.commands() << ": ";
      cin >> answer; dev.set_command(answer); cout << endl;
      break;
    case 'o':
      cout << "stop command " << dev.stop_commands() << ": ";
      cin >> answer; dev.set_stop_command(answer); cout << endl;
      break;
    case 'e':
      cout << "speed regulation (off, on): ";
      cin >> answer; dev.set_speed_regulation_enabled(answer); cout << endl;
      break;
    case 's':
      if (dev.speed_regulation_enabled()==dev.speed_regulation_on)
      {
        cout << "speed: "; cin >> new_value; dev.set_speed_sp(new_value); cout << endl;
      }
      else
      {
        cout << "duty cycle: "; cin >> new_value; dev.set_duty_cycle_sp(new_value); cout << endl;
      }
      break;
    case 'p':
      cout << "position: "; cin >> new_value; dev.set_position_sp(new_value); cout << endl;
      break;
    case 'u':
      cout << "ramp up: "; cin >> new_value; dev.set_ramp_up_sp(new_value); cout << endl;
      break;
    case 'd':
      cout << "ramp down: "; cin >> new_value; dev.set_ramp_down_sp(new_value); cout << endl;
      break;
    case 't':
      cout << "time: "; cin >> new_value; dev.set_time_sp(new_value); cout << endl;
      break;
    case '0':
      dev.set_position(0);
      break;
    }
  }
  while (c != 'b');
}

void motor_action(dc_motor &m)
{
  char c = 0;
  int new_value = 0;
  std::string new_mode;

  do
  {
    cout << endl
         << "*** dc motor (" << m.address() << ") actions ***" << endl
         << endl
         << "(d)uty cycle sp(" << m.duty_cycle_sp() << ")" << endl
         << "(r)amp down sp (" << m.ramp_down_sp() << ")" << endl
         << "ramp (u)p sp   (" << m.ramp_up_sp()   << ")" << endl
         << "p(o)larity     [" << m.polarity()     << "]" << endl;
    cout << endl << "(b)ack" << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 'c':
      cout << "command " << m.commands() << ": ";
      cin >> new_mode; m.set_command(new_mode); cout << endl;
      break;
    case 'o':
      cout << "polarity (normal, inverted): ";
      cin >> new_mode; m.set_polarity(new_mode); cout << endl;
      break;
    case 'd':
      cout << "duty cycle sp: "; cin >> new_value; m.set_duty_cycle_sp(new_value); cout << endl;
      break;
    case 'r':
      cout << "ramp down sp: "; cin >> new_value; m.set_ramp_down_sp(new_value); cout << endl;
      break;
    case 'u':
      cout << "ramp up sp: "; cin >> new_value; m.set_ramp_up_sp(new_value); cout << endl;
      break;
    }
  }
  while (c != 'b');
}

void motor_action(servo_motor &m)
{
  char c = 0;
  int new_value = 0;
  std::string new_mode;

  do
  {
    cout << endl
         << "*** servo motor (" << m.address() << ") actions ***" << endl
         << endl
         << "(c)ommand" << endl
         << "(p)osition_sp  (" << m.position_sp()  << ")" << endl
         << "(r)ate_sp      (" << m.rate_sp()      << ")" << endl
         << "mi(n) pulse sp (" << m.min_pulse_sp() << ")" << endl
         << "mi(d) pulse sp (" << m.mid_pulse_sp() << ")" << endl
         << "ma(x) pulse sp (" << m.max_pulse_sp() << ")" << endl
         << "p(o)larity     [" << m.polarity()     << "]" << endl;
    cout << endl << "(b)ack" << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 'c':
      cout << "command (run, float): ";
      cin >> new_mode; m.set_command(new_mode); cout << endl;
      break;
    case 'p':
      cout << "position sp: "; cin >> new_value; m.set_position_sp(new_value); cout << endl;
      break;
    case 'o':
      cout << "polarity (normal, inverted): ";
      cin >> new_mode; m.set_polarity(new_mode); cout << endl;
      break;
    case 'r':
      cout << "rate sp: "; cin >> new_value; m.set_rate_sp(new_value); cout << endl;
      break;
    case 'n':
      cout << "min pulse sp: "; cin >> new_value; m.set_min_pulse_sp(new_value); cout << endl;
      break;
    case 'd':
      cout << "mid pulse sp: "; cin >> new_value; m.set_mid_pulse_sp(new_value); cout << endl;
      break;
    case 'x':
      cout << "max pulse sp: "; cin >> new_value; m.set_max_pulse_sp(new_value); cout << endl;
      break;
    }
  }
  while (c != 'b');
}

void motor_menu()
{
  motor arrMotors[4] = {
    { OUTPUT_A },
    { OUTPUT_B },
    { OUTPUT_C },
    { OUTPUT_D }
  };
  dc_motor arrDCMotors[4] = {
    { OUTPUT_A },
    { OUTPUT_B },
    { OUTPUT_C },
    { OUTPUT_D }
  };
  servo_motor arrServoMotors[4] = {
    { OUTPUT_A },
    { OUTPUT_B },
    { OUTPUT_C },
    { OUTPUT_D }
  };

  char c = 0;
  do
  {
    cout << endl
         << "*** motor menu ***" << endl
         << endl;

    for (unsigned i=0; i<4; ++i)
    {
      motor &m = arrMotors[i];
      if (m.connected())
      {
        cout << "(" << 1+i << ") " << m.driver_name() << " motor on port " << m.address() << endl;
      }
      else if (arrDCMotors[i].connected())
      {
        cout << "(" << 1+i << ") dc motor on port " << arrDCMotors[i].address() << endl;
      }
      else if (arrServoMotors[i].connected())
      {
        cout << "(" << 1+i << ") servo motor on port " << arrServoMotors[i].address() << endl;
      }
    }
    cout << endl;
    cout << "(b)ack"  << endl;
    cout << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case '1':
    case '2':
    case '3':
    case '4':
      if (arrMotors[c-'1'].connected())
        motor_action(arrMotors[c-'1']);
      else if (arrMotors[c-'1'].connected())
        motor_action(arrDCMotors[c-'1']);
      else if (arrServoMotors[c-'1'].connected())
        motor_action(arrServoMotors[c-'1']);
      break;
    }
  }
  while (c != 'b');
}

void led_action(const char *name, led &l)
{
  int interval = 500;
  char c = 0;
  do
  {
    cout << endl
         << "*** " << name << " actions ***" << endl
         << endl
         << "(0) off"    << endl
         << "(1) on"     << endl
         << "(f)lash"    << endl
         << "(i)nterval" << endl
         << "(t)rigger"  << endl
         << endl
         << "(b)ack"     << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case '0':
      l.off();
      break;
    case '1':
      l.on();
      break;
    case 'f':
      l.flash(200, 200);
      break;
    case 'i':
      cout << "interval: "; cin >> interval; cout << endl;
      l.set_delay_on(interval); l.set_delay_off(interval);
      break;
    case 't':
      {
        cout << "available triggers are " << endl;
        mode_type t = l.trigger();
        mode_set  s = l.triggers();
        for (mode_set::const_iterator it = s.begin(); it!=s.end(); ++it)
        {
          if (*it == t)
            cout << "[" << *it << "] ";
          else
            cout << *it << " ";
        }
        cout << endl << endl << "choice: ";

        cin >> t;
        if (!t.empty())
          l.set_trigger(t);
      }
      break;
    }
  }
  while (c != 'b');
}

void led_menu()
{
  led arrPortLEDs[4] {
    { "ev3::outA" }, { "ev3::outB" }, { "ev3::outC" }, { "ev3::outD" } };

  char c = 0;
  do
  {
    cout << endl
         << "*** led menu ***" << endl
         << endl
         << "(1) green:left"  << endl
         << "(2) green:right" << endl
         << "(3) red:left"    << endl
         << "(4) red:right"   << endl;

    for (unsigned i=0; i<4; ++i)
    {
      if (arrPortLEDs[i].connected())
      {
        cout << "(" << 5+i << ") out" << static_cast<char>('A'+i) << endl;
      }
    }

    cout << endl
         << "(b)ack"          << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case '1':
      led_action("green:left", led::green_left);
      break;
    case '2':
      led_action("green:right", led::green_right);
      break;
    case '3':
      led_action("red:left", led::red_left);
      break;
    case '4':
      led_action("red:right", led::red_right);
      break;
    case '5':
      led_action("outA", arrPortLEDs[0]);
      break;
    case '6':
      led_action("outB", arrPortLEDs[1]);
      break;
    case '7':
      led_action("outC", arrPortLEDs[2]);
      break;
    case '8':
      led_action("outD", arrPortLEDs[3]);
      break;
    }
  }
  while (c != 'b');
}

void button_menu()
{
  char c = 0;
  do
  {
    cout << endl
         << "*** button menu ***" << endl
         << endl
         << "(1) back"  << endl
         << "(2) left"  << endl
         << "(3) right" << endl
         << "(4) up"    << endl
         << "(5) down"  << endl
         << "(6) enter" << endl
         << endl
         << "(b)ack"    << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case '1':
      cout << endl << "back button is " << (button::back.pressed() ? "down" : "up") << endl;
      break;
    case '2':
      cout << endl << "left button is " << (button::left.pressed() ? "down" : "up") << endl;
      break;
    case '3':
      cout << endl << "right button is " << (button::right.pressed() ? "down" : "up") << endl;
      break;
    case '4':
      cout << endl << "up button is " << (button::up.pressed() ? "down" : "up") << endl;
      break;
    case '5':
      cout << endl << "down button is " << (button::down.pressed() ? "down" : "up") << endl;
      break;
    case '6':
      cout << endl << "enter button is " << (button::enter.pressed() ? "down" : "up") << endl;
      break;
    }
  }
  while (c != 'b');
}

void sound_menu()
{
  unsigned frequency = 440; // 440 Hz
  unsigned duration = 1000; // 1 sec
  char c = 0;
  do
  {
    cout << endl
         << "*** sound menu ***" << endl
         << endl
         << "b(e)ep"          << endl
         << "(t)one"          << endl
         << "tone (d)uration" << endl
         << "(p)lay"          << endl
         << "(s)peak"         << endl
         << "(v)olume"        << endl
         << "(b)ack"          << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 'e':
      sound::beep();
      break;
    case 't':
      cout << "frequency: "; cin >> frequency; cout << endl;
      sound::tone(frequency, duration);
      break;
    case 'd':
      cout << "duration: "; cin >> duration; cout << endl;
      sound::tone(frequency, duration);
      break;
    case 'p':
      {
        std::string fname;
        cout << "soundfile: "; cin >> fname; cout << endl;
        sound::play(fname, true);
      }
      break;
    case 's':
      {
        std::string text;
        cout << "text: "; getline(cin, text); getline(cin, text); cout << endl;
        sound::speak(text, true);
      }
      break;
    }
  }
  while (c != 'b');
}

void battery_menu()
{
  char c = 0;
  do
  {
    cout << endl
         << "*** battery menu ***" << endl
         << endl
         << "(v)oltage" << endl
         << "(c)urrent" << endl
         << endl
         << "(b)ack"  << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 'v':
      cout << endl << "voltage is " << power_supply::battery.measured_volts() << " Volt" << endl << endl;
      break;
    case 'c':
      cout << endl << "current is " << power_supply::battery.measured_amps() << " A" << endl << endl;
      break;
    }
  }
  while (c != 'b');
}

void lcd_menu()
{
  lcd l;

  if (!l.available())
  {
    cout << endl
         << "###error: lcd not available ###" << endl;
    return;
  }

  char c = 0;
  do
  {
    cout << endl
         << "*** lcd menu ***" << endl
         << endl
         << "(i)nfo" << endl
         << "(f)ill" << endl
         << "(c)lear" << endl
         << endl
         << "(b)ack"  << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 'i':
      cout << endl
           << "Resolution is " << l.resolution_x() << " x " << l.resolution_y()
           << ", " << l.bits_per_pixel() << " bit(s) per pixel" << endl
           << "Frame buffer size is " << l.frame_buffer_size() << " byte, "
           << "line length is " << l.line_length() << " byte" << endl;
    case 'f':
      l.fill(0xFF);
      break;
    case 'c':
      l.fill(0);
      break;
    }
  }
  while (c != 'b');
}

void main_menu()
{
  char c = 0;
  do
  {
    cout << endl
         << "*** main menu ***" << endl
         << endl
         << "(s)ensors" << endl
         << "(m)otors" << endl
         << "(l)eds" << endl
         << "(b)uttons" << endl
         << "s(o)und" << endl
         << "b(a)ttery" << endl
         << "l(c)d" << endl
         << "(q)uit" << endl
         << endl
         << "Choice: ";
    cin >> c;

    switch (c)
    {
    case 's':
      sensor_menu();
      break;
    case 'm':
      motor_menu();
      break;
    case 'l':
      led_menu();
      break;
    case 'b':
      button_menu();
      break;
    case 'o':
      sound_menu();
      break;
    case 'a':
      battery_menu();
      break;
    case 'c':
      lcd_menu();
      break;
    }
  }
  while (c != 'q');
}

int main()
{
  main_menu();

  return 0;
}
