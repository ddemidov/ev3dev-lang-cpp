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

using namespace std;
using namespace ev3dev;

void print_values(sensor &s)
{
  auto dp = s.dp();
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
        cout << "(" << i+1 << ") " << s.type_name() << " (type " << s.type()
             << ", port " << s.port_name() << ", mode " << s.mode() << ")" << endl;
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

void motor_action(motor &m)
{
  char c = 0;
  int new_value = 0;
  std::string new_mode;
  bool running = false;
  
  do
  {
    cout << endl
         << "*** " << m.type() << " motor (" << m.port_name() << ") actions ***" << endl
         << endl
         << "(i)nfo" << endl
         << "(r)un mode          [" << m.run_mode()          << "]" << endl
         << "st(o)p mode         [" << m.stop_mode()         << "]" << endl
         << "r(e)gulation mode   [" << m.regulation_mode()   << "]" << endl;
    
    if (m.regulation_mode()==m.mode_on)
      cout << "pulses/sec (s)etpoint (" << m.pulses_per_second_setpoint() << ")" << endl;
    else
      cout << "duty cycle (s)etpoint (" << m.duty_cycle_setpoint() << ")" << endl;

    if (m.run_mode()==m.run_mode_position)
    {
      cout << "position (m)ode     [" << m.position_mode()     << "]" << endl
           << "(p)osition setpoint (" << m.position_setpoint() << ")" << endl
           << "ramp (u)p           (" << m.ramp_up()           << ")" << endl
           << "ramp (d)own         (" << m.ramp_down()         << ")" << endl;
    }
    else if (m.run_mode()==m.run_mode_time)
      cout << "(t)ime setpoint     (" << m.time_setpoint()     << ")" << endl;
    
    cout << endl
         << "(0) reset position" << endl
         << "(#) reset all" << endl
         << "(!) ";
    running = m.running();
    if (running)
      cout << "STOP" << endl;
    else
      cout << "RUN" << endl;
    cout << endl << "(b)ack" << endl
         << endl
         << "Choice: ";
    cin >> c;
    
    switch (c)
    {
    case 'i':
      cout << endl
           << "  state      is " << m.state() << endl
           << "  duty cycle is " << m.duty_cycle() << endl
           << "  pulses/sec is " << m.pulses_per_second() << endl
           << "  position   is " << m.position() << endl;
      break;
    case 'r':
      cout << "run mode (forever, position, time): ";
      cin >> new_mode; m.set_run_mode(new_mode); cout << endl;
      break;
    case 'o':
      cout << "stop mode (coast, brake, hold): ";
      cin >> new_mode; m.set_stop_mode(new_mode); cout << endl;
      break;
    case 'e':
      cout << "regulation mode (off, on): ";
      cin >> new_mode; m.set_regulation_mode(new_mode); cout << endl;
      break;
    case 's':
      if (m.regulation_mode()==m.mode_on)
      {
        cout << "pulses/sec: "; cin >> new_value; m.set_pulses_per_second_setpoint(new_value); cout << endl;
      }
      else
      {
        cout << "duty cycle: "; cin >> new_value; m.set_duty_cycle_setpoint(new_value); cout << endl;
      }
      break;
    case 'm':
      if (m.run_mode()==m.run_mode_position)
      {
        cout << "position mode (absolute, relative): ";
        cin >> new_mode; m.set_position_mode(new_mode); cout << endl;
      }
      break;
    case 'p':
      if (m.run_mode()==m.run_mode_position)
      {
        cout << "position: "; cin >> new_value; m.set_position_setpoint(new_value); cout << endl;
      }
      break;
    case 'u':
      if (m.run_mode()==m.run_mode_position)
      {
        cout << "ramp up: "; cin >> new_value; m.set_ramp_up(new_value); cout << endl;
      }
      break;
    case 'd':
      if (m.run_mode()==m.run_mode_position)
      {
        cout << "ramp down: "; cin >> new_value; m.set_ramp_down(new_value); cout << endl;
      }
      break;
    case 't':
      if (m.run_mode()==m.run_mode_time)
      {
        cout << "time: "; cin >> new_value; m.set_time_setpoint(new_value); cout << endl;
      }
      break;
    case '0':
      m.set_position(0);
      break;
    case '#':
      m.reset();
      break;
    case '!':
      m.run(!running);
      break;
    }
  }
  while (c != 'b');
}

void motor_menu()
{
  motor arrMotors[4] = {
    motor(OUTPUT_A),
    motor(OUTPUT_B),
    motor(OUTPUT_C),
    motor(OUTPUT_D)
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
        cout << "(" << i+1 << ") " << m.type() << " motor on port " << m.port_name() << endl;
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
      motor_action(arrMotors[c-'1']);
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
      l.flash(0);
      break;
    case 'i':
      cout << "interval: "; cin >> interval; cout << endl;
      l.set_on_delay(interval); l.set_off_delay(interval);
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
  char c = 0;
  do
  {
    cout << endl
         << "*** led menu ***" << endl
         << endl
         << "(1) green:left"  << endl
         << "(2) green:right" << endl
         << "(3) red:left"    << endl
         << "(4) red:right"   << endl
         << endl
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
    case 'v':
      {
        cout << endl << "current volume is " << sound::volume()
             << endl << "new volume: ";

        unsigned volume = 0;
        cin >> volume; cout << endl;
        if (volume)
          sound::set_volume(volume);
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
      cout << endl << "voltage is " << power_supply::battery.voltage_volts() << " Volt" << endl << endl;
      break;
    case 'c':
      cout << endl << "current is " << power_supply::battery.current_amps() << " A" << endl << endl;
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
