/*
 * Copyright (c) 2014 - Franz Detro
 *
 * Some real world test program for motor control
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

#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>

#ifndef NO_LINUX_HEADERS
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#define KEY_RELEASE 0
#define KEY_PRESS   1
#define KEY_REPEAT  2
#endif

using namespace std;
using namespace ev3dev;

class control
{
public:
  control();
  ~control();

  void drive(int speed, int time=0);
  void turn(int direction);
  void stop();
  void reset();

  bool initialized() const;

  void terminate_on_key();
  void panic_if_touched();

  void remote_loop();
  void drive_autonomously();

  void terminate() { _terminate = true; }

protected:
  large_motor     _motor_left;
  large_motor     _motor_right;
  infrared_sensor _sensor_ir;
  touch_sensor    _sensor_touch;

  enum state
  {
    state_idle,
    state_driving,
    state_turning
  };

  state _state;
  bool  _terminate;
};

control::control() :
  _motor_left(OUTPUT_B),
  _motor_right(OUTPUT_C),
  _state(state_idle),
  _terminate(false)
{
}

control::~control()
{
  reset();
}

void control::drive(int speed, int time)
{
  if (time > 0)
  {
    _motor_left .set_run_mode(motor::run_mode_time);
    _motor_right.set_run_mode(motor::run_mode_time);

    _motor_left .set_time_sp(time);
    _motor_right.set_time_sp(time);
  }
  else
  {
    _motor_left .set_run_mode(motor::run_mode_forever);
    _motor_right.set_run_mode(motor::run_mode_forever);
  }

  _motor_left.set_duty_cycle_sp(-speed);

  _motor_right.set_duty_cycle_sp(-speed);

  _state = state_driving;

  _motor_left .run();
  _motor_right.run();

  if (time > 0)
  {
    while (_motor_left.running() || _motor_right.running())
      this_thread::sleep_for(chrono::milliseconds(10));

    _state = state_idle;
  }
}

void control::turn(int direction)
{
  if (_state != state_idle)
    stop();

  if (direction == 0)
    return;

  _motor_left.set_run_mode(motor::run_mode_position);
  //_motor_left.set_position_mode(motor::position_mode_relative);
  _motor_left.set_position(0);
  _motor_left .set_position_sp(direction);
  //_motor_left.set_regulation_mode(motor::mode_on);
  _motor_left.set_duty_cycle_sp(50);

  _motor_right.set_run_mode(motor::run_mode_position);
  //_motor_right.set_position_mode(motor::position_mode_relative);
  _motor_right.set_position(0);
  _motor_right.set_position_sp(-direction);
  //_motor_right.set_regulation_mode(motor::mode_on);
  _motor_right.set_duty_cycle_sp(50);

  _state = state_turning;

  _motor_left .run();
  _motor_right.run();

  while (_motor_left.running() || _motor_right.running())
    this_thread::sleep_for(chrono::milliseconds(10));

  _state = state_idle;
}

void control::stop()
{
  _motor_left .stop();
  _motor_right.stop();

  _state = state_idle;
}

void control::reset()
{
  if (_motor_left.connected())
    _motor_left .reset();

  if (_motor_right.connected())
    _motor_right.reset();

  _state = state_idle;
}

bool control::initialized() const
{
  return (_motor_left .connected() &&
          _motor_right.connected() &&
          _sensor_ir  .connected());
}

void control::terminate_on_key()
{
 #ifndef NO_LINUX_HEADERS
  thread t([&] () {
    int fd = open("/dev/input/by-path/platform-gpio-keys.0-event", O_RDONLY);
    if (fd  < 0)
    {
      cout << "Couldn't open platform-gpio-keys device!" << endl;
      return;
    }

    input_event ev;
    while (true)
    {
      size_t rb = read(fd, &ev, sizeof(ev));

      if (rb < sizeof(input_event))
        continue;

      if ((ev.type == EV_KEY) /*&& (ev.value == KEY_PRESS)*/)
      {
        terminate();
        return;
      }
    }
  });
  t.detach();
 #endif
}

void control::panic_if_touched()
{
  if (!_sensor_touch.connected())
  {
    cout << "no touch sensor found!" << endl;
    return;
  }

  thread t([&] () {
    while (!_terminate) {
      if (_sensor_touch.value())
      {
        terminate();
        reset();
        break;
      }
      this_thread::sleep_for(chrono::milliseconds(100));
    }
  });
  t.detach();
}

void control::remote_loop()
{
  remote_control r(_sensor_ir);

  if (!r.connected())
  {
    cout << "no infrared sensor found!" << endl;
    return;
  }

  const int speed = 70;
  const int ninety_degrees = 260;

  r.on_red_up = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        drive(speed);
    }
    else
      stop();
  };

  r.on_red_down = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        drive(-speed);
    }
    else
      stop();
  };

  r.on_blue_up = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        turn(-ninety_degrees);
    }
  };

  r.on_blue_down = [&] (bool state)
  {
    if (state)
    {
      if (_state == state_idle)
        turn(ninety_degrees);
    }
  };

  r.on_beacon = [&] (bool state)
  {
    if (state)
      terminate();
  };

  while (!_terminate)
  {
    if (!r.process())
    {
      this_thread::sleep_for(chrono::milliseconds(10));
    }
  }

  reset();
}

void control::drive_autonomously()
{
  if (!_sensor_ir.connected())
  {
    cout << "no infrared sensor found!" << endl;
    return;
  }

  _sensor_ir.set_mode(infrared_sensor::mode_proximity);

  while (!_terminate)
  {
    int distance = _sensor_ir.value();
    if (distance <= 0)
    {
      // panic
      terminate();
      reset();
      break;
    }
    else if (distance >= 20)
    {
      if (_state != state_driving)
        drive(75);
      this_thread::sleep_for(chrono::milliseconds(10));
    }
    else
    {
      stop();

      int direction = 100;
      int start_distance = distance;

      while (distance <= 40)
      {
        turn(direction);

        distance = _sensor_ir.value();
        if (distance < start_distance)
        {
          if (direction < 0)
          {
            drive(-70, 1000);
          }
          else
          {
            direction = -200;
          }
        }
      }
    }
  }
}

int main()
{
  control c;

  if (c.initialized())
  {
    c.terminate_on_key(); // we terminate if a button is pressed
    c.panic_if_touched(); // we panic if the touch sensor is triggered

  // change mode to 1 to get IR remote mode
    int mode = 2;
    if (mode == 1)
    {
      cout << "ensure that channel 1 is selected on your remote control." << endl << endl
           << "upper red button  - forward" << endl
           << "lower red button  - backward" << endl
           << "upper blue button - left" << endl
           << "lower blue button - right" << endl
           << "middle button     - exit" << endl << endl;

      c.remote_loop();
    }
    else if (mode == 2)
    {
      cout << "touch the sensor or press a button to stop." << endl << endl;

      c.drive_autonomously();
    }
  }
  else
  {
    cout << "you need to connect an infrared sensor and large motors to ports B and C!" << endl;
    return 1;
  }

  return 0;
}
