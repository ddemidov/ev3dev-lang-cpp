/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
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
 *
 * Modification:
 *  Add new button management for ev3dev Release 02.00.00 (ev3dev-jessie-2014-07-12) - Christophe Chaudelet
 *
 */

#pragma once

//-----------------------------------------------------------------------------

#include <set>
#include <string>
#include <functional>

//-----------------------------------------------------------------------------

namespace ev3dev {

//-----------------------------------------------------------------------------
  
typedef std::string         mode_type;
typedef std::set<mode_type> mode_set;

//-----------------------------------------------------------------------------

class device
{
protected:
  int         get_attr_int   (const std::string &name) const;
  void        set_attr_int   (const std::string &name,
                              int value);
  std::string get_attr_string(const std::string &name) const;
  void        set_attr_string(const std::string &name,
                              const std::string &value);
protected:
  std::string _path;
};

//-----------------------------------------------------------------------------

class sensor : protected device
{
public:
  sensor();
  
  inline bool     connected()  const { return (_port != 0); }
	inline unsigned port()       const { return _port; }
	inline unsigned type()       const { return _type; }
  inline unsigned num_values() const { return _nvalues; }
  
  virtual int value(unsigned index=0) const = 0;
  
  const mode_set  &modes() const;
  const mode_type &mode()  const;
  
  virtual void set_mode(const mode_type&) = 0;
  
  enum type
  {
    nxt_touch       = 1,
    nxt_light       = 2,
    nxt_sound       = 3,
    nxt_color       = 4,
    nxt_ultrasonic  = 5,
    nxt_temperature = 6,
    
    ev3_touch       = 16,
    ev3_color       = 29,
    ev3_ultrasonic  = 30,
    ev3_gyro        = 32,
    ev3_infrared    = 33,
  };

  static const std::string &as_string(unsigned type);
  
protected:
  virtual void read_modes();
  
protected:
  unsigned _port;
  unsigned _type;
  unsigned _nvalues;
  
  mode_set  _modes;
  mode_type _mode;
};

//-----------------------------------------------------------------------------

class msensor : public sensor
{
public:
  msensor(unsigned type_, unsigned port_ = 0);
  
  virtual int  value(unsigned index=0) const;
  virtual void set_mode(const mode_type&);
  
protected:
  bool init(unsigned type_, unsigned port_ = 0);
  
  virtual void read_modes();
};

//-----------------------------------------------------------------------------

class touch_sensor : public msensor
{
public:
  touch_sensor(unsigned port_ = 0);
};

//-----------------------------------------------------------------------------

class color_sensor : public msensor
{
public:
  color_sensor(unsigned port_ = 0);
  
  static const mode_type mode_reflect;
  static const mode_type mode_ambient;
  static const mode_type mode_color;
};

//-----------------------------------------------------------------------------

class ultrasonic_sensor : public msensor
{
public:
  ultrasonic_sensor(unsigned port_ = 0);

  static const mode_type mode_dist_cm;
  static const mode_type mode_dist_in;
  static const mode_type mode_listen;
  static const mode_type mode_single_cm;
  static const mode_type mode_single_in;
};

//-----------------------------------------------------------------------------

class gyro_sensor : public msensor
{
public:
  gyro_sensor(unsigned port_ = 0);
  
  static const mode_type mode_angle;
  static const mode_type mode_speed;
  static const mode_type mode_angle_and_speed;
};

//-----------------------------------------------------------------------------

class infrared_sensor : public msensor
{
public:
  infrared_sensor(unsigned port_ = 0);

  static const mode_type mode_proximity;
  static const mode_type mode_ir_seeker;
  static const mode_type mode_ir_remote;
};

//-----------------------------------------------------------------------------

class motor : protected device
{
public:
  typedef std::string motor_type;
  
  motor(const motor_type&, unsigned port_ = 0);
  
  static const motor_type motor_large;
  static const motor_type motor_medium;
  
  static const mode_type mode_off;
  static const mode_type mode_on;
    
  static const mode_type run_mode_forever;
  static const mode_type run_mode_time;
  static const mode_type run_mode_position;

  static const mode_type stop_mode_coast;
  static const mode_type stop_mode_brake;
  static const mode_type stop_mode_hold;
  
  static const mode_type position_mode_absolute;
  static const mode_type position_mode_relative;
  
  inline bool     connected()  const { return (_port != 0); }
	inline unsigned port()       const { return _port; }

  motor_type type() const;
  
  void run(bool bRun=true);
  void stop();
  void reset();

  bool      running() const;
  mode_type state()   const;
  
  int duty_cycle()        const;
  int pulses_per_second() const;
  int position()          const;
  
  void set_position(int);
  
  mode_type run_mode() const;
  void set_run_mode(const mode_type&);
  
  mode_type stop_mode() const;
  void set_stop_mode(const mode_type&);
  
  mode_type regulation_mode() const;
  void set_regulation_mode(const mode_type&);

  mode_type position_mode() const;
  void set_position_mode(const mode_type&);

  int  duty_cycle_setpoint() const;
  void set_duty_cycle_setpoint(int);
  
  int  pulses_per_second_setpoint() const;
  void set_pulses_per_second_setpoint(int);

  int  time_setpoint() const;
  void set_time_setpoint(int);

  int  position_setpoint() const;
  void set_position_setpoint(int);

  int  ramp_up() const;
  void set_ramp_up(int);
  
  int  ramp_down() const;
  void set_ramp_down(int);
  
protected:
  bool init(const motor_type&, unsigned port_ = 0);
  
protected:
  unsigned   _port;
  motor_type _type;
};

//-----------------------------------------------------------------------------

class medium_motor : public motor
{
public:
  medium_motor(unsigned port_ = 0);
};

//-----------------------------------------------------------------------------

class large_motor : public motor
{
public:
  large_motor(unsigned port_ = 0);
};

//-----------------------------------------------------------------------------

class led : protected device
{
public:
  led(const std::string &name);
 
  int  level() const;
  void on();
  void off();
  void flash(unsigned interval_ms);
  void set_on_delay (unsigned ms);
  void set_off_delay(unsigned ms);
  
  mode_type trigger() const;
  mode_set  triggers() const;
  void set_trigger(const mode_type&);

  static led red_right;
  static led red_left;
  static led green_right;
  static led green_left;

  static void red_on   ();
  static void red_off  ();
  static void green_on ();
  static void green_off();
  static void all_on   ();
  static void all_off  ();
};

//-----------------------------------------------------------------------------

class button
{
public:
  button(int bit);
  ~button()
  {
    delete _buf;
  }
  
  bool pressed() const;
  
  static button back;
  static button left;
  static button right;
  static button up;
  static button down;
  static button enter;

private:
  int _bit;
  int _fd;
  int _bits_per_long;
  unsigned long *_buf;
  unsigned long _buf_size;

};

//-----------------------------------------------------------------------------

class sound
{
public:
  static void beep();
  static void tone(unsigned frequency, unsigned ms);
  
  static void play (const std::string &soundfile, bool bSynchronous = false);
  static void speak(const std::string &text, bool bSynchronous = false);
  
  static unsigned volume();
  static void set_volume(unsigned);
};

//-----------------------------------------------------------------------------

class battery
{
public:
  static float voltage();
  static float current();
};

//-----------------------------------------------------------------------------

class lcd
{
public:
  lcd();
  ~lcd();

  bool available() const { return _fb != nullptr; }

  uint32_t resolution_x()   const { return _xres; }
  uint32_t resolution_y()   const { return _yres; }
  uint32_t bits_per_pixel() const { return _bpp; }

  uint32_t frame_buffer_size() const { return _fbsize; }
  uint32_t line_length()       const { return _llength; }
  
  unsigned char *frame_buffer() { return _fb; }
  
  void fill(unsigned char pixel);
  
protected:
  void init();
  void deinit();
  
private:
  unsigned char *_fb;
  uint32_t _fbsize;
  uint32_t _llength;
  uint32_t _xres;
  uint32_t _yres;
  uint32_t _bpp;
};

//-----------------------------------------------------------------------------

class remote_control
{
public:
  remote_control(unsigned channel = 1);
  remote_control(infrared_sensor&, unsigned channel = 1);
  virtual ~remote_control();
  
  inline bool   connected() const { return _sensor->connected(); }
  inline unsigned channel() const { return _channel+1; }

  bool process();

  std::function<void (bool)> on_red_up;
  std::function<void (bool)> on_red_down;
  std::function<void (bool)> on_blue_up;
  std::function<void (bool)> on_blue_down;
  std::function<void (bool)> on_beacon;
  
protected:
  virtual void on_value_changed(int value);
  
  enum
  {
    red_up    = (1 << 0),
    red_down  = (1 << 1),
    blue_up   = (1 << 2),
    blue_down = (1 << 3),
    beacon    = (1 << 4),
  };
  
protected:
  infrared_sensor *_sensor;
  bool             _owns_sensor;
  unsigned         _channel;
  int              _value;
  int              _state;
};

//-----------------------------------------------------------------------------

} // namespace ev3dev

//-----------------------------------------------------------------------------
