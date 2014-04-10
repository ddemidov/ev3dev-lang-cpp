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
 */

#pragma once

//-----------------------------------------------------------------------------

#include <set>
#include <string>

//-----------------------------------------------------------------------------

namespace ev3dev {

//-----------------------------------------------------------------------------
  
typedef std::string         path_type;
typedef std::string         mode_type;
typedef std::set<mode_type> mode_set;
  
//-----------------------------------------------------------------------------

class sensor
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
  
  path_type _path;
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
};

//-----------------------------------------------------------------------------

class ultrasonic_sensor : public msensor
{
public:
  ultrasonic_sensor(unsigned port_ = 0);
};

//-----------------------------------------------------------------------------

class gyro_sensor : public msensor
{
public:
  gyro_sensor(unsigned port_ = 0);
};

//-----------------------------------------------------------------------------

class infrared_sensor : public msensor
{
public:
  infrared_sensor(unsigned port_ = 0);
};

//-----------------------------------------------------------------------------

class led
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
  
protected:
  path_type _path;
};

//-----------------------------------------------------------------------------

class button
{
public:
  button(const std::string &name);
  
  bool pressed() const;
  
  static button back;
  static button left;
  static button right;
  static button up;
  static button down;
  static button enter;
  
protected:
  path_type _path;
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

} // namespace ev3dev

//-----------------------------------------------------------------------------
