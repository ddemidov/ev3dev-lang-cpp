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

#include <map>
#include <set>
#include <string>
#include <functional>

//-----------------------------------------------------------------------------

namespace ev3dev {

//-----------------------------------------------------------------------------
  
typedef std::string         device_type;
typedef std::string         port_type;
typedef std::string         mode_type;
typedef std::set<mode_type> mode_set;
typedef std::string         address_type;

//-----------------------------------------------------------------------------

const port_type INPUT_AUTO;          //!< Automatic input selection
const port_type INPUT_1  { "in1" };  //!< Sensor port 1
const port_type INPUT_2  { "in2" };  //!< Sensor port 2
const port_type INPUT_3  { "in3" };  //!< Sensor port 3
const port_type INPUT_4  { "in4" };  //!< Sensor port 4
 
const port_type OUTPUT_AUTO;         //!< Automatic output selection
const port_type OUTPUT_A { "outA" }; //!< Motor port A
const port_type OUTPUT_B { "outB" }; //!< Motor port B
const port_type OUTPUT_C { "outC" }; //!< Motor port C
const port_type OUTPUT_D { "outD" }; //!< Motor port D

//-----------------------------------------------------------------------------

class device
{
public:
  bool connect(const std::string &dir,
               const std::string &pattern,
               const std::map<std::string,
                              std::set<std::string>> match) noexcept;
  inline bool connected() const { return !_path.empty(); }

  int         device_index() const;
  
  int         get_attr_int   (const std::string &name) const;
  void        set_attr_int   (const std::string &name,
                              int value);
  std::string get_attr_string(const std::string &name) const;
  void        set_attr_string(const std::string &name,
                              const std::string &value);

  std::string get_attr_line  (const std::string &name) const;
  mode_set    get_attr_set   (const std::string &name,
                              std::string *pCur = nullptr) const;
  
  std::string get_attr_from_set(const std::string &name) const;
  
protected:
  std::string _path;
  mutable int _device_index = -1;
};

//-----------------------------------------------------------------------------

class sensor : protected device
{
public:
  typedef device_type sensor_type;
  
  static const sensor_type ev3_touch;
  static const sensor_type ev3_color;
  static const sensor_type ev3_ultrasonic;
  static const sensor_type ev3_gyro;
  static const sensor_type ev3_infrared;
  
  static const sensor_type nxt_touch;
  static const sensor_type nxt_light;
  static const sensor_type nxt_sound;
  static const sensor_type nxt_ultrasonic;
  static const sensor_type nxt_i2c_sensor;
  
  sensor(port_type);
  sensor(port_type, const std::set<sensor_type>&);
 
  using device::connected;
  using device::device_index;

  inline const std::string &port_name()   const { return _port_name; }
  inline const sensor_type &type()        const { return _type; }
         const std::string &type_name()   const;
  inline unsigned           num_values()  const { return _nvalues; }
  inline unsigned           dp()          const { return _dp; }
  
  int   value(unsigned index=0) const;
  float float_value(unsigned index=0) const;
  
  const mode_set  &modes() const;
  const mode_type &mode()  const;
  
  void set_mode(const mode_type&);
  
protected:
  sensor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
  void init_members(bool);
  
protected:
  unsigned _nvalues = 0;
  unsigned _dp = 0;
  float    _dp_scale = 1.f;
  
  sensor_type _type;
  port_type   _port_name;
  mode_set    _modes;
  mode_type   _mode;
};

//-----------------------------------------------------------------------------

class i2c_sensor : public sensor
{
public:
  i2c_sensor(port_type port = INPUT_AUTO);
  i2c_sensor(port_type port, address_type address);
};

//-----------------------------------------------------------------------------

class touch_sensor : public sensor
{
public:
  touch_sensor(port_type port_ = INPUT_AUTO);
};

//-----------------------------------------------------------------------------

class color_sensor : public sensor
{
public:
  color_sensor(port_type port_ = INPUT_AUTO);
  
  static const mode_type mode_reflect;
  static const mode_type mode_ambient;
  static const mode_type mode_color;
};

//-----------------------------------------------------------------------------

class ultrasonic_sensor : public sensor
{
public:
  ultrasonic_sensor(port_type port_ = INPUT_AUTO);

  static const mode_type mode_dist_cm;
  static const mode_type mode_dist_in;
  static const mode_type mode_listen;
  static const mode_type mode_single_cm;
  static const mode_type mode_single_in;
};

//-----------------------------------------------------------------------------

class gyro_sensor : public sensor
{
public:
  gyro_sensor(port_type port_ = INPUT_AUTO);
  
  static const mode_type mode_angle;
  static const mode_type mode_speed;
  static const mode_type mode_angle_and_speed;
};

//-----------------------------------------------------------------------------

class infrared_sensor : public sensor
{
public:
  infrared_sensor(port_type port_ = INPUT_AUTO);

  static const mode_type mode_proximity;
  static const mode_type mode_ir_seeker;
  static const mode_type mode_ir_remote;
};

//-----------------------------------------------------------------------------

class motor : protected device
{
public:
  typedef device_type motor_type;
  
  motor(port_type);
  motor(port_type, const motor_type&);
  
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
  
  using device::connected;
  using device::device_index;

  inline const std::string port_name() const { return _port_name; }

  motor_type type() const { return get_attr_string("type"); }

  void run(bool bRun=true) { set_attr_int("run",   bRun); }
  void stop()              { set_attr_int("run",   0);    }
  void reset()             { set_attr_int("reset", 1);    }

  bool      running() const { return get_attr_int("run")!=0; }
  mode_type state()   const { return get_attr_string("state"); }

  int duty_cycle()        const { return get_attr_int("duty_cycle"); }
  int pulses_per_second() const { return get_attr_int("pulses_per_second"); }
  int position()          const { return get_attr_int("position"); }

  void set_position(int p) { set_attr_int("position", p); }

  // Stop Modes|String Array|Read

  mode_type run_mode() const     { return get_attr_string("run_mode"); }
  void set_run_mode(const mode_type &v) { set_attr_string("run_mode", v); }

  mode_type stop_mode() const     { return get_attr_string("stop_mode"); }
  void set_stop_mode(const mode_type &v) { set_attr_string("stop_mode", v); }

  mode_type regulation_mode() const     { return get_attr_string("regulation_mode"); }
  void set_regulation_mode(const mode_type &v) { set_attr_string("regulation_mode", v); }

  mode_type position_mode() const     { return get_attr_string("position_mode"); }
  void set_position_mode(const mode_type &v) { set_attr_string("position_mode", v); }

  int  duty_cycle_setpoint() const { return get_attr_int("duty_cycle_sp"); }
  void set_duty_cycle_setpoint(int v)     { set_attr_int("duty_cycle_sp", v); }

  int  pulses_per_second_setpoint() const { return get_attr_int("pulses_per_second_sp"); }
  void set_pulses_per_second_setpoint(int v)     { set_attr_int("pulses_per_second_sp", v); }

  int  time_setpoint() const { return get_attr_int("time_sp"); }
  void set_time_setpoint(int v)     { set_attr_int("time_sp", v); }

  int  position_setpoint() const { return get_attr_int("position_sp"); }
  void set_position_setpoint(int v)     { set_attr_int("position_sp", v); }

  int  ramp_up() const { return get_attr_int("ramp_up_sp"); }
  void set_ramp_up(int v)     { set_attr_int("ramp_up_sp", v); }

  int  ramp_down() const { return get_attr_int("ramp_down_sp"); }
  void set_ramp_down(int v)     { set_attr_int("ramp_down_sp", v); }

  int speed_regulation_p() const { return get_attr_int("speed_regulation_p"); }
  void set_speed_regulation_p(int v)    { set_attr_int("speed_regulation_p", v); }

  int speed_regulation_i() const { return get_attr_int("speed_regulation_i"); }
  void set_speed_regulation_i(int v)    { set_attr_int("speed_regulation_i", v); }

  int speed_regulation_d() const { return get_attr_int("speed_regulation_d"); }
  void set_speed_regulation_d(int v)    { set_attr_int("speed_regulation_d", v); }

  int speed_regulation_k() const { return get_attr_int("speed_regulation_k"); }
  void set_speed_regulation_k(int v) { set_attr_int("speed_regulation_k", v); }
  
protected:
  motor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
  
protected:
  std::string _port_name;
  motor_type  _type;
};

//-----------------------------------------------------------------------------

class medium_motor : public motor
{
public:
  medium_motor(port_type port_ = OUTPUT_AUTO);
};

//-----------------------------------------------------------------------------

class large_motor : public motor
{
public:
  large_motor(port_type port_ = OUTPUT_AUTO);
};

//-----------------------------------------------------------------------------

class led : protected device
{
public:
  led(std::string name);

  using device::connected;
  
  int brightness() const { return get_attr_int("brightness"); }
  void set_brightness(int v)    { set_attr_int("brightness", v); }

  inline int max_brightness() const { return _max_brightness; }
  
  void on()  { set_attr_int("brightness", _max_brightness); }
  void off() { set_attr_int("brightness", 0); }
  
  void flash(unsigned interval_ms);
  void set_on_delay (unsigned ms) { set_attr_int("delay_on",  ms); }
  void set_off_delay(unsigned ms) { set_attr_int("delay_off", ms); }
  
  mode_type trigger()  const  { return get_attr_from_set("trigger"); }
  mode_set  triggers() const  { return get_attr_set     ("trigger"); }
  void set_trigger(const mode_type &v) { set_attr_string("trigger", v); }

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
  int _max_brightness = 0;
};

//-----------------------------------------------------------------------------

class power_supply : protected device
{
public:
  power_supply(std::string name);
  
  using device::connected;
  
  int   current_now()        const { return get_attr_int("current_now"); }
  float current_amps()       const { return get_attr_int("current_now") / 1000.f; }
  int   current_max_design() const { return get_attr_int("current_max_design"); }

  int   voltage_now()        const { return get_attr_int("voltage_now"); }
  float voltage_volts()      const { return get_attr_int("voltage_now") / 1000000.f; }
  int   voltage_max_design() const { return get_attr_int("voltage_max_design"); }
  
  std::string technology() const { return get_attr_string("technology"); }
  std::string type()       const { return get_attr_string("type"); }
  
  static power_supply battery;
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
  infrared_sensor *_sensor = nullptr;
  bool             _owns_sensor = false;
  unsigned         _channel = 0;
  int              _value = 0;
  int              _state = 0;
};

//-----------------------------------------------------------------------------

} // namespace ev3dev

//-----------------------------------------------------------------------------
