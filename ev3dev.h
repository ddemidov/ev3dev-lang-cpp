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
//~autogen autogen-header
    // Sections of the following code were auto-generated based on spec v0.9.2-pre, rev 3. 
//~autogen
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
                              std::set<std::string>> &match) noexcept;
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

  int   value(unsigned index=0) const;
  float float_value(unsigned index=0) const;
  std::string type_name() const;

  //~autogen cpp_generic-get-set classes.sensor>currentClass

    void set_command(std::string v) { set_attr_string("command", v); }
    mode_set commands() const { return get_attr_set("commands"); }
    int decimals() const { return get_attr_int("decimals"); }
    std::string driver_name() const { return get_attr_string("driver_name"); }
    std::string mode() const { return get_attr_string("mode"); }
    void set_mode(std::string v) { set_attr_string("mode", v); }
    mode_set modes() const { return get_attr_set("modes"); }
    int num_values() const { return get_attr_int("num_values"); }
    std::string port_name() const { return get_attr_string("port_name"); }
    std::string units() const { return get_attr_string("units"); }

//~autogen

protected:
  sensor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
};

//-----------------------------------------------------------------------------

class i2c_sensor : public sensor
{
public:
  i2c_sensor(port_type port = INPUT_AUTO);
  i2c_sensor(port_type port, address_type address);

  //~autogen cpp_generic-get-set classes.i2cSensor>currentClass

    std::string fw_version() const { return get_attr_string("fw_version"); }
    int poll_ms() const { return get_attr_int("poll_ms"); }
    void set_poll_ms(int v) { set_attr_int("poll_ms", v); }

//~autogen
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

  //~autogen cpp_generic-declare-property-value classes.colorSensor>currentClass

    static const std::string mode_col_reflect;
    static const std::string mode_col_ambient;
    static const std::string mode_col_color;
    static const std::string mode_ref_raw;
    static const std::string mode_rgb_raw;
    static const std::string mode_col_cal;

//~autogen
};

//-----------------------------------------------------------------------------

class ultrasonic_sensor : public sensor
{
public:
  ultrasonic_sensor(port_type port_ = INPUT_AUTO);

  //~autogen cpp_generic-declare-property-value classes.ultrasonicSensor>currentClass

    static const std::string mode_us_dist_cm;
    static const std::string mode_us_dist_in;
    static const std::string mode_us_listen;
    static const std::string mode_us_si_cm;
    static const std::string mode_us_si_in;
    static const std::string mode_us_dc_cm;
    static const std::string mode_us_dc_in;

//~autogen
};

//-----------------------------------------------------------------------------

class gyro_sensor : public sensor
{
public:
  gyro_sensor(port_type port_ = INPUT_AUTO);

  //~autogen cpp_generic-declare-property-value classes.gyroSensor>currentClass

    static const std::string mode_gyro_ang;
    static const std::string mode_gyro_rate;
    static const std::string mode_gyro_fas;
    static const std::string mode_gyro_g_a;
    static const std::string mode_gyro_cal;

//~autogen
};

//-----------------------------------------------------------------------------

class infrared_sensor : public sensor
{
public:
  infrared_sensor(port_type port_ = INPUT_AUTO);

  //~autogen cpp_generic-declare-property-value classes.infraredSensor>currentClass

    static const std::string mode_ir_prox;
    static const std::string mode_ir_seek;
    static const std::string mode_ir_remote;
    static const std::string mode_ir_rem_a;
    static const std::string mode_ir_s_alt;
    static const std::string mode_ir_cal;

//~autogen
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

  using device::connected;
  using device::device_index;

  //~autogen cpp_generic-declare-property-value classes.motor>currentClass

    static const std::string command_run_forever;
    static const std::string command_run_to_abs_pos;
    static const std::string command_run_to_rel_pos;
    static const std::string command_run_timed;
    static const std::string command_run_direct;
    static const std::string command_stop;
    static const std::string command_reset;
    static const std::string encoder_polarity_normal;
    static const std::string encoder_polarity_inverted;
    static const std::string polarity_normal;
    static const std::string polarity_inverted;
    static const std::string speed_regulation_on;
    static const std::string speed_regulation_off;
    static const std::string stop_command_coast;
    static const std::string stop_command_brake;
    static const std::string stop_command_hold;

//~autogen

  //~autogen cpp_generic-get-set classes.motor>currentClass

    void set_command(std::string v) { set_attr_string("command", v); }
    mode_set commands() const { return get_attr_set("commands"); }
    int count_per_rot() const { return get_attr_int("count_per_rot"); }
    std::string driver_name() const { return get_attr_string("driver_name"); }
    int duty_cycle() const { return get_attr_int("duty_cycle"); }
    int duty_cycle_sp() const { return get_attr_int("duty_cycle_sp"); }
    void set_duty_cycle_sp(int v) { set_attr_int("duty_cycle_sp", v); }
    std::string encoder_polarity() const { return get_attr_string("encoder_polarity"); }
    void set_encoder_polarity(std::string v) { set_attr_string("encoder_polarity", v); }
    std::string polarity() const { return get_attr_string("polarity"); }
    void set_polarity(std::string v) { set_attr_string("polarity", v); }
    std::string port_name() const { return get_attr_string("port_name"); }
    int position() const { return get_attr_int("position"); }
    void set_position(int v) { set_attr_int("position", v); }
    int position_p() const { return get_attr_int("hold_pid/Kp"); }
    void set_position_p(int v) { set_attr_int("hold_pid/Kp", v); }
    int position_i() const { return get_attr_int("hold_pid/Ki"); }
    void set_position_i(int v) { set_attr_int("hold_pid/Ki", v); }
    int position_d() const { return get_attr_int("hold_pid/Kd"); }
    void set_position_d(int v) { set_attr_int("hold_pid/Kd", v); }
    int position_sp() const { return get_attr_int("position_sp"); }
    void set_position_sp(int v) { set_attr_int("position_sp", v); }
    int speed() const { return get_attr_int("speed"); }
    int speed_sp() const { return get_attr_int("speed_sp"); }
    void set_speed_sp(int v) { set_attr_int("speed_sp", v); }
    int ramp_up_sp() const { return get_attr_int("ramp_up_sp"); }
    void set_ramp_up_sp(int v) { set_attr_int("ramp_up_sp", v); }
    int ramp_down_sp() const { return get_attr_int("ramp_down_sp"); }
    void set_ramp_down_sp(int v) { set_attr_int("ramp_down_sp", v); }
    std::string speed_regulation_enabled() const { return get_attr_string("speed_regulation"); }
    void set_speed_regulation_enabled(std::string v) { set_attr_string("speed_regulation", v); }
    int speed_regulation_p() const { return get_attr_int("speed_pid/Kp"); }
    void set_speed_regulation_p(int v) { set_attr_int("speed_pid/Kp", v); }
    int speed_regulation_i() const { return get_attr_int("speed_pid/Ki"); }
    void set_speed_regulation_i(int v) { set_attr_int("speed_pid/Ki", v); }
    int speed_regulation_d() const { return get_attr_int("speed_pid/Kd"); }
    void set_speed_regulation_d(int v) { set_attr_int("speed_pid/Kd", v); }
    mode_set state() const { return get_attr_set("state"); }
    std::string stop_command() const { return get_attr_string("stop_command"); }
    void set_stop_command(std::string v) { set_attr_string("stop_command", v); }
    mode_set stop_commands() const { return get_attr_set("stop_commands"); }
    int time_sp() const { return get_attr_int("time_sp"); }
    void set_time_sp(int v) { set_attr_int("time_sp", v); }

//~autogen

protected:
  motor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
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

class dc_motor : protected device
{
public:
  dc_motor(port_type port_ = OUTPUT_AUTO);

  using device::connected;
  using device::device_index;

  //~autogen cpp_generic-declare-property-value classes.dcMotor>currentClass

    static const std::string command_run_forever;
    static const std::string command_run_timed;
    static const std::string command_stop;
    static const std::string polarity_normal;
    static const std::string polarity_inverted;
    static const std::string stop_command_coast;
    static const std::string stop_command_brake;

//~autogen

  //~autogen cpp_generic-get-set classes.dcMotor>currentClass

    void set_command(std::string v) { set_attr_string("command", v); }
    mode_set commands() const { return get_attr_set("commands"); }
    std::string driver_name() const { return get_attr_string("driver_name"); }
    int duty_cycle() const { return get_attr_int("duty_cycle"); }
    int duty_cycle_sp() const { return get_attr_int("duty_cycle_sp"); }
    void set_duty_cycle_sp(int v) { set_attr_int("duty_cycle_sp", v); }
    std::string polarity() const { return get_attr_string("polarity"); }
    void set_polarity(std::string v) { set_attr_string("polarity", v); }
    std::string port_name() const { return get_attr_string("port_name"); }
    int ramp_down_sp() const { return get_attr_int("ramp_down_sp"); }
    void set_ramp_down_sp(int v) { set_attr_int("ramp_down_sp", v); }
    int ramp_up_sp() const { return get_attr_int("ramp_up_sp"); }
    void set_ramp_up_sp(int v) { set_attr_int("ramp_up_sp", v); }
    mode_set state() const { return get_attr_set("state"); }
    void set_stop_command(std::string v) { set_attr_string("stop_command", v); }
    mode_set stop_commands() const { return get_attr_set("stop_commands"); }

//~autogen

protected:
  std::string _port_name;
};

//-----------------------------------------------------------------------------

class servo_motor : protected device
{
public:
  servo_motor(port_type port_ = OUTPUT_AUTO);

  using device::connected;
  using device::device_index;

  //~autogen cpp_generic-declare-property-value classes.servoMotor>currentClass

    static const std::string command_run;
    static const std::string command_float;
    static const std::string polarity_normal;
    static const std::string polarity_inverted;

//~autogen

  //~autogen cpp_generic-get-set classes.servoMotor>currentClass

    void set_command(std::string v) { set_attr_string("command", v); }
    std::string driver_name() const { return get_attr_string("driver_name"); }
    int max_pulse_sp() const { return get_attr_int("max_pulse_sp"); }
    void set_max_pulse_sp(int v) { set_attr_int("max_pulse_sp", v); }
    int mid_pulse_sp() const { return get_attr_int("mid_pulse_sp"); }
    void set_mid_pulse_sp(int v) { set_attr_int("mid_pulse_sp", v); }
    int min_pulse_sp() const { return get_attr_int("min_pulse_sp"); }
    void set_min_pulse_sp(int v) { set_attr_int("min_pulse_sp", v); }
    std::string polarity() const { return get_attr_string("polarity"); }
    void set_polarity(std::string v) { set_attr_string("polarity", v); }
    std::string port_name() const { return get_attr_string("port_name"); }
    int position_sp() const { return get_attr_int("position_sp"); }
    void set_position_sp(int v) { set_attr_int("position_sp", v); }
    int rate_sp() const { return get_attr_int("rate_sp"); }
    void set_rate_sp(int v) { set_attr_int("rate_sp", v); }
    mode_set state() const { return get_attr_set("state"); }

//~autogen
};

//-----------------------------------------------------------------------------

class led : protected device
{
public:
  led(std::string name);

  using device::connected;

  //~autogen cpp_generic-get-set classes.led>currentClass

    int max_brightness() const { return get_attr_int("max_brightness"); }
    int brightness() const { return get_attr_int("brightness"); }
    void set_brightness(int v) { set_attr_int("brightness", v); }
    std::string trigger() const { return get_attr_string("trigger"); }
    void set_trigger(std::string v) { set_attr_string("trigger", v); }

//~autogen

  mode_set  triggers() const  { return get_attr_set     ("trigger"); }

  void on()  { set_brightness(max_brightness()); }
  void off() { set_brightness(0); }

  void flash(unsigned interval_ms);
  void set_on_delay (unsigned ms) { set_attr_int("delay_on",  ms); }
  void set_off_delay(unsigned ms) { set_attr_int("delay_off", ms); }

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

  //~autogen cpp_generic-get-set classes.powerSupply>currentClass

    int measured_current() const { return get_attr_int("current_now"); }
    int measured_voltage() const { return get_attr_int("voltage_now"); }
    int max_voltage() const { return get_attr_int("voltage_max_design"); }
    int min_voltage() const { return get_attr_int("voltage_min_design"); }
    std::string technology() const { return get_attr_string("technology"); }
    std::string type() const { return get_attr_string("type"); }

//~autogen

  float measured_amps()       const { return measured_current() / 1000000.f; }
  float measured_volts()      const { return measured_voltage() / 1000000.f; }

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
  std::function<void (int)>  on_state_change;

  enum buttons
  {
    red_up    = (1 << 0),
    red_down  = (1 << 1),
    blue_up   = (1 << 2),
    blue_down = (1 << 3),
    beacon    = (1 << 4),
  };

protected:
  virtual void on_value_changed(int value);

  infrared_sensor *_sensor = nullptr;
  bool             _owns_sensor = false;
  unsigned         _channel = 0;
  int              _value = 0;
  int              _state = 0;
};

//-----------------------------------------------------------------------------

} // namespace ev3dev

//-----------------------------------------------------------------------------
