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

// Sections of the following code were auto-generated based on spec v0.9.3-pre, rev 2.

//~autogen
//-----------------------------------------------------------------------------

#include <map>
#include <set>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <memory>

//-----------------------------------------------------------------------------

namespace ev3dev {

//-----------------------------------------------------------------------------

typedef std::string         device_type;
typedef std::string         mode_type;
typedef std::set<mode_type> mode_set;
typedef std::string         address_type;

//-----------------------------------------------------------------------------

const address_type INPUT_AUTO;  //!< Automatic input selection
const address_type OUTPUT_AUTO; //!< Automatic output selection

#ifdef EV3DEV_PLATFORM_BRICKPI
const address_type INPUT_1  { "ttyAMA0:in1" };  //!< Sensor port 1
const address_type INPUT_2  { "ttyAMA0:in2" };  //!< Sensor port 2
const address_type INPUT_3  { "ttyAMA0:in3" };  //!< Sensor port 3
const address_type INPUT_4  { "ttyAMA0:in4" };  //!< Sensor port 4

const address_type OUTPUT_A { "ttyAMA0:outA" }; //!< Motor port A
const address_type OUTPUT_B { "ttyAMA0:outB" }; //!< Motor port B
const address_type OUTPUT_C { "ttyAMA0:outC" }; //!< Motor port C
const address_type OUTPUT_D { "ttyAMA0:outD" }; //!< Motor port D
#else
const address_type INPUT_1  { "in1" };  //!< Sensor port 1
const address_type INPUT_2  { "in2" };  //!< Sensor port 2
const address_type INPUT_3  { "in3" };  //!< Sensor port 3
const address_type INPUT_4  { "in4" };  //!< Sensor port 4

const address_type OUTPUT_A { "outA" }; //!< Motor port A
const address_type OUTPUT_B { "outB" }; //!< Motor port B
const address_type OUTPUT_C { "outC" }; //!< Motor port C
const address_type OUTPUT_D { "outD" }; //!< Motor port D
#endif

//-----------------------------------------------------------------------------

// Generic device class.
class device
{
public:
  bool connect(const std::string &dir,
               const std::string &pattern,
               const std::map<std::string, std::set<std::string>> &match) noexcept;

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

//~autogen generic-class-description classes.sensor>currentClass

// The sensor class provides a uniform interface for using most of the
// sensors available for the EV3. The various underlying device drivers will
// create a `lego-sensor` device for interacting with the sensors.
// 
// Sensors are primarily controlled by setting the `mode` and monitored by
// reading the `value<N>` attributes. Values can be converted to floating point
// if needed by `value<N>` / 10.0 ^ `decimals`.
// 
// Since the name of the `sensor<N>` device node does not correspond to the port
// that a sensor is plugged in to, you must look at the `address` attribute if
// you need to know which port a sensor is plugged in to. However, if you don't
// have more than one sensor of each type, you can just look for a matching
// `driver_name`. Then it will not matter which port a sensor is plugged in to - your
// program will still work.

//~autogen
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
  static const sensor_type nxt_analog;

  sensor(address_type);
  sensor(address_type, const std::set<sensor_type>&);

  using device::connected;
  using device::device_index;

  // Returns the value or values measured by the sensor. Check `num_values` to
  // see how many values there are. Values with index >= num_values will return
  // an error. The values are fixed point numbers, so check `decimals` to see
  // if you need to divide to get the actual value.
  int   value(unsigned index=0) const;

  // The value converted to float using `decimals`.
  float float_value(unsigned index=0) const;

  // Human-readable name of the connected sensor.
  std::string type_name() const;

  // Bin Data Format: read-only
  // Returns the format of the values in `bin_data` for the current mode.
  // Possible values are:
  //
  //    - `u8`: Unsigned 8-bit integer (byte)
  //    - `s8`: Signed 8-bit integer (sbyte)
  //    - `u16`: Unsigned 16-bit integer (ushort)
  //    - `s16`: Signed 16-bit integer (short)
  //    - `s16_be`: Signed 16-bit integer, big endian
  //    - `s32`: Signed 32-bit integer (int)
  //    - `float`: IEEE 754 32-bit floating point (float)
  std::string bin_data_format() const { return get_attr_string("bin_data_format"); };

  // Bin Data: read-only
  // Returns the unscaled raw values in the `value<N>` attributes as raw byte
  // array. Use `bin_data_format`, `num_values` and the individual sensor
  // documentation to determine how to interpret the data.
  const std::vector<char>& bin_data() const;

  // Bin Data: read-only
  // Writes the unscaled raw values in the `value<N>` attributes into the
  // user-provided struct/buffer.  Use `bin_data_format`, `num_values` and the
  // individual sensor documentation to determine how to interpret the data.
  template <class T>
  void bin_data(T *buf) const {
      bin_data(); // fills _bin_data
      std::copy_n(_bin_data.data(), _bin_data.size(), static_cast<char*>(buf));
  }

//~autogen generic-get-set classes.sensor>currentClass

  // Command: write-only
  // Sends a command to the sensor.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Commands: read-only
  // Returns a list of the valid commands for the sensor.
  // Returns -EOPNOTSUPP if no commands are supported.
  mode_set commands() const { return get_attr_set("commands"); }

  // Decimals: read-only
  // Returns the number of decimal places for the values in the `value<N>`
  // attributes of the current mode.
  int decimals() const { return get_attr_int("decimals"); }

  // Driver Name: read-only
  // Returns the name of the sensor device/driver. See the list of [supported
  // sensors] for a complete list of drivers.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Mode: read/write
  // Returns the current mode. Writing one of the values returned by `modes`
  // sets the sensor to that mode.
  std::string mode() const { return get_attr_string("mode"); }
  auto set_mode(std::string v) -> decltype(*this) {
    set_attr_string("mode", v);
    return *this;
  }

  // Modes: read-only
  // Returns a list of the valid modes for the sensor.
  mode_set modes() const { return get_attr_set("modes"); }

  // Num Values: read-only
  // Returns the number of `value<N>` attributes that will return a valid value
  // for the current mode.
  int num_values() const { return get_attr_int("num_values"); }

  // Address: read-only
  // Returns the name of the port that the sensor is connected to, e.g. `ev3:in1`.
  // I2C sensors also include the I2C address (decimal), e.g. `ev3:in1:i2c8`.
  std::string address() const { return get_attr_string("address"); }

  // Units: read-only
  // Returns the units of the measured value for the current mode. May return
  // empty string
  std::string units() const { return get_attr_string("units"); }


//~autogen

protected:
  sensor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;

  mutable std::vector<char> _bin_data;
};

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.i2cSensor>currentClass

// A generic interface to control I2C-type EV3 sensors.

//~autogen
class i2c_sensor : public sensor
{
public:
  i2c_sensor(address_type address = INPUT_AUTO);

//~autogen generic-get-set classes.i2cSensor>currentClass

  // FW Version: read-only
  // Returns the firmware version of the sensor if available. Currently only
  // I2C/NXT sensors support this.
  std::string fw_version() const { return get_attr_string("fw_version"); }

  // Poll MS: read/write
  // Returns the polling period of the sensor in milliseconds. Writing sets the
  // polling period. Setting to 0 disables polling. Minimum value is hard
  // coded as 50 msec. Returns -EOPNOTSUPP if changing polling is not supported.
  // Currently only I2C/NXT sensors support changing the polling period.
  int poll_ms() const { return get_attr_int("poll_ms"); }
  auto set_poll_ms(int v) -> decltype(*this) {
    set_attr_int("poll_ms", v);
    return *this;
  }


//~autogen
};

//-----------------------------------------------------------------------------

//~autogen special-sensor-declaration specialSensorTypes.touchSensor>currentClass

// Touch Sensor
class touch_sensor : public sensor
{
public:
  touch_sensor(address_type address = INPUT_AUTO);

  // Button state
  static const std::string mode_touch;


  // A boolean indicating whether the current touch sensor is being
  // pressed.
  bool is_pressed() {
    set_mode(mode_touch);
    return value(0);
  }

};

//~autogen

//-----------------------------------------------------------------------------

//~autogen special-sensor-declaration specialSensorTypes.colorSensor>currentClass

// LEGO EV3 color sensor.
class color_sensor : public sensor
{
public:
  color_sensor(address_type address = INPUT_AUTO);

  // Reflected light. Red LED on.
  static const std::string mode_col_reflect;

  // Ambient light. Red LEDs off.
  static const std::string mode_col_ambient;

  // Color. All LEDs rapidly cycling, appears white.
  static const std::string mode_col_color;

  // Raw reflected. Red LED on
  static const std::string mode_ref_raw;

  // Raw Color Components. All LEDs rapidly cycling, appears white.
  static const std::string mode_rgb_raw;


  // Reflected light intensity as a percentage. Light on sensor is red.
  int reflected_light_intensity() {
    set_mode(mode_col_reflect);
    return value(0);
  }

  // Ambient light intensity. Light on sensor is dimly lit blue.
  int ambient_light_intensity() {
    set_mode(mode_col_ambient);
    return value(0);
  }

  // Color detected by the sensor, categorized by overall value.
  //   - 0: No color
  //   - 1: Black
  //   - 2: Blue
  //   - 3: Green
  //   - 4: Yellow
  //   - 5: Red
  //   - 6: White
  //   - 7: Brown
  int color() {
    set_mode(mode_col_color);
    return value(0);
  }

  // Red component of the detected color, in the range 0-1020.
  int red() {
    set_mode(mode_rgb_raw);
    return value(0);
  }

  // Green component of the detected color, in the range 0-1020.
  int green() {
    set_mode(mode_rgb_raw);
    return value(1);
  }

  // Blue component of the detected color, in the range 0-1020.
  int blue() {
    set_mode(mode_rgb_raw);
    return value(2);
  }

};

//~autogen

//-----------------------------------------------------------------------------

//~autogen special-sensor-declaration specialSensorTypes.ultrasonicSensor>currentClass

// LEGO EV3 ultrasonic sensor.
class ultrasonic_sensor : public sensor
{
public:
  ultrasonic_sensor(address_type address = INPUT_AUTO);

  // Continuous measurement in centimeters.
  static const std::string mode_us_dist_cm;

  // Continuous measurement in inches.
  static const std::string mode_us_dist_in;

  // Listen.
  static const std::string mode_us_listen;

  // Single measurement in centimeters.
  static const std::string mode_us_si_cm;

  // Single measurement in inches.
  static const std::string mode_us_si_in;


  // Measurement of the distance detected by the sensor,
  // in centimeters.
  float distance_centimeters() {
    set_mode(mode_us_dist_cm);
    return float_value(0);
  }

  // Measurement of the distance detected by the sensor,
  // in inches.
  float distance_inches() {
    set_mode(mode_us_dist_in);
    return float_value(0);
  }

  // Value indicating whether another ultrasonic sensor could
  // be heard nearby.
  bool other_sensor_present() {
    set_mode(mode_us_listen);
    return value(0);
  }

};

//~autogen

//-----------------------------------------------------------------------------

//~autogen special-sensor-declaration specialSensorTypes.gyroSensor>currentClass

// LEGO EV3 gyro sensor.
class gyro_sensor : public sensor
{
public:
  gyro_sensor(address_type address = INPUT_AUTO);

  // Angle
  static const std::string mode_gyro_ang;

  // Rotational speed
  static const std::string mode_gyro_rate;

  // Raw sensor value
  static const std::string mode_gyro_fas;

  // Angle and rotational speed
  static const std::string mode_gyro_g_a;

  // Calibration ???
  static const std::string mode_gyro_cal;


  // The number of degrees that the sensor has been rotated
  // since it was put into this mode.
  int angle() {
    set_mode(mode_gyro_ang);
    return value(0);
  }

  // The rate at which the sensor is rotating, in degrees/second.
  int rate() {
    set_mode(mode_gyro_rate);
    return value(0);
  }

};

//~autogen

//-----------------------------------------------------------------------------

//~autogen special-sensor-declaration specialSensorTypes.infraredSensor>currentClass

// LEGO EV3 infrared sensor.
class infrared_sensor : public sensor
{
public:
  infrared_sensor(address_type address = INPUT_AUTO);

  // Proximity
  static const std::string mode_ir_prox;

  // IR Seeker
  static const std::string mode_ir_seek;

  // IR Remote Control
  static const std::string mode_ir_remote;

  // IR Remote Control. State of the buttons is coded in binary
  static const std::string mode_ir_rem_a;

  // Calibration ???
  static const std::string mode_ir_cal;


  // A measurement of the distance between the sensor and the remote,
  // as a percentage. 100% is approximately 70cm/27in.
  int proximity() {
    set_mode(mode_ir_prox);
    return value(0);
  }

};

//~autogen

//-----------------------------------------------------------------------------

//~autogen special-sensor-declaration specialSensorTypes.soundSensor>currentClass

// LEGO NXT Sound Sensor
class sound_sensor : public sensor
{
public:
  sound_sensor(address_type address = INPUT_AUTO);

  // Sound pressure level. Flat weighting
  static const std::string mode_db;

  // Sound pressure level. A weighting
  static const std::string mode_dba;


  // A measurement of the measured sound pressure level, as a
  // percent. Uses a flat weighting.
  float sound_pressure() {
    set_mode(mode_db);
    return float_value(0);
  }

  // A measurement of the measured sound pressure level, as a
  // percent. Uses A-weighting, which focuses on levels up to 55 dB.
  float sound_pressure_low() {
    set_mode(mode_dba);
    return float_value(0);
  }

};

//~autogen

//-----------------------------------------------------------------------------

//~autogen special-sensor-declaration specialSensorTypes.lightSensor>currentClass

// LEGO NXT Light Sensor
class light_sensor : public sensor
{
public:
  light_sensor(address_type address = INPUT_AUTO);

  // Reflected light. LED on
  static const std::string mode_reflect;

  // Ambient light. LED off
  static const std::string mode_ambient;


  // A measurement of the reflected light intensity, as a percentage.
  float reflected_light_intensity() {
    set_mode(mode_reflect);
    return float_value(0);
  }

  // A measurement of the ambient light intensity, as a percentage.
  float ambient_light_intensity() {
    set_mode(mode_ambient);
    return float_value(0);
  }

};

//~autogen

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.motor>currentClass

// The motor class provides a uniform interface for using motors with
// positional and directional feedback such as the EV3 and NXT motors.
// This feedback allows for precise control of the motors. This is the
// most common type of motor, so we just call it `motor`.

//~autogen
class motor : protected device
{
public:
  typedef device_type motor_type;

  motor(address_type);
  motor(address_type, const motor_type&);

  static const motor_type motor_large;
  static const motor_type motor_medium;

  using device::connected;
  using device::device_index;

//~autogen generic-declare-property-value classes.motor>currentClass

  // Run the motor until another command is sent.
  static const std::string command_run_forever;

  // Run to an absolute position specified by `position_sp` and then
  // stop using the command specified in `stop_command`.
  static const std::string command_run_to_abs_pos;

  // Run to a position relative to the current `position` value.
  // The new position will be current `position` + `position_sp`.
  // When the new position is reached, the motor will stop using
  // the command specified by `stop_command`.
  static const std::string command_run_to_rel_pos;

  // Run the motor for the amount of time specified in `time_sp`
  // and then stop the motor using the command specified by `stop_command`.
  static const std::string command_run_timed;

  // Run the motor at the duty cycle specified by `duty_cycle_sp`.
  // Unlike other run commands, changing `duty_cycle_sp` while running *will*
  // take effect immediately.
  static const std::string command_run_direct;

  // Stop any of the run commands before they are complete using the
  // command specified by `stop_command`.
  static const std::string command_stop;

  // Reset all of the motor parameter attributes to their default value.
  // This will also have the effect of stopping the motor.
  static const std::string command_reset;

  // Sets the normal polarity of the rotary encoder.
  static const std::string encoder_polarity_normal;

  // Sets the inversed polarity of the rotary encoder.
  static const std::string encoder_polarity_inversed;

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static const std::string polarity_normal;

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static const std::string polarity_inversed;

  // The motor controller will vary the power supplied to the motor
  // to try to maintain the speed specified in `speed_sp`.
  static const std::string speed_regulation_on;

  // The motor controller will use the power specified in `duty_cycle_sp`.
  static const std::string speed_regulation_off;

  // Power will be removed from the motor and it will freely coast to a stop.
  static const std::string stop_command_coast;

  // Power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting.
  static const std::string stop_command_brake;

  // Does not remove power from the motor. Instead it actively try to hold the motor
  // at the current position. If an external force tries to turn the motor, the motor
  // will ``push back`` to maintain its position.
  static const std::string stop_command_hold;


//~autogen

//~autogen generic-get-set classes.motor>currentClass

  // Command: write-only
  // Sends a command to the motor controller. See `commands` for a list of
  // possible values.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Commands: read-only
  // Returns a list of commands that are supported by the motor
  // controller. Possible values are `run-forever`, `run-to-abs-pos`, `run-to-rel-pos`,
  // `run-timed`, `run-direct`, `stop` and `reset`. Not all commands may be supported.
  // 
  // - `run-forever` will cause the motor to run until another command is sent.
  // - `run-to-abs-pos` will run to an absolute position specified by `position_sp`
  //   and then stop using the command specified in `stop_command`.
  // - `run-to-rel-pos` will run to a position relative to the current `position` value.
  //   The new position will be current `position` + `position_sp`. When the new
  //   position is reached, the motor will stop using the command specified by `stop_command`.
  // - `run-timed` will run the motor for the amount of time specified in `time_sp`
  //   and then stop the motor using the command specified by `stop_command`.
  // - `run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.
  //   Unlike other run commands, changing `duty_cycle_sp` while running *will*
  //   take effect immediately.
  // - `stop` will stop any of the run commands before they are complete using the
  //   command specified by `stop_command`.
  // - `reset` will reset all of the motor parameter attributes to their default value.
  //   This will also have the effect of stopping the motor.
  mode_set commands() const { return get_attr_set("commands"); }

  // Count Per Rot: read-only
  // Returns the number of tacho counts in one rotation of the motor. Tacho counts
  // are used by the position and speed attributes, so you can use this value
  // to convert rotations or degrees to tacho counts. In the case of linear
  // actuators, the units here will be counts per centimeter.
  int count_per_rot() const { return get_attr_int("count_per_rot"); }

  // Driver Name: read-only
  // Returns the name of the driver that provides this tacho motor device.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Duty Cycle: read-only
  // Returns the current duty cycle of the motor. Units are percent. Values
  // are -100 to 100.
  int duty_cycle() const { return get_attr_int("duty_cycle"); }

  // Duty Cycle SP: read/write
  // Writing sets the duty cycle setpoint. Reading returns the current value.
  // Units are in percent. Valid values are -100 to 100. A negative value causes
  // the motor to rotate in reverse. This value is only used when `speed_regulation`
  // is off.
  int duty_cycle_sp() const { return get_attr_int("duty_cycle_sp"); }
  auto set_duty_cycle_sp(int v) -> decltype(*this) {
    set_attr_int("duty_cycle_sp", v);
    return *this;
  }

  // Encoder Polarity: read/write
  // Sets the polarity of the rotary encoder. This is an advanced feature to all
  // use of motors that send inversed encoder signals to the EV3. This should
  // be set correctly by the driver of a device. It You only need to change this
  // value if you are using a unsupported device. Valid values are `normal` and
  // `inversed`.
  std::string encoder_polarity() const { return get_attr_string("encoder_polarity"); }
  auto set_encoder_polarity(std::string v) -> decltype(*this) {
    set_attr_string("encoder_polarity", v);
    return *this;
  }

  // Polarity: read/write
  // Sets the polarity of the motor. With `normal` polarity, a positive duty
  // cycle will cause the motor to rotate clockwise. With `inversed` polarity,
  // a positive duty cycle will cause the motor to rotate counter-clockwise.
  // Valid values are `normal` and `inversed`.
  std::string polarity() const { return get_attr_string("polarity"); }
  auto set_polarity(std::string v) -> decltype(*this) {
    set_attr_string("polarity", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

  // Position: read/write
  // Returns the current position of the motor in pulses of the rotary
  // encoder. When the motor rotates clockwise, the position will increase.
  // Likewise, rotating counter-clockwise causes the position to decrease.
  // Writing will set the position to that value.
  int position() const { return get_attr_int("position"); }
  auto set_position(int v) -> decltype(*this) {
    set_attr_int("position", v);
    return *this;
  }

  // Position P: read/write
  // The proportional constant for the position PID.
  int position_p() const { return get_attr_int("hold_pid/Kp"); }
  auto set_position_p(int v) -> decltype(*this) {
    set_attr_int("hold_pid/Kp", v);
    return *this;
  }

  // Position I: read/write
  // The integral constant for the position PID.
  int position_i() const { return get_attr_int("hold_pid/Ki"); }
  auto set_position_i(int v) -> decltype(*this) {
    set_attr_int("hold_pid/Ki", v);
    return *this;
  }

  // Position D: read/write
  // The derivative constant for the position PID.
  int position_d() const { return get_attr_int("hold_pid/Kd"); }
  auto set_position_d(int v) -> decltype(*this) {
    set_attr_int("hold_pid/Kd", v);
    return *this;
  }

  // Position SP: read/write
  // Writing specifies the target position for the `run-to-abs-pos` and `run-to-rel-pos`
  // commands. Reading returns the current value. Units are in tacho counts. You
  // can use the value returned by `counts_per_rot` to convert tacho counts to/from
  // rotations or degrees.
  int position_sp() const { return get_attr_int("position_sp"); }
  auto set_position_sp(int v) -> decltype(*this) {
    set_attr_int("position_sp", v);
    return *this;
  }

  // Speed: read-only
  // Returns the current motor speed in tacho counts per second. Not, this is
  // not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`
  // attribute to convert this value to RPM or deg/sec.
  int speed() const { return get_attr_int("speed"); }

  // Speed SP: read/write
  // Writing sets the target speed in tacho counts per second used when `speed_regulation`
  // is on. Reading returns the current value.  Use the `count_per_rot` attribute
  // to convert RPM or deg/sec to tacho counts per second.
  int speed_sp() const { return get_attr_int("speed_sp"); }
  auto set_speed_sp(int v) -> decltype(*this) {
    set_attr_int("speed_sp", v);
    return *this;
  }

  // Ramp Up SP: read/write
  // Writing sets the ramp up setpoint. Reading returns the current value. Units
  // are in milliseconds. When set to a value > 0, the motor will ramp the power
  // sent to the motor from 0 to 100% duty cycle over the span of this setpoint
  // when starting the motor. If the maximum duty cycle is limited by `duty_cycle_sp`
  // or speed regulation, the actual ramp time duration will be less than the setpoint.
  int ramp_up_sp() const { return get_attr_int("ramp_up_sp"); }
  auto set_ramp_up_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_up_sp", v);
    return *this;
  }

  // Ramp Down SP: read/write
  // Writing sets the ramp down setpoint. Reading returns the current value. Units
  // are in milliseconds. When set to a value > 0, the motor will ramp the power
  // sent to the motor from 100% duty cycle down to 0 over the span of this setpoint
  // when stopping the motor. If the starting duty cycle is less than 100%, the
  // ramp time duration will be less than the full span of the setpoint.
  int ramp_down_sp() const { return get_attr_int("ramp_down_sp"); }
  auto set_ramp_down_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_down_sp", v);
    return *this;
  }

  // Speed Regulation Enabled: read/write
  // Turns speed regulation on or off. If speed regulation is on, the motor
  // controller will vary the power supplied to the motor to try to maintain the
  // speed specified in `speed_sp`. If speed regulation is off, the controller
  // will use the power specified in `duty_cycle_sp`. Valid values are `on` and
  // `off`.
  std::string speed_regulation_enabled() const { return get_attr_string("speed_regulation"); }
  auto set_speed_regulation_enabled(std::string v) -> decltype(*this) {
    set_attr_string("speed_regulation", v);
    return *this;
  }

  // Speed Regulation P: read/write
  // The proportional constant for the speed regulation PID.
  int speed_regulation_p() const { return get_attr_int("speed_pid/Kp"); }
  auto set_speed_regulation_p(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Kp", v);
    return *this;
  }

  // Speed Regulation I: read/write
  // The integral constant for the speed regulation PID.
  int speed_regulation_i() const { return get_attr_int("speed_pid/Ki"); }
  auto set_speed_regulation_i(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Ki", v);
    return *this;
  }

  // Speed Regulation D: read/write
  // The derivative constant for the speed regulation PID.
  int speed_regulation_d() const { return get_attr_int("speed_pid/Kd"); }
  auto set_speed_regulation_d(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Kd", v);
    return *this;
  }

  // State: read-only
  // Reading returns a list of state flags. Possible flags are
  // `running`, `ramping` `holding` and `stalled`.
  mode_set state() const { return get_attr_set("state"); }

  // Stop Command: read/write
  // Reading returns the current stop command. Writing sets the stop command.
  // The value determines the motors behavior when `command` is set to `stop`.
  // Also, it determines the motors behavior when a run command completes. See
  // `stop_commands` for a list of possible values.
  std::string stop_command() const { return get_attr_string("stop_command"); }
  auto set_stop_command(std::string v) -> decltype(*this) {
    set_attr_string("stop_command", v);
    return *this;
  }

  // Stop Commands: read-only
  // Returns a list of stop modes supported by the motor controller.
  // Possible values are `coast`, `brake` and `hold`. `coast` means that power will
  // be removed from the motor and it will freely coast to a stop. `brake` means
  // that power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting. `hold` does not remove
  // power from the motor. Instead it actively try to hold the motor at the current
  // position. If an external force tries to turn the motor, the motor will 'push
  // back' to maintain its position.
  mode_set stop_commands() const { return get_attr_set("stop_commands"); }

  // Time SP: read/write
  // Writing specifies the amount of time the motor will run when using the
  // `run-timed` command. Reading returns the current value. Units are in
  // milliseconds.
  int time_sp() const { return get_attr_int("time_sp"); }
  auto set_time_sp(int v) -> decltype(*this) {
    set_attr_int("time_sp", v);
    return *this;
  }


//~autogen

//~autogen motor_commands classes.motor>currentClass

    // Run the motor until another command is sent.
    void run_forever() { set_command("run-forever"); }

    // Run to an absolute position specified by `position_sp` and then
    // stop using the command specified in `stop_command`.
    void run_to_abs_pos() { set_command("run-to-abs-pos"); }

    // Run to a position relative to the current `position` value.
    // The new position will be current `position` + `position_sp`.
    // When the new position is reached, the motor will stop using
    // the command specified by `stop_command`.
    void run_to_rel_pos() { set_command("run-to-rel-pos"); }

    // Run the motor for the amount of time specified in `time_sp`
    // and then stop the motor using the command specified by `stop_command`.
    void run_timed() { set_command("run-timed"); }

    // Run the motor at the duty cycle specified by `duty_cycle_sp`.
    // Unlike other run commands, changing `duty_cycle_sp` while running *will*
    // take effect immediately.
    void run_direct() { set_command("run-direct"); }

    // Stop any of the run commands before they are complete using the
    // command specified by `stop_command`.
    void stop() { set_command("stop"); }

    // Reset all of the motor parameter attributes to their default value.
    // This will also have the effect of stopping the motor.
    void reset() { set_command("reset"); }


//~autogen

protected:
  motor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
};

//-----------------------------------------------------------------------------

// EV3 medium motor
class medium_motor : public motor
{
public:
  medium_motor(address_type address = OUTPUT_AUTO);
};

//-----------------------------------------------------------------------------

// EV3 large motor
class large_motor : public motor
{
public:
  large_motor(address_type address = OUTPUT_AUTO);
};

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.dcMotor>currentClass

// The DC motor class provides a uniform interface for using regular DC motors
// with no fancy controls or feedback. This includes LEGO MINDSTORMS RCX motors
// and LEGO Power Functions motors.

//~autogen
class dc_motor : protected device
{
public:
  dc_motor(address_type address = OUTPUT_AUTO);

  using device::connected;
  using device::device_index;

//~autogen generic-declare-property-value classes.dcMotor>currentClass

  // Run the motor until another command is sent.
  static const std::string command_run_forever;

  // Run the motor for the amount of time specified in `time_sp`
  // and then stop the motor using the command specified by `stop_command`.
  static const std::string command_run_timed;

  // Run the motor at the duty cycle specified by `duty_cycle_sp`.
  // Unlike other run commands, changing `duty_cycle_sp` while running *will*
  // take effect immediately.
  static const std::string command_run_direct;

  // Stop any of the run commands before they are complete using the
  // command specified by `stop_command`.
  static const std::string command_stop;

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static const std::string polarity_normal;

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static const std::string polarity_inversed;

  // Power will be removed from the motor and it will freely coast to a stop.
  static const std::string stop_command_coast;

  // Power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting.
  static const std::string stop_command_brake;


//~autogen

//~autogen generic-get-set classes.dcMotor>currentClass

  // Command: write-only
  // Sets the command for the motor. Possible values are `run-forever`, `run-timed` and
  // `stop`. Not all commands may be supported, so be sure to check the contents
  // of the `commands` attribute.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Commands: read-only
  // Returns a list of commands supported by the motor
  // controller.
  mode_set commands() const { return get_attr_set("commands"); }

  // Driver Name: read-only
  // Returns the name of the motor driver that loaded this device. See the list
  // of [supported devices] for a list of drivers.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Duty Cycle: read-only
  // Shows the current duty cycle of the PWM signal sent to the motor. Values
  // are -100 to 100 (-100% to 100%).
  int duty_cycle() const { return get_attr_int("duty_cycle"); }

  // Duty Cycle SP: read/write
  // Writing sets the duty cycle setpoint of the PWM signal sent to the motor.
  // Valid values are -100 to 100 (-100% to 100%). Reading returns the current
  // setpoint.
  int duty_cycle_sp() const { return get_attr_int("duty_cycle_sp"); }
  auto set_duty_cycle_sp(int v) -> decltype(*this) {
    set_attr_int("duty_cycle_sp", v);
    return *this;
  }

  // Polarity: read/write
  // Sets the polarity of the motor. Valid values are `normal` and `inversed`.
  std::string polarity() const { return get_attr_string("polarity"); }
  auto set_polarity(std::string v) -> decltype(*this) {
    set_attr_string("polarity", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

  // Ramp Down SP: read/write
  // Sets the time in milliseconds that it take the motor to ramp down from 100%
  // to 0%. Valid values are 0 to 10000 (10 seconds). Default is 0.
  int ramp_down_sp() const { return get_attr_int("ramp_down_sp"); }
  auto set_ramp_down_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_down_sp", v);
    return *this;
  }

  // Ramp Up SP: read/write
  // Sets the time in milliseconds that it take the motor to up ramp from 0% to
  // 100%. Valid values are 0 to 10000 (10 seconds). Default is 0.
  int ramp_up_sp() const { return get_attr_int("ramp_up_sp"); }
  auto set_ramp_up_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_up_sp", v);
    return *this;
  }

  // State: read-only
  // Gets a list of flags indicating the motor status. Possible
  // flags are `running` and `ramping`. `running` indicates that the motor is
  // powered. `ramping` indicates that the motor has not yet reached the
  // `duty_cycle_sp`.
  mode_set state() const { return get_attr_set("state"); }

  // Stop Command: write-only
  // Sets the stop command that will be used when the motor stops. Read
  // `stop_commands` to get the list of valid values.
  auto set_stop_command(std::string v) -> decltype(*this) {
    set_attr_string("stop_command", v);
    return *this;
  }

  // Stop Commands: read-only
  // Gets a list of stop commands. Valid values are `coast`
  // and `brake`.
  mode_set stop_commands() const { return get_attr_set("stop_commands"); }

  // Time SP: read/write
  // Writing specifies the amount of time the motor will run when using the
  // `run-timed` command. Reading returns the current value. Units are in
  // milliseconds.
  int time_sp() const { return get_attr_int("time_sp"); }
  auto set_time_sp(int v) -> decltype(*this) {
    set_attr_int("time_sp", v);
    return *this;
  }


//~autogen

//~autogen motor_commands classes.dcMotor>currentClass

    // Run the motor until another command is sent.
    void run_forever() { set_command("run-forever"); }

    // Run the motor for the amount of time specified in `time_sp`
    // and then stop the motor using the command specified by `stop_command`.
    void run_timed() { set_command("run-timed"); }

    // Run the motor at the duty cycle specified by `duty_cycle_sp`.
    // Unlike other run commands, changing `duty_cycle_sp` while running *will*
    // take effect immediately.
    void run_direct() { set_command("run-direct"); }

    // Stop any of the run commands before they are complete using the
    // command specified by `stop_command`.
    void stop() { set_command("stop"); }


//~autogen

protected:
  std::string _port_name;
};

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.servoMotor>currentClass

// The servo motor class provides a uniform interface for using hobby type
// servo motors.

//~autogen
class servo_motor : protected device
{
public:
  servo_motor(address_type address = OUTPUT_AUTO);

  using device::connected;
  using device::device_index;

//~autogen generic-declare-property-value classes.servoMotor>currentClass

  // Drive servo to the position set in the `position_sp` attribute.
  static const std::string command_run;

  // Remove power from the motor.
  static const std::string command_float;

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static const std::string polarity_normal;

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static const std::string polarity_inversed;


//~autogen

//~autogen generic-get-set classes.servoMotor>currentClass

  // Command: write-only
  // Sets the command for the servo. Valid values are `run` and `float`. Setting
  // to `run` will cause the servo to be driven to the position_sp set in the
  // `position_sp` attribute. Setting to `float` will remove power from the motor.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Driver Name: read-only
  // Returns the name of the motor driver that loaded this device. See the list
  // of [supported devices] for a list of drivers.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Max Pulse SP: read/write
  // Used to set the pulse size in milliseconds for the signal that tells the
  // servo to drive to the maximum (clockwise) position_sp. Default value is 2400.
  // Valid values are 2300 to 2700. You must write to the position_sp attribute for
  // changes to this attribute to take effect.
  int max_pulse_sp() const { return get_attr_int("max_pulse_sp"); }
  auto set_max_pulse_sp(int v) -> decltype(*this) {
    set_attr_int("max_pulse_sp", v);
    return *this;
  }

  // Mid Pulse SP: read/write
  // Used to set the pulse size in milliseconds for the signal that tells the
  // servo to drive to the mid position_sp. Default value is 1500. Valid
  // values are 1300 to 1700. For example, on a 180 degree servo, this would be
  // 90 degrees. On continuous rotation servo, this is the 'neutral' position_sp
  // where the motor does not turn. You must write to the position_sp attribute for
  // changes to this attribute to take effect.
  int mid_pulse_sp() const { return get_attr_int("mid_pulse_sp"); }
  auto set_mid_pulse_sp(int v) -> decltype(*this) {
    set_attr_int("mid_pulse_sp", v);
    return *this;
  }

  // Min Pulse SP: read/write
  // Used to set the pulse size in milliseconds for the signal that tells the
  // servo to drive to the miniumum (counter-clockwise) position_sp. Default value
  // is 600. Valid values are 300 to 700. You must write to the position_sp
  // attribute for changes to this attribute to take effect.
  int min_pulse_sp() const { return get_attr_int("min_pulse_sp"); }
  auto set_min_pulse_sp(int v) -> decltype(*this) {
    set_attr_int("min_pulse_sp", v);
    return *this;
  }

  // Polarity: read/write
  // Sets the polarity of the servo. Valid values are `normal` and `inversed`.
  // Setting the value to `inversed` will cause the position_sp value to be
  // inversed. i.e `-100` will correspond to `max_pulse_sp`, and `100` will
  // correspond to `min_pulse_sp`.
  std::string polarity() const { return get_attr_string("polarity"); }
  auto set_polarity(std::string v) -> decltype(*this) {
    set_attr_string("polarity", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

  // Position SP: read/write
  // Reading returns the current position_sp of the servo. Writing instructs the
  // servo to move to the specified position_sp. Units are percent. Valid values
  // are -100 to 100 (-100% to 100%) where `-100` corresponds to `min_pulse_sp`,
  // `0` corresponds to `mid_pulse_sp` and `100` corresponds to `max_pulse_sp`.
  int position_sp() const { return get_attr_int("position_sp"); }
  auto set_position_sp(int v) -> decltype(*this) {
    set_attr_int("position_sp", v);
    return *this;
  }

  // Rate SP: read/write
  // Sets the rate_sp at which the servo travels from 0 to 100.0% (half of the full
  // range of the servo). Units are in milliseconds. Example: Setting the rate_sp
  // to 1000 means that it will take a 180 degree servo 2 second to move from 0
  // to 180 degrees. Note: Some servo controllers may not support this in which
  // case reading and writing will fail with `-EOPNOTSUPP`. In continuous rotation
  // servos, this value will affect the rate_sp at which the speed ramps up or down.
  int rate_sp() const { return get_attr_int("rate_sp"); }
  auto set_rate_sp(int v) -> decltype(*this) {
    set_attr_int("rate_sp", v);
    return *this;
  }

  // State: read-only
  // Returns a list of flags indicating the state of the servo.
  // Possible values are:
  // * `running`: Indicates that the motor is powered.
  mode_set state() const { return get_attr_set("state"); }


//~autogen

//~autogen motor_commands classes.servoMotor>currentClass

    // Drive servo to the position set in the `position_sp` attribute.
    void run() { set_command("run"); }

    // Remove power from the motor.
    void float_() { set_command("float"); }


//~autogen
};

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.led>currentClass

// Any device controlled by the generic LED driver.
// See https://www.kernel.org/doc/Documentation/leds/leds-class.txt
// for more details.

//~autogen
class led : protected device
{
public:
  led(std::string name);

  using device::connected;

//~autogen generic-get-set classes.led>currentClass

  // Max Brightness: read-only
  // Returns the maximum allowable brightness value.
  int max_brightness() const { return get_attr_int("max_brightness"); }

  // Brightness: read/write
  // Sets the brightness level. Possible values are from 0 to `max_brightness`.
  int brightness() const { return get_attr_int("brightness"); }
  auto set_brightness(int v) -> decltype(*this) {
    set_attr_int("brightness", v);
    return *this;
  }

  // Triggers: read-only
  // Returns a list of available triggers.
  mode_set triggers() const { return get_attr_set("trigger"); }

  // Trigger: read/write
  // Sets the led trigger. A trigger
  // is a kernel based source of led events. Triggers can either be simple or
  // complex. A simple trigger isn't configurable and is designed to slot into
  // existing subsystems with minimal additional code. Examples are the `ide-disk` and
  // `nand-disk` triggers.
  // 
  // Complex triggers whilst available to all LEDs have LED specific
  // parameters and work on a per LED basis. The `timer` trigger is an example.
  // The `timer` trigger will periodically change the LED brightness between
  // 0 and the current brightness setting. The `on` and `off` time can
  // be specified via `delay_{on,off}` attributes in milliseconds.
  // You can change the brightness value of a LED independently of the timer
  // trigger. However, if you set the brightness value to 0 it will
  // also disable the `timer` trigger.
  std::string trigger() const { return get_attr_from_set("trigger"); }
  auto set_trigger(std::string v) -> decltype(*this) {
    set_attr_string("trigger", v);
    return *this;
  }

  // Delay On: read/write
  // The `timer` trigger will periodically change the LED brightness between
  // 0 and the current brightness setting. The `on` time can
  // be specified via `delay_on` attribute in milliseconds.
  int delay_on() const { return get_attr_int("delay_on"); }
  auto set_delay_on(int v) -> decltype(*this) {
    set_attr_int("delay_on", v);
    return *this;
  }

  // Delay Off: read/write
  // The `timer` trigger will periodically change the LED brightness between
  // 0 and the current brightness setting. The `off` time can
  // be specified via `delay_off` attribute in milliseconds.
  int delay_off() const { return get_attr_int("delay_off"); }
  auto set_delay_off(int v) -> decltype(*this) {
    set_attr_int("delay_off", v);
    return *this;
  }


//~autogen

  // Gets the LED's brightness as a percentage (0-1) of the maximum.
  float brightness_pct() const {
    return static_cast<float>(brightness()) / max_brightness();
  }

  // Sets the LED's brightness as a percentage (0-1) of the maximum.
  auto set_brightness_pct(float v) -> decltype(*this) {
    return set_brightness(v * max_brightness());
  }

  // Turns the led on by setting its brightness to the maximum level.
  void on()  { set_brightness(max_brightness()); }

  // Turns the led off.
  void off() { set_brightness(0); }

  // Enables timer trigger and sets delay_on and delay_off attributes to the
  // provided values (in milliseconds).
  void flash(unsigned on_ms, unsigned off_ms);

#ifdef EV3DEV_PLATFORM_BRICKPI
//~autogen leds-declare platforms.brickpi.led>currentClass

    static led blue_led1;
    static led blue_led2;

    static std::vector<led*> led1;
    static std::vector<led*> led2;

    static std::vector<float> blue;

//~autogen
#else
//~autogen leds-declare platforms.ev3.led>currentClass

    static led red_left;
    static led red_right;
    static led green_left;
    static led green_right;

    static std::vector<led*> left;
    static std::vector<led*> right;

    static std::vector<float> red;
    static std::vector<float> green;
    static std::vector<float> amber;
    static std::vector<float> orange;
    static std::vector<float> yellow;

//~autogen
#endif

  // Assigns to each led in `group` corresponding brightness percentage from `color`.
  static void set_color(const std::vector<led*> &group, const std::vector<float> &color);

  static void all_off();

protected:
  int _max_brightness = 0;
};

//-----------------------------------------------------------------------------

//~autogen generic-class-description classes.powerSupply>currentClass

// A generic interface to read data from the system's power_supply class.
// Uses the built-in legoev3-battery if none is specified.

//~autogen
class power_supply : protected device
{
public:
  power_supply(std::string name);

  using device::connected;

//~autogen generic-get-set classes.powerSupply>currentClass

  // Measured Current: read-only
  // The measured current that the battery is supplying (in microamps)
  int measured_current() const { return get_attr_int("current_now"); }

  // Measured Voltage: read-only
  // The measured voltage that the battery is supplying (in microvolts)
  int measured_voltage() const { return get_attr_int("voltage_now"); }

  // Max Voltage: read-only
  int max_voltage() const { return get_attr_int("voltage_max_design"); }

  // Min Voltage: read-only
  int min_voltage() const { return get_attr_int("voltage_min_design"); }

  // Technology: read-only
  std::string technology() const { return get_attr_string("technology"); }

  // Type: read-only
  std::string type() const { return get_attr_string("type"); }


//~autogen

  float measured_amps()       const { return measured_current() / 1000000.f; }
  float measured_volts()      const { return measured_voltage() / 1000000.f; }

  static power_supply battery;
};

//-----------------------------------------------------------------------------

// EV3 buttons
class button
{
public:
  button(int bit);

  // Check if the button is pressed.
  bool pressed() const;

  // Gets called whenever the button state changes.
  // The user has to call the process() function to check for state change.
  std::function<void(bool)> onclick;

  // Check if the button state has changed,
  // call onclick function in case it has.
  // Returns true if the state has changed since the last call.
  bool process();

  static button back;
  static button left;
  static button right;
  static button up;
  static button down;
  static button enter;

  // Call process() for each of the EV3 buttons.
  // Returns true if any of the states have changed since the last call.
  static bool process_all();

private:
  int _bit;
  bool _state = false;
  std::vector<unsigned long> _buf;

  struct file_descriptor {
    int _fd;

    file_descriptor(const char *path, int flags);
    ~file_descriptor();
    operator int() { return _fd; }
  };

  std::shared_ptr<file_descriptor> _fd;
};

//-----------------------------------------------------------------------------

// EV3 Sound
class sound
{
public:
  static void beep(const std::string &args = "", bool bSynchronous = false);
  static void tone(float frequency, float ms, bool bSynchronous = false);
  static void tone(const std::vector< std::vector<float> > &sequence, bool bSynchronous = false);
  static void play(const std::string &soundfile, bool bSynchronous = false);
  static void speak(const std::string &text, bool bSynchronous = false);
};

//-----------------------------------------------------------------------------

// EV3 LCD
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

// EV3 remote control
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

//~autogen generic-class-description classes.legoPort>currentClass

// The `lego-port` class provides an interface for working with input and
// output ports that are compatible with LEGO MINDSTORMS RCX/NXT/EV3, LEGO
// WeDo and LEGO Power Functions sensors and motors. Supported devices include
// the LEGO MINDSTORMS EV3 Intelligent Brick, the LEGO WeDo USB hub and
// various sensor multiplexers from 3rd party manufacturers.
// 
// Some types of ports may have multiple modes of operation. For example, the
// input ports on the EV3 brick can communicate with sensors using UART, I2C
// or analog validate signals - but not all at the same time. Therefore there
// are multiple modes available to connect to the different types of sensors.
// 
// In most cases, ports are able to automatically detect what type of sensor
// or motor is connected. In some cases though, this must be manually specified
// using the `mode` and `set_device` attributes. The `mode` attribute affects
// how the port communicates with the connected device. For example the input
// ports on the EV3 brick can communicate using UART, I2C or analog voltages,
// but not all at the same time, so the mode must be set to the one that is
// appropriate for the connected sensor. The `set_device` attribute is used to
// specify the exact type of sensor that is connected. Note: the mode must be
// correctly set before setting the sensor type.
// 
// Ports can be found at `/sys/class/lego-port/port<N>` where `<N>` is
// incremented each time a new port is registered. Note: The number is not
// related to the actual port at all - use the `address` attribute to find
// a specific port.

//~autogen
class lego_port : protected device
{
public:
  lego_port(address_type);

  using device::connected;
  using device::device_index;

//~autogen generic-get-set classes.legoPort>currentClass

  // Driver Name: read-only
  // Returns the name of the driver that loaded this device. You can find the
  // complete list of drivers in the [list of port drivers].
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Modes: read-only
  // Returns a list of the available modes of the port.
  mode_set modes() const { return get_attr_set("modes"); }

  // Mode: read/write
  // Reading returns the currently selected mode. Writing sets the mode.
  // Generally speaking when the mode changes any sensor or motor devices
  // associated with the port will be removed new ones loaded, however this
  // this will depend on the individual driver implementing this class.
  std::string mode() const { return get_attr_string("mode"); }
  auto set_mode(std::string v) -> decltype(*this) {
    set_attr_string("mode", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port. See individual driver documentation for
  // the name that will be returned.
  std::string address() const { return get_attr_string("address"); }

  // Set Device: write-only
  // For modes that support it, writing the name of a driver will cause a new
  // device to be registered for that driver and attached to this port. For
  // example, since NXT/Analog sensors cannot be auto-detected, you must use
  // this attribute to load the correct driver. Returns -EOPNOTSUPP if setting a
  // device is not supported.
  auto set_set_device(std::string v) -> decltype(*this) {
    set_attr_string("set_device", v);
    return *this;
  }

  // Status: read-only
  // In most cases, reading status will return the same value as `mode`. In
  // cases where there is an `auto` mode additional values may be returned,
  // such as `no-device` or `error`. See individual port driver documentation
  // for the full list of possible values.
  std::string status() const { return get_attr_string("status"); }


//~autogen

protected:
  lego_port() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
};

//-----------------------------------------------------------------------------

} // namespace ev3dev

// vim: sw=2
