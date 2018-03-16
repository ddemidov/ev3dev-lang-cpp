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

// Sections of the following code were auto-generated based on spec v1.2.0.

//~autogen
//-----------------------------------------------------------------------------

#include <map>
#include <set>
#include <string>
#include <tuple>
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

#if defined(EV3DEV_PLATFORM_BRICKPI)
constexpr char INPUT_1[]  = "ttyAMA0:in1";  //!< Sensor port 1
constexpr char INPUT_2[]  = "ttyAMA0:in2";  //!< Sensor port 2
constexpr char INPUT_3[]  = "ttyAMA0:in3";  //!< Sensor port 3
constexpr char INPUT_4[]  = "ttyAMA0:in4";  //!< Sensor port 4

constexpr char OUTPUT_A[] = "ttyAMA0:outA"; //!< Motor port A
constexpr char OUTPUT_B[] = "ttyAMA0:outB"; //!< Motor port B
constexpr char OUTPUT_C[] = "ttyAMA0:outC"; //!< Motor port C
constexpr char OUTPUT_D[] = "ttyAMA0:outD"; //!< Motor port D
#elif defined(EV3DEV_PLATFORM_BRICKPI3)
constexpr char INPUT_1[]  = "spi0.1:S1";  //!< Sensor port 1
constexpr char INPUT_2[]  = "spi0.1:S2";  //!< Sensor port 2
constexpr char INPUT_3[]  = "spi0.1:S3";  //!< Sensor port 3
constexpr char INPUT_4[]  = "spi0.1:S4";  //!< Sensor port 4

constexpr char OUTPUT_A[] = "spi0.1:MA"; //!< Motor port A
constexpr char OUTPUT_B[] = "spi0.1:MB"; //!< Motor port B
constexpr char OUTPUT_C[] = "spi0.1:MC"; //!< Motor port C
constexpr char OUTPUT_D[] = "spi0.1:MD"; //!< Motor port D
#elif defined(EV3DEV_PLATFORM_PISTORMS)
constexpr char INPUT_1[]  = "pistorms:BAS1"; //!< Sensor port 1
constexpr char INPUT_2[]  = "pistorms:BAS2"; //!< Sensor port 2
constexpr char INPUT_3[]  = "pistorms:BBS1"; //!< Sensor port 3
constexpr char INPUT_4[]  = "pistorms:BBS2"; //!< Sensor port 4

constexpr char OUTPUT_A[] = "pistorms:BAM1"; //!< Motor port A
constexpr char OUTPUT_B[] = "pistorms:BAM2"; //!< Motor port B
constexpr char OUTPUT_C[] = "pistorms:BBM1"; //!< Motor port C
constexpr char OUTPUT_D[] = "pistorms:BBM2"; //!< Motor port D
#else // assume EV3DEV_PLATFORM_EV3
constexpr char INPUT_1[]  = "ev3-ports:in1";  //!< Sensor port 1
constexpr char INPUT_2[]  = "ev3-ports:in2";  //!< Sensor port 2
constexpr char INPUT_3[]  = "ev3-ports:in3";  //!< Sensor port 3
constexpr char INPUT_4[]  = "ev3-ports:in4";  //!< Sensor port 4

constexpr char OUTPUT_A[] = "ev3-ports:outA"; //!< Motor port A
constexpr char OUTPUT_B[] = "ev3-ports:outB"; //!< Motor port B
constexpr char OUTPUT_C[] = "ev3-ports:outC"; //!< Motor port C
constexpr char OUTPUT_D[] = "ev3-ports:outD"; //!< Motor port D
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

  static constexpr char ev3_touch[]      = "lego-ev3-touch";
  static constexpr char ev3_color[]      = "lego-ev3-color";
  static constexpr char ev3_ultrasonic[] = "lego-ev3-us";
  static constexpr char ev3_gyro[]       = "lego-ev3-gyro";
  static constexpr char ev3_infrared[]   = "lego-ev3-ir";

  static constexpr char nxt_touch[]      = "lego-nxt-touch";
  static constexpr char nxt_light[]      = "lego-nxt-light";
  static constexpr char nxt_sound[]      = "lego-nxt-sound";
  static constexpr char nxt_ultrasonic[] = "lego-nxt-us";
  static constexpr char nxt_i2c_sensor[] = "nxt-i2c-sensor";
  static constexpr char nxt_analog[]     = "nxt-analog";

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
      std::copy_n(_bin_data.data(), _bin_data.size(), reinterpret_cast<char*>(buf));
  }

//~autogen generic-get-set classes.sensor>currentClass

  // Address: read-only
  // Returns the name of the port that the sensor is connected to, e.g. `ev3:in1`.
  // I2C sensors also include the I2C address (decimal), e.g. `ev3:in1:i2c8`.
  std::string address() const { return get_attr_string("address"); }

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
  i2c_sensor(
      address_type address = INPUT_AUTO,
      const std::set<sensor_type> &types = {}
      );

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
  static constexpr char mode_touch[] = "TOUCH";


  // A boolean indicating whether the current touch sensor is being
  // pressed.
  bool is_pressed(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_touch);
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
  static constexpr char mode_col_reflect[] = "COL-REFLECT";

  // Ambient light. Red LEDs off.
  static constexpr char mode_col_ambient[] = "COL-AMBIENT";

  // Color. All LEDs rapidly cycling, appears white.
  static constexpr char mode_col_color[] = "COL-COLOR";

  // Raw reflected. Red LED on
  static constexpr char mode_ref_raw[] = "REF-RAW";

  // Raw Color Components. All LEDs rapidly cycling, appears white.
  static constexpr char mode_rgb_raw[] = "RGB-RAW";

  // No color.
  static constexpr char color_nocolor[] = "NoColor";

  // Black color.
  static constexpr char color_black[] = "Black";

  // Blue color.
  static constexpr char color_blue[] = "Blue";

  // Green color.
  static constexpr char color_green[] = "Green";

  // Yellow color.
  static constexpr char color_yellow[] = "Yellow";

  // Red color.
  static constexpr char color_red[] = "Red";

  // White color.
  static constexpr char color_white[] = "White";

  // Brown color.
  static constexpr char color_brown[] = "Brown";


  // Reflected light intensity as a percentage. Light on sensor is red.
  int reflected_light_intensity(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_col_reflect);
    return value(0);
  }

  // Ambient light intensity. Light on sensor is dimly lit blue.
  int ambient_light_intensity(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_col_ambient);
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
  int color(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_col_color);
    return value(0);
  }

  // Red, green, and blue components of the detected color, in the range 0-1020.
  std::tuple<int, int, int> raw(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_rgb_raw);
    return std::make_tuple( value(0), value(1), value(2) );
  }

  // Red component of the detected color, in the range 0-1020.
  int red(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_rgb_raw);
    return value(0);
  }

  // Green component of the detected color, in the range 0-1020.
  int green(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_rgb_raw);
    return value(1);
  }

  // Blue component of the detected color, in the range 0-1020.
  int blue(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_rgb_raw);
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

  ultrasonic_sensor(address_type address, const std::set<sensor_type>& sensorTypes);

  // Continuous measurement in centimeters.
  static constexpr char mode_us_dist_cm[] = "US-DIST-CM";

  // Continuous measurement in inches.
  static constexpr char mode_us_dist_in[] = "US-DIST-IN";

  // Listen.
  static constexpr char mode_us_listen[] = "US-LISTEN";

  // Single measurement in centimeters.
  static constexpr char mode_us_si_cm[] = "US-SI-CM";

  // Single measurement in inches.
  static constexpr char mode_us_si_in[] = "US-SI-IN";


  // Measurement of the distance detected by the sensor,
  // in centimeters.
  float distance_centimeters(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_us_dist_cm);
    return float_value(0);
  }

  // Measurement of the distance detected by the sensor,
  // in inches.
  float distance_inches(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_us_dist_in);
    return float_value(0);
  }

  // Value indicating whether another ultrasonic sensor could
  // be heard nearby.
  bool other_sensor_present(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_us_listen);
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
  static constexpr char mode_gyro_ang[] = "GYRO-ANG";

  // Rotational speed
  static constexpr char mode_gyro_rate[] = "GYRO-RATE";

  // Raw sensor value
  static constexpr char mode_gyro_fas[] = "GYRO-FAS";

  // Angle and rotational speed
  static constexpr char mode_gyro_g_a[] = "GYRO-G&A";

  // Calibration ???
  static constexpr char mode_gyro_cal[] = "GYRO-CAL";


  // The number of degrees that the sensor has been rotated
  // since it was put into this mode.
  int angle(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_gyro_ang);
    return value(0);
  }

  // The rate at which the sensor is rotating, in degrees/second.
  int rate(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_gyro_rate);
    return value(0);
  }

  // Angle (degrees) and Rotational Speed (degrees/second).
  std::tuple<int, int> rate_and_angle(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_gyro_g_a);
    return std::make_tuple( value(0), value(1) );
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
  static constexpr char mode_ir_prox[] = "IR-PROX";

  // IR Seeker
  static constexpr char mode_ir_seek[] = "IR-SEEK";

  // IR Remote Control
  static constexpr char mode_ir_remote[] = "IR-REMOTE";

  // IR Remote Control. State of the buttons is coded in binary
  static constexpr char mode_ir_rem_a[] = "IR-REM-A";

  // Calibration ???
  static constexpr char mode_ir_cal[] = "IR-CAL";


  // A measurement of the distance between the sensor and the remote,
  // as a percentage. 100% is approximately 70cm/27in.
  int proximity(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_ir_prox);
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
  static constexpr char mode_db[] = "DB";

  // Sound pressure level. A weighting
  static constexpr char mode_dba[] = "DBA";


  // A measurement of the measured sound pressure level, as a
  // percent. Uses a flat weighting.
  float sound_pressure(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_db);
    return float_value(0);
  }

  // A measurement of the measured sound pressure level, as a
  // percent. Uses A-weighting, which focuses on levels up to 55 dB.
  float sound_pressure_low(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_dba);
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
  static constexpr char mode_reflect[] = "REFLECT";

  // Ambient light. LED off
  static constexpr char mode_ambient[] = "AMBIENT";


  // A measurement of the reflected light intensity, as a percentage.
  float reflected_light_intensity(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_reflect);
    return float_value(0);
  }

  // A measurement of the ambient light intensity, as a percentage.
  float ambient_light_intensity(bool do_set_mode = true) {
    if (do_set_mode) set_mode(mode_ambient);
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
// 
// The way to configure a motor is to set the '_sp' attributes when
// calling a command or before. Only in 'run_direct' mode attribute
// changes are processed immediately, in the other modes they only
// take place when a new command is issued.

//~autogen
class motor : protected device
{
public:
  typedef device_type motor_type;

  motor(address_type);
  motor(address_type, const motor_type&);

  static constexpr char motor_large[]  = "lego-ev3-l-motor";
  static constexpr char motor_medium[] = "lego-ev3-m-motor";

  using device::connected;
  using device::device_index;

//~autogen generic-declare-property-value classes.motor>currentClass

  // Run the motor until another command is sent.
  static constexpr char command_run_forever[] = "run-forever";

  // Run to an absolute position specified by `position_sp` and then
  // stop using the action specified in `stop_action`.
  static constexpr char command_run_to_abs_pos[] = "run-to-abs-pos";

  // Run to a position relative to the current `position` value.
  // The new position will be current `position` + `position_sp`.
  // When the new position is reached, the motor will stop using
  // the action specified by `stop_action`.
  static constexpr char command_run_to_rel_pos[] = "run-to-rel-pos";

  // Run the motor for the amount of time specified in `time_sp`
  // and then stop the motor using the action specified by `stop_action`.
  static constexpr char command_run_timed[] = "run-timed";

  // Run the motor at the duty cycle specified by `duty_cycle_sp`.
  // Unlike other run commands, changing `duty_cycle_sp` while running *will*
  // take effect immediately.
  static constexpr char command_run_direct[] = "run-direct";

  // Stop any of the run commands before they are complete using the
  // action specified by `stop_action`.
  static constexpr char command_stop[] = "stop";

  // Reset all of the motor parameter attributes to their default value.
  // This will also have the effect of stopping the motor.
  static constexpr char command_reset[] = "reset";

  // Sets the normal polarity of the rotary encoder.
  static constexpr char encoder_polarity_normal[] = "normal";

  // Sets the inversed polarity of the rotary encoder.
  static constexpr char encoder_polarity_inversed[] = "inversed";

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static constexpr char polarity_normal[] = "normal";

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static constexpr char polarity_inversed[] = "inversed";

  // Power is being sent to the motor.
  static constexpr char state_running[] = "running";

  // The motor is ramping up or down and has not yet reached a constant output level.
  static constexpr char state_ramping[] = "ramping";

  // The motor is not turning, but rather attempting to hold a fixed position.
  static constexpr char state_holding[] = "holding";

  // The motor is turning, but cannot reach its `speed_sp`.
  static constexpr char state_overloaded[] = "overloaded";

  // The motor is not turning when it should be.
  static constexpr char state_stalled[] = "stalled";

  // Power will be removed from the motor and it will freely coast to a stop.
  static constexpr char stop_action_coast[] = "coast";

  // Power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting.
  static constexpr char stop_action_brake[] = "brake";

  // Does not remove power from the motor. Instead it actively try to hold the motor
  // at the current position. If an external force tries to turn the motor, the motor
  // will `push back` to maintain its position.
  static constexpr char stop_action_hold[] = "hold";


//~autogen

//~autogen generic-get-set classes.motor>currentClass

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

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
  //   and then stop using the action specified in `stop_action`.
  // - `run-to-rel-pos` will run to a position relative to the current `position` value.
  //   The new position will be current `position` + `position_sp`. When the new
  //   position is reached, the motor will stop using the action specified by `stop_action`.
  // - `run-timed` will run the motor for the amount of time specified in `time_sp`
  //   and then stop the motor using the action specified by `stop_action`.
  // - `run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.
  //   Unlike other run commands, changing `duty_cycle_sp` while running *will*
  //   take effect immediately.
  // - `stop` will stop any of the run commands before they are complete using the
  //   action specified by `stop_action`.
  // - `reset` will reset all of the motor parameter attributes to their default value.
  //   This will also have the effect of stopping the motor.
  mode_set commands() const { return get_attr_set("commands"); }

  // Count Per Rot: read-only
  // Returns the number of tacho counts in one rotation of the motor. Tacho counts
  // are used by the position and speed attributes, so you can use this value
  // to convert rotations or degrees to tacho counts. (rotation motors only)
  int count_per_rot() const { return get_attr_int("count_per_rot"); }

  // Count Per M: read-only
  // Returns the number of tacho counts in one meter of travel of the motor. Tacho
  // counts are used by the position and speed attributes, so you can use this
  // value to convert from distance to tacho counts. (linear motors only)
  int count_per_m() const { return get_attr_int("count_per_m"); }

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
  // the motor to rotate in reverse.
  int duty_cycle_sp() const { return get_attr_int("duty_cycle_sp"); }
  auto set_duty_cycle_sp(int v) -> decltype(*this) {
    set_attr_int("duty_cycle_sp", v);
    return *this;
  }

  // Full Travel Count: read-only
  // Returns the number of tacho counts in the full travel of the motor. When
  // combined with the `count_per_m` atribute, you can use this value to
  // calculate the maximum travel distance of the motor. (linear motors only)
  int full_travel_count() const { return get_attr_int("full_travel_count"); }

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

  // Max Speed: read-only
  // Returns the maximum value that is accepted by the `speed_sp` attribute. This
  // may be slightly different than the maximum speed that a particular motor can
  // reach - it's the maximum theoretical speed.
  int max_speed() const { return get_attr_int("max_speed"); }

  // Speed: read-only
  // Returns the current motor speed in tacho counts per second. Note, this is
  // not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`
  // attribute to convert this value to RPM or deg/sec.
  int speed() const { return get_attr_int("speed"); }

  // Speed SP: read/write
  // Writing sets the target speed in tacho counts per second used for all `run-*`
  // commands except `run-direct`. Reading returns the current value. A negative
  // value causes the motor to rotate in reverse with the exception of `run-to-*-pos`
  // commands where the sign is ignored. Use the `count_per_rot` attribute to convert
  // RPM or deg/sec to tacho counts per second. Use the `count_per_m` attribute to
  // convert m/s to tacho counts per second.
  int speed_sp() const { return get_attr_int("speed_sp"); }
  auto set_speed_sp(int v) -> decltype(*this) {
    set_attr_int("speed_sp", v);
    return *this;
  }

  // Ramp Up SP: read/write
  // Writing sets the ramp up setpoint. Reading returns the current value. Units
  // are in milliseconds and must be positive. When set to a non-zero value, the
  // motor speed will increase from 0 to 100% of `max_speed` over the span of this
  // setpoint. The actual ramp time is the ratio of the difference between the
  // `speed_sp` and the current `speed` and max_speed multiplied by `ramp_up_sp`.
  int ramp_up_sp() const { return get_attr_int("ramp_up_sp"); }
  auto set_ramp_up_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_up_sp", v);
    return *this;
  }

  // Ramp Down SP: read/write
  // Writing sets the ramp down setpoint. Reading returns the current value. Units
  // are in milliseconds and must be positive. When set to a non-zero value, the
  // motor speed will decrease from 0 to 100% of `max_speed` over the span of this
  // setpoint. The actual ramp time is the ratio of the difference between the
  // `speed_sp` and the current `speed` and max_speed multiplied by `ramp_down_sp`.
  int ramp_down_sp() const { return get_attr_int("ramp_down_sp"); }
  auto set_ramp_down_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_down_sp", v);
    return *this;
  }

  // Speed P: read/write
  // The proportional constant for the speed regulation PID.
  int speed_p() const { return get_attr_int("speed_pid/Kp"); }
  auto set_speed_p(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Kp", v);
    return *this;
  }

  // Speed I: read/write
  // The integral constant for the speed regulation PID.
  int speed_i() const { return get_attr_int("speed_pid/Ki"); }
  auto set_speed_i(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Ki", v);
    return *this;
  }

  // Speed D: read/write
  // The derivative constant for the speed regulation PID.
  int speed_d() const { return get_attr_int("speed_pid/Kd"); }
  auto set_speed_d(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Kd", v);
    return *this;
  }

  // State: read-only
  // Reading returns a list of state flags. Possible flags are
  // `running`, `ramping`, `holding`, `overloaded` and `stalled`.
  mode_set state() const { return get_attr_set("state"); }

  // Stop Action: read/write
  // Reading returns the current stop action. Writing sets the stop action.
  // The value determines the motors behavior when `command` is set to `stop`.
  // Also, it determines the motors behavior when a run command completes. See
  // `stop_actions` for a list of possible values.
  std::string stop_action() const { return get_attr_string("stop_action"); }
  auto set_stop_action(std::string v) -> decltype(*this) {
    set_attr_string("stop_action", v);
    return *this;
  }

  // Stop Actions: read-only
  // Returns a list of stop actions supported by the motor controller.
  // Possible values are `coast`, `brake` and `hold`. `coast` means that power will
  // be removed from the motor and it will freely coast to a stop. `brake` means
  // that power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting. `hold` does not remove
  // power from the motor. Instead it actively tries to hold the motor at the current
  // position. If an external force tries to turn the motor, the motor will 'push
  // back' to maintain its position.
  mode_set stop_actions() const { return get_attr_set("stop_actions"); }

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
    // stop using the action specified in `stop_action`.
    void run_to_abs_pos() { set_command("run-to-abs-pos"); }

    // Run to a position relative to the current `position` value.
    // The new position will be current `position` + `position_sp`.
    // When the new position is reached, the motor will stop using
    // the action specified by `stop_action`.
    void run_to_rel_pos() { set_command("run-to-rel-pos"); }

    // Run the motor for the amount of time specified in `time_sp`
    // and then stop the motor using the action specified by `stop_action`.
    void run_timed() { set_command("run-timed"); }

    // Run the motor at the duty cycle specified by `duty_cycle_sp`.
    // Unlike other run commands, changing `duty_cycle_sp` while running *will*
    // take effect immediately.
    void run_direct() { set_command("run-direct"); }

    // Stop any of the run commands before they are complete using the
    // action specified by `stop_action`.
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
  static constexpr char command_run_forever[] = "run-forever";

  // Run the motor for the amount of time specified in `time_sp`
  // and then stop the motor using the action specified by `stop_action`.
  static constexpr char command_run_timed[] = "run-timed";

  // Run the motor at the duty cycle specified by `duty_cycle_sp`.
  // Unlike other run commands, changing `duty_cycle_sp` while running *will*
  // take effect immediately.
  static constexpr char command_run_direct[] = "run-direct";

  // Stop any of the run commands before they are complete using the
  // action specified by `stop_action`.
  static constexpr char command_stop[] = "stop";

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static constexpr char polarity_normal[] = "normal";

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static constexpr char polarity_inversed[] = "inversed";

  // Power will be removed from the motor and it will freely coast to a stop.
  static constexpr char stop_action_coast[] = "coast";

  // Power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting.
  static constexpr char stop_action_brake[] = "brake";


//~autogen

//~autogen generic-get-set classes.dcMotor>currentClass

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

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

  // Stop Action: write-only
  // Sets the stop action that will be used when the motor stops. Read
  // `stop_actions` to get the list of valid values.
  auto set_stop_action(std::string v) -> decltype(*this) {
    set_attr_string("stop_action", v);
    return *this;
  }

  // Stop Actions: read-only
  // Gets a list of stop actions. Valid values are `coast`
  // and `brake`.
  mode_set stop_actions() const { return get_attr_set("stop_actions"); }

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
    // and then stop the motor using the action specified by `stop_action`.
    void run_timed() { set_command("run-timed"); }

    // Run the motor at the duty cycle specified by `duty_cycle_sp`.
    // Unlike other run commands, changing `duty_cycle_sp` while running *will*
    // take effect immediately.
    void run_direct() { set_command("run-direct"); }

    // Stop any of the run commands before they are complete using the
    // action specified by `stop_action`.
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
  static constexpr char command_run[] = "run";

  // Remove power from the motor.
  static constexpr char command_float[] = "float";

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static constexpr char polarity_normal[] = "normal";

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static constexpr char polarity_inversed[] = "inversed";


//~autogen

//~autogen generic-get-set classes.servoMotor>currentClass

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

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

#if defined(EV3DEV_PLATFORM_BRICKPI)
//~autogen leds-declare platforms.brickpi.led>currentClass

    static led blue_led1;
    static led blue_led2;

    static std::vector<led*> led1;
    static std::vector<led*> led2;

    static std::vector<float> black;
    static std::vector<float> blue;

//~autogen
#elif defined(EV3DEV_PLATFORM_BRICKPI3)

    static led amber_led1;

    static std::vector<led*> led1;

    static std::vector<float> black;
    static std::vector<float> blue;

#elif defined(EV3DEV_PLATFORM_PISTORMS)
//~autogen leds-declare platforms.pistorms.led>currentClass

    static led red_left;
    static led red_right;
    static led green_left;
    static led green_right;
    static led blue_left;
    static led blue_right;

    static std::vector<led*> left;
    static std::vector<led*> right;

    static std::vector<float> black;
    static std::vector<float> red;
    static std::vector<float> green;
    static std::vector<float> blue;
    static std::vector<float> yellow;
    static std::vector<float> purple;
    static std::vector<float> cyan;
    static std::vector<float> white;
    static std::vector<float> orange;

//~autogen
#else
//~autogen leds-declare platforms.ev3.led>currentClass

    static led red_left;
    static led red_right;
    static led green_left;
    static led green_right;

    static std::vector<led*> left;
    static std::vector<led*> right;

    static std::vector<float> black;
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

  // Address: read-only
  // Returns the name of the port. See individual driver documentation for
  // the name that will be returned.
  std::string address() const { return get_attr_string("address"); }

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
