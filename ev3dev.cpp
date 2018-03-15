/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 *
 * Copyright (c) 2014 - Franz Detro
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
 *  Add new button for ev3dev Release 02.00.00 (ev3dev-jessie-2014-07-12) - Christophe Chaudelet
 *
 */

//-----------------------------------------------------------------------------
//~autogen autogen-header

// Sections of the following code were auto-generated based on spec v1.2.0.

//~autogen
//-----------------------------------------------------------------------------

#include "ev3dev.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <list>
#include <map>
#include <array>
#include <algorithm>
#include <system_error>
#include <mutex>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <string.h>
#include <math.h>

#include <dirent.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#ifndef SYS_ROOT
#define SYS_ROOT "/sys/class"
#endif

#ifndef FSTREAM_CACHE_SIZE
#define FSTREAM_CACHE_SIZE 16
#endif

#ifndef NO_LINUX_HEADERS
#include <linux/fb.h>
#include <linux/input.h>
#else
#define KEY_CNT 8
#endif
static const int bits_per_long = sizeof(long) * 8;

//-----------------------------------------------------------------------------

namespace ev3dev {

namespace {

// This class implements a small LRU cache. It assumes the number of elements
// is small, and so uses a simple linear search.
template <typename K, typename V>
class lru_cache
{
private:
  // typedef st::pair<K, V> item;
  // std::pair seems to be missing necessary move constructors :(
  struct item
  {
    K first;
    V second;

    item(const K &k) : first(k) {}
    item(item &&m) : first(std::move(m.first)), second(std::move(m.second)) {}
  };

public:
  lru_cache(size_t size = 3) : _size(size)
  {
  }

  V &operator[] (const K &k)
  {
    iterator i = find(k);
    if (i != _items.end())
    {
      // Found the key, bring the item to the front.
      _items.splice(_items.begin(), _items, i);
    } else {
      // If the cache is full, remove oldest items to make room.
      while (_items.size() + 1 > _size)
      {
        _items.pop_back();
      }
      // Insert a new default constructed value for this new key.
      _items.emplace_front(k);
    }
    // The new item is the most recently used.
    return _items.front().second;
  }

  void clear()
  {
    _items.clear();
  }

private:
  typedef typename std::list<item>::iterator iterator;

  iterator find(const K &k)
  {
    return std::find_if(_items.begin(), _items.end(),
                        [&](const item &i) { return i.first == k; });
  }

  size_t _size;
  std::list<item> _items;
};

// A global cache of files.
std::ifstream& ifstream_cache(const std::string &path) {
  static lru_cache<std::string, std::ifstream> cache(FSTREAM_CACHE_SIZE);
  static std::mutex mx;

  std::lock_guard<std::mutex> lock(mx);
  return cache[path];
}

std::ofstream& ofstream_cache(const std::string &path) {
  static lru_cache<std::string, std::ofstream> cache(FSTREAM_CACHE_SIZE);
  static std::mutex mx;

  std::lock_guard<std::mutex> lock(mx);
  return cache[path];
}

//-----------------------------------------------------------------------------

std::ofstream &ofstream_open(const std::string &path)
{
  std::ofstream &file = ofstream_cache(path);
  if (!file.is_open())
  {
    // Don't buffer writes to avoid latency. Also saves a bit of memory.
    file.rdbuf()->pubsetbuf(NULL, 0);
    file.open(path);
  }
  else
  {
    // Clear the error bits in case something happened.
    file.clear();
  }
  return file;
}

std::ifstream &ifstream_open(const std::string &path)
{
  std::ifstream &file = ifstream_cache(path);
  if (!file.is_open())
  {
    file.open(path);
  }
  else
  {
    // Clear the flags bits in case something happened (like reaching EOF).
    file.clear();
    file.seekg(0, std::ios::beg);
  }
  return file;
}

} // namespace

//-----------------------------------------------------------------------------

bool device::connect(
    const std::string &dir,
    const std::string &pattern,
    const std::map<std::string, std::set<std::string>> &match
    ) noexcept
{
  using namespace std;

  const size_t pattern_length = pattern.length();

  struct dirent *dp;
  DIR *dfd;

  if ((dfd = opendir(dir.c_str())) != nullptr)
  {
    while ((dp = readdir(dfd)) != nullptr)
    {
      if (strncmp(dp->d_name, pattern.c_str(), pattern_length)==0)
      {
        try
        {
          _path = dir + dp->d_name + '/';

          bool bMatch = true;
          for (auto &m : match)
          {
            const auto &attribute = m.first;
            const auto &matches   = m.second;
            const auto strValue   = get_attr_string(attribute);

            if (!matches.empty() && !matches.begin()->empty() &&
                (matches.find(strValue) == matches.end()))
            {
              bMatch = false;
              break;
            }
          }

          if (bMatch) {
            closedir(dfd);
            return true;
          }
        }
        catch (...) { }

        _path.clear();
      }
    }

    closedir(dfd);
  }

  return false;
}

//-----------------------------------------------------------------------------

int device::device_index() const
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  if (_device_index < 0)
  {
    unsigned f = 1;
    _device_index = 0;
    for (auto it=_path.rbegin(); it!=_path.rend(); ++it)
    {
      if(*it =='/')
        continue;		
      if ((*it < '0') || (*it > '9'))
        break;

      _device_index += (*it -'0') * f;
      f *= 10;
    }
  }

  return _device_index;
}

//-----------------------------------------------------------------------------

int device::get_attr_int(const std::string &name) const {
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  for(int attempt = 0; attempt < 2; ++attempt) {
    ifstream &is = ifstream_open(_path + name);
    if (is.is_open())
    {
      int result = 0;
      try {
        is >> result;
        return result;
      } catch(...) {
        // This could mean the sysfs attribute was recreated and the
        // corresponding file handle got stale. Lets close the file and try
        // again (once):
        if (attempt != 0) throw;

        is.close();
        is.clear();
      }
    } else break;
  }
  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

void device::set_attr_int(const std::string &name, int value) {
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  for(int attempt = 0; attempt < 2; ++attempt) {
    ofstream &os = ofstream_open(_path + name);
    if (os.is_open())
    {
      if (os << value) return;

      // An error could mean that sysfs attribute was recreated and the cached
      // file handle is stale. Lets close the file and try again (once):
      if (attempt == 0 && errno == ENODEV) {
        os.close();
        os.clear();
      } else {
        throw system_error(std::error_code(errno, std::system_category()));
      }
    } else {
      throw system_error(make_error_code(errc::no_such_device), _path + name);
    }
  }
}

//-----------------------------------------------------------------------------

std::string device::get_attr_string(const std::string &name) const
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ifstream &is = ifstream_open(_path + name);
  if (is.is_open())
  {
    string result;
    is >> result;
    return result;
  }

  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

void device::set_attr_string(const std::string &name, const std::string &value)
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ofstream &os = ofstream_open(_path + name);
  if (os.is_open())
  {
    if (!(os << value)) throw system_error(std::error_code(errno, std::system_category()));
    return;
  }

  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

std::string device::get_attr_line(const std::string &name) const
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ifstream &is = ifstream_open(_path + name);
  if (is.is_open())
  {
    string result;
    getline(is, result);
    return result;
  }

  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

mode_set device::get_attr_set(const std::string &name,
                              std::string *pCur) const
{
  using namespace std;

  string s = get_attr_line(name);

  mode_set result;
  size_t pos, last_pos = 0;
  string t;
  do {
    pos = s.find(' ', last_pos);

    if (pos != string::npos)
    {
      t = s.substr(last_pos, pos-last_pos);
      last_pos = pos+1;
    }
    else
      t = s.substr(last_pos);

    if (!t.empty())
    {
      if (*t.begin()=='[')
      {
        t = t.substr(1, t.length()-2);
        if (pCur)
          *pCur = t;
      }
      result.insert(t);
    }
  } while (pos!=string::npos);

  return result;
}

//-----------------------------------------------------------------------------

std::string device::get_attr_from_set(const std::string &name) const
{
  using namespace std;

  string s = get_attr_line(name);

  size_t pos, last_pos = 0;
  string t;
  do {
    pos = s.find(' ', last_pos);

    if (pos != string::npos)
    {
      t = s.substr(last_pos, pos-last_pos);
      last_pos = pos+1;
    }
    else
      t = s.substr(last_pos);

    if (!t.empty())
    {
      if (*t.begin()=='[')
      {
        return t.substr(1, t.length()-2);
      }
    }
  } while (pos!=string::npos);

  return { "none" };
}

//-----------------------------------------------------------------------------

constexpr char sensor::ev3_touch[];
constexpr char sensor::ev3_color[];
constexpr char sensor::ev3_ultrasonic[];
constexpr char sensor::ev3_gyro[];
constexpr char sensor::ev3_infrared[];
constexpr char sensor::nxt_touch[];
constexpr char sensor::nxt_light[];
constexpr char sensor::nxt_sound[];
constexpr char sensor::nxt_ultrasonic[];
constexpr char sensor::nxt_i2c_sensor[];
constexpr char sensor::nxt_analog[];

//-----------------------------------------------------------------------------

sensor::sensor(address_type address)
{
  connect({{ "address", { address }}});
}

//-----------------------------------------------------------------------------

sensor::sensor(address_type address, const std::set<sensor_type> &types)
{
  connect({{ "address", { address }},
           { "driver_name", types }});
}

//-----------------------------------------------------------------------------

bool sensor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
{
  static const std::string _strClassDir { SYS_ROOT "/lego-sensor/" };
  static const std::string _strPattern  { "sensor" };

  try
  {
    if (device::connect(_strClassDir, _strPattern, match))
    {
      return true;
    }
  }
  catch (...) { }

  _path.clear();

  return false;
}

//-----------------------------------------------------------------------------

std::string sensor::type_name() const
{
  auto type = driver_name();
  if (type.empty())
  {
    static const std::string s("<none>");
    return s;
  }

  static const std::map<sensor_type, const std::string> lookup_table {
    { ev3_touch,       "EV3 touch" },
    { ev3_color,       "EV3 color" },
    { ev3_ultrasonic,  "EV3 ultrasonic" },
    { ev3_gyro,        "EV3 gyro" },
    { ev3_infrared,    "EV3 infrared" },
    { nxt_touch,       "NXT touch" },
    { nxt_light,       "NXT light" },
    { nxt_sound,       "NXT sound" },
    { nxt_ultrasonic,  "NXT ultrasonic" },
    { nxt_i2c_sensor,  "I2C sensor" },
  };

  auto s = lookup_table.find(type);
  if (s != lookup_table.end())
    return s->second;

  return type;
}

//-----------------------------------------------------------------------------

int sensor::value(unsigned index) const
{
  if (static_cast<int>(index) >= num_values())
    throw std::invalid_argument("index");

  char svalue[7] = "value0";
  svalue[5] += index;

  return get_attr_int(svalue);
}

//-----------------------------------------------------------------------------

float sensor::float_value(unsigned index) const
{
  return value(index) * powf(10, -decimals());
}

//-----------------------------------------------------------------------------
const std::vector<char>& sensor::bin_data() const
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  if (_bin_data.empty()) {
    static const map<string, int> lookup_table {
      {"u8",     1},
      {"s8",     1},
      {"u16",    2},
      {"s16",    2},
      {"s16_be", 2},
      {"s32",    4},
      {"float",  4}
    };

    int value_size = 1;

    auto s = lookup_table.find(bin_data_format());
    if (s != lookup_table.end())
      value_size = s->second;

    _bin_data.resize(num_values() * value_size);
  }

  const string fname = _path + "bin_data";
  ifstream &is = ifstream_open(fname);
  if (is.is_open())
  {
    is.read(_bin_data.data(), _bin_data.size());
    return _bin_data;
  }

  throw system_error(make_error_code(errc::no_such_device), fname);
}

//-----------------------------------------------------------------------------

i2c_sensor::i2c_sensor(address_type address, const std::set<sensor_type> &types) :
  sensor(address, types)
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.touchSensor>currentClass

constexpr char touch_sensor::mode_touch[];

//~autogen

touch_sensor::touch_sensor(address_type address) :
  sensor(address, { ev3_touch, nxt_touch })
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.colorSensor>currentClass

constexpr char color_sensor::mode_col_reflect[];
constexpr char color_sensor::mode_col_ambient[];
constexpr char color_sensor::mode_col_color[];
constexpr char color_sensor::mode_ref_raw[];
constexpr char color_sensor::mode_rgb_raw[];
constexpr char color_sensor::color_nocolor[];
constexpr char color_sensor::color_black[];
constexpr char color_sensor::color_blue[];
constexpr char color_sensor::color_green[];
constexpr char color_sensor::color_yellow[];
constexpr char color_sensor::color_red[];
constexpr char color_sensor::color_white[];
constexpr char color_sensor::color_brown[];

//~autogen

color_sensor::color_sensor(address_type address) :
  sensor(address, { ev3_color })
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.ultrasonicSensor>currentClass

constexpr char ultrasonic_sensor::mode_us_dist_cm[];
constexpr char ultrasonic_sensor::mode_us_dist_in[];
constexpr char ultrasonic_sensor::mode_us_listen[];
constexpr char ultrasonic_sensor::mode_us_si_cm[];
constexpr char ultrasonic_sensor::mode_us_si_in[];

//~autogen

ultrasonic_sensor::ultrasonic_sensor(address_type address) :
  sensor(address, { ev3_ultrasonic, nxt_ultrasonic })
{
}

ultrasonic_sensor::ultrasonic_sensor(address_type address, const std::set<sensor_type>& sensorTypes) :
  sensor(address, sensorTypes)
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.gyroSensor>currentClass

constexpr char gyro_sensor::mode_gyro_ang[];
constexpr char gyro_sensor::mode_gyro_rate[];
constexpr char gyro_sensor::mode_gyro_fas[];
constexpr char gyro_sensor::mode_gyro_g_a[];
constexpr char gyro_sensor::mode_gyro_cal[];

//~autogen

gyro_sensor::gyro_sensor(address_type address) :
  sensor(address, { ev3_gyro })
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.infraredSensor>currentClass

constexpr char infrared_sensor::mode_ir_prox[];
constexpr char infrared_sensor::mode_ir_seek[];
constexpr char infrared_sensor::mode_ir_remote[];
constexpr char infrared_sensor::mode_ir_rem_a[];
constexpr char infrared_sensor::mode_ir_cal[];

//~autogen

infrared_sensor::infrared_sensor(address_type address) :
  sensor(address, { ev3_infrared })
{
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.soundSensor>currentClass

constexpr char sound_sensor::mode_db[];
constexpr char sound_sensor::mode_dba[];

//~autogen

sound_sensor::sound_sensor(address_type address) :
  sensor(address, { nxt_sound, nxt_analog })
{
    if (connected() && driver_name() == nxt_analog) {
        lego_port port(address);

        if (port.connected()) {
            port.set_set_device(nxt_sound);

            if (port.status() != nxt_sound) {
                // Failed to load lego-nxt-sound friver. Wrong port?
                _path.clear();
            }
        } else {
            _path.clear();
        }
    }
}

//-----------------------------------------------------------------------------

//~autogen generic-define-property-value specialSensorTypes.lightSensor>currentClass

constexpr char light_sensor::mode_reflect[];
constexpr char light_sensor::mode_ambient[];

//~autogen

light_sensor::light_sensor(address_type address) :
  sensor(address, { nxt_light })
{
}

//-----------------------------------------------------------------------------

constexpr char motor::motor_large[];
constexpr char motor::motor_medium[];

//~autogen generic-define-property-value classes.motor>currentClass

constexpr char motor::command_run_forever[];
constexpr char motor::command_run_to_abs_pos[];
constexpr char motor::command_run_to_rel_pos[];
constexpr char motor::command_run_timed[];
constexpr char motor::command_run_direct[];
constexpr char motor::command_stop[];
constexpr char motor::command_reset[];
constexpr char motor::encoder_polarity_normal[];
constexpr char motor::encoder_polarity_inversed[];
constexpr char motor::polarity_normal[];
constexpr char motor::polarity_inversed[];
constexpr char motor::state_running[];
constexpr char motor::state_ramping[];
constexpr char motor::state_holding[];
constexpr char motor::state_overloaded[];
constexpr char motor::state_stalled[];
constexpr char motor::stop_action_coast[];
constexpr char motor::stop_action_brake[];
constexpr char motor::stop_action_hold[];

//~autogen

//-----------------------------------------------------------------------------

motor::motor(address_type address)
{
  connect({{ "address", { address } }});
}

//-----------------------------------------------------------------------------

motor::motor(address_type address, const motor_type &t)
{
  connect({{ "address", { address } }, { "driver_name", { t }}});
}

//-----------------------------------------------------------------------------

bool motor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
{
  static const std::string _strClassDir { SYS_ROOT "/tacho-motor/" };
  static const std::string _strPattern  { "motor" };

  try
  {
    return device::connect(_strClassDir, _strPattern, match);
  }
  catch (...) { }

  _path.clear();

  return false;
}

//-----------------------------------------------------------------------------

medium_motor::medium_motor(address_type address) : motor(address, motor_medium)
{
}

//-----------------------------------------------------------------------------

large_motor::large_motor(address_type address) : motor(address, motor_large)
{
}

//-----------------------------------------------------------------------------

dc_motor::dc_motor(address_type address)
{
  static const std::string _strClassDir { SYS_ROOT "/dc-motor/" };
  static const std::string _strPattern  { "motor" };

  connect(_strClassDir, _strPattern, {{ "address", { address }}});
}

//~autogen generic-define-property-value classes.dcMotor>currentClass

constexpr char dc_motor::command_run_forever[];
constexpr char dc_motor::command_run_timed[];
constexpr char dc_motor::command_run_direct[];
constexpr char dc_motor::command_stop[];
constexpr char dc_motor::polarity_normal[];
constexpr char dc_motor::polarity_inversed[];
constexpr char dc_motor::stop_action_coast[];
constexpr char dc_motor::stop_action_brake[];

//~autogen

//-----------------------------------------------------------------------------

servo_motor::servo_motor(address_type address)
{
  static const std::string _strClassDir { SYS_ROOT "/servo-motor/" };
  static const std::string _strPattern  { "motor" };

  connect(_strClassDir, _strPattern, {{ "address", { address }}});
}

//~autogen generic-define-property-value classes.servoMotor>currentClass

constexpr char servo_motor::command_run[];
constexpr char servo_motor::command_float[];
constexpr char servo_motor::polarity_normal[];
constexpr char servo_motor::polarity_inversed[];

//~autogen

//-----------------------------------------------------------------------------

led::led(std::string name)
{
  static const std::string _strClassDir { SYS_ROOT "/leds/" };
  connect(_strClassDir, name, std::map<std::string, std::set<std::string>>());
}

//-----------------------------------------------------------------------------

void led::flash(unsigned on_ms, unsigned off_ms)
{
  static const mode_type timer("timer");
  set_trigger(timer);
  if (on_ms)
  {
    // A workaround for ev3dev/ev3dev#225.
    // It takes some time for delay_{on,off} sysfs attributes to appear after
    // led trigger has been set to "timer".
    for (int i = 0; ; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      try {
        set_delay_on (on_ms );
        set_delay_off(off_ms);
        break;
      } catch(...) {
        if (i >= 5) throw;
      }
    }
  }
}

//-----------------------------------------------------------------------------

#if defined(EV3DEV_PLATFORM_BRICKPI)
//~autogen leds-define platforms.brickpi.led>currentClass

led led::blue_led1{"brickpi:led1:blue:ev3dev"};
led led::blue_led2{"brickpi:led2:blue:ev3dev"};

std::vector<led*> led::led1{ &led::blue_led1 };
std::vector<led*> led::led2{ &led::blue_led2 };

std::vector<float> led::black{ static_cast<float>(0) };
std::vector<float> led::blue{ static_cast<float>(1) };

//-----------------------------------------------------------------------------
void led::all_off() {

    blue_led1.off();
    blue_led2.off();

}

//~autogen
#elif defined(EV3DEV_PLATFORM_BRICKPI3)
led led::amber_led1{"brickpi3:amber:ev3dev"};

std::vector<led*> led::led1{ &led::amber_led1 };

std::vector<float> led::black{ static_cast<float>(0) };
std::vector<float> led::blue{ static_cast<float>(1) };

//-----------------------------------------------------------------------------
void led::all_off() {

    amber_led1.off();

}
#elif defined(EV3DEV_PLATFORM_PISTORMS)
//~autogen leds-define platforms.pistorms.led>currentClass

led led::red_left{"pistorms:BB:red:ev3dev"};
led led::red_right{"pistorms:BA:red:ev3dev"};
led led::green_left{"pistorms:BB:green:ev3dev"};
led led::green_right{"pistorms:BA:green:ev3dev"};
led led::blue_left{"pistorms:BB:blue:ev3dev"};
led led::blue_right{"pistorms:BA:blue:ev3dev"};

std::vector<led*> led::left{ &led::red_left, &led::green_left, &led::blue_left };
std::vector<led*> led::right{ &led::red_right, &led::green_right, &led::blue_right };

std::vector<float> led::black{ static_cast<float>(0), static_cast<float>(0), static_cast<float>(0) };
std::vector<float> led::red{ static_cast<float>(1), static_cast<float>(0), static_cast<float>(0) };
std::vector<float> led::green{ static_cast<float>(0), static_cast<float>(1), static_cast<float>(0) };
std::vector<float> led::blue{ static_cast<float>(0), static_cast<float>(0), static_cast<float>(1) };
std::vector<float> led::yellow{ static_cast<float>(1), static_cast<float>(1), static_cast<float>(0) };
std::vector<float> led::purple{ static_cast<float>(1), static_cast<float>(0), static_cast<float>(1) };
std::vector<float> led::cyan{ static_cast<float>(0), static_cast<float>(1), static_cast<float>(1) };
std::vector<float> led::white{ static_cast<float>(1), static_cast<float>(1), static_cast<float>(1) };
std::vector<float> led::orange{ static_cast<float>(1), static_cast<float>(0.5), static_cast<float>(0) };

//-----------------------------------------------------------------------------
void led::all_off() {

    red_left.off();
    red_right.off();
    green_left.off();
    green_right.off();
    blue_left.off();
    blue_right.off();

}

//~autogen
#else
//~autogen leds-define platforms.ev3.led>currentClass

led led::red_left{"led0:red:brick-status"};
led led::red_right{"led1:red:brick-status"};
led led::green_left{"led0:green:brick-status"};
led led::green_right{"led1:green:brick-status"};

std::vector<led*> led::left{ &led::red_left, &led::green_left };
std::vector<led*> led::right{ &led::red_right, &led::green_right };

std::vector<float> led::black{ static_cast<float>(0), static_cast<float>(0) };
std::vector<float> led::red{ static_cast<float>(1), static_cast<float>(0) };
std::vector<float> led::green{ static_cast<float>(0), static_cast<float>(1) };
std::vector<float> led::amber{ static_cast<float>(1), static_cast<float>(1) };
std::vector<float> led::orange{ static_cast<float>(1), static_cast<float>(0.5) };
std::vector<float> led::yellow{ static_cast<float>(0.1), static_cast<float>(1) };

//-----------------------------------------------------------------------------
void led::all_off() {

    red_left.off();
    red_right.off();
    green_left.off();
    green_right.off();

}

//~autogen
#endif

//-----------------------------------------------------------------------------

void led::set_color(const std::vector<led*> &group, const std::vector<float> &color) {
  const size_t n = std::min(group.size(), color.size());
  for(size_t i = 0; i < n; ++i)
    group[i]->set_brightness_pct(color[i]);
}

//-----------------------------------------------------------------------------

power_supply power_supply::battery { "" };

//-----------------------------------------------------------------------------

power_supply::power_supply(std::string name)
{
  static const std::string _strClassDir { SYS_ROOT "/power_supply/" };

  if (name.empty())
    name = "legoev3-battery";

  connect(_strClassDir, name, std::map<std::string, std::set<std::string>>());
}

//-----------------------------------------------------------------------------

button::file_descriptor::file_descriptor(const char *path, int flags)
  : _fd(open(path, flags))
{}

button::file_descriptor::~file_descriptor()
{
  if (_fd != -1) close(_fd);
}

//-----------------------------------------------------------------------------

button::button(int bit)
  : _bit(bit),
    _buf((KEY_CNT + bits_per_long - 1) / bits_per_long),
    _fd( new file_descriptor("/dev/input/by-path/platform-gpio_keys-event", O_RDONLY) )
{ }

//-----------------------------------------------------------------------------

bool button::pressed() const
{
#ifndef NO_LINUX_HEADERS
  if (ioctl(*_fd, EVIOCGKEY(_buf.size()), _buf.data()) < 0)
  {
    // handle error
  }
#endif
  return (_buf[_bit / bits_per_long] & 1 << (_bit % bits_per_long));
}

//-----------------------------------------------------------------------------

bool button::process()
{
  bool new_state = pressed();

  if (new_state != _state) {
    _state = new_state;
    if (onclick) onclick(new_state);
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------

#ifndef NO_LINUX_HEADERS
button button::back (KEY_BACKSPACE);
button button::left (KEY_LEFT);
button button::right(KEY_RIGHT);
button button::up   (KEY_UP);
button button::down (KEY_DOWN);
button button::enter(KEY_ENTER);
#endif

//-----------------------------------------------------------------------------

bool button::process_all() {
  std::array<bool, 6> changed{{
    back. process(),
    left. process(),
    right.process(),
    up.   process(),
    down. process(),
    enter.process()
  }};
  return std::any_of(changed.begin(), changed.end(), [](bool c){ return c; });
}

//-----------------------------------------------------------------------------

void sound::beep(const std::string &args, bool bSynchronous)
{
  std::ostringstream cmd;
  cmd << "/usr/bin/beep " << args;
  if (!bSynchronous) cmd << " &";
  std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

void sound::tone(
    const std::vector< std::vector<float> > &sequence,
    bool bSynchronous
    )
{
  std::ostringstream args;
  bool first = true;

  for(auto v : sequence) {
    if (first) {
      first = false;
    } else {
      args << " -n";
    }

    if (v.size() > 0) {
      args << " -f " << v[0];
    } else {
      continue;
    }

    if (v.size() > 1) {
      args << " -l " << v[1];
    } else {
      continue;
    }

    if (v.size() > 2) {
      args << " -D " << v[2];
    } else {
      continue;
    }
  }

  beep(args.str(), bSynchronous);
}

//-----------------------------------------------------------------------------

void sound::tone(float frequency, float ms, bool bSynchronous) {
  tone({{frequency, ms, 0.0f}}, bSynchronous);
}

//-----------------------------------------------------------------------------

void sound::play(const std::string &soundfile, bool bSynchronous)
{
  std::ostringstream cmd;
  cmd << "/usr/bin/aplay -q " << soundfile;

  if (!bSynchronous) cmd << " &";

  std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

void sound::speak(const std::string &text, bool bSynchronous)
{
  std::ostringstream cmd;

  cmd << "/usr/bin/espeak -a 200 --stdout \"" << text << "\""
      << " | /usr/bin/aplay -q";

  if (!bSynchronous) cmd << " &";

  std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

lcd::lcd() :
  _fb(nullptr),
  _fbsize(0),
  _llength(0),
  _xres(0),
  _yres(0),
  _bpp(0)
{
  init();
}

//-----------------------------------------------------------------------------

lcd::~lcd()
{
  deinit();
}

//-----------------------------------------------------------------------------

void lcd::fill(unsigned char pixel)
{
  if (_fb && _fbsize)
  {
    memset(_fb, pixel, _fbsize);
  }
}

//-----------------------------------------------------------------------------

void lcd::init()
{
  using namespace std;

 #ifdef _LINUX_FB_H
  int fbf = open("/dev/fb0", O_RDWR);
  if (fbf < 0)
    return;

  fb_fix_screeninfo i;
  if (ioctl(fbf, FBIOGET_FSCREENINFO, &i) < 0)
    return;

  _fbsize  = i.smem_len;
  _llength = i.line_length;

  _fb = (unsigned char*)mmap(NULL, _fbsize, PROT_READ|PROT_WRITE, MAP_SHARED, fbf, 0);
  if (_fb == nullptr)
    return;

	fb_var_screeninfo v;

  if (ioctl(fbf, FBIOGET_VSCREENINFO, &v) < 0)
    return;

  _xres = v.xres;
  _yres = v.yres;
  _bpp  = v.bits_per_pixel;
 #endif
}

//-----------------------------------------------------------------------------

void lcd::deinit()
{
  if (_fb)
  {
    munmap(_fb, 0);
  }

  _fbsize = 0;
}

//-----------------------------------------------------------------------------

remote_control::remote_control(unsigned channel) :
  _sensor(new infrared_sensor),
  _owns_sensor(true)
{
  if ((channel >= 1) && (channel <=4))
    _channel = channel-1;

  if (_sensor->connected())
    _sensor->set_mode(infrared_sensor::mode_ir_remote);
}

//-----------------------------------------------------------------------------

remote_control::remote_control(infrared_sensor &ir, unsigned channel) :
  _sensor(&ir),
  _owns_sensor(false)
{
  if ((channel >= 1) && (channel <=4))
    _channel = channel-1;

  if (_sensor->connected())
    _sensor->set_mode(infrared_sensor::mode_ir_remote);
}

//-----------------------------------------------------------------------------

remote_control::~remote_control()
{
  if (_owns_sensor)
    delete _sensor;
}

//-----------------------------------------------------------------------------

bool remote_control::process()
{
  int value = _sensor->value(_channel);
  if (value != _value)
  {
    on_value_changed(value);
    _value = value;
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------

void remote_control::on_value_changed(int value)
{
  int new_state = 0;

  switch (value)
  {
  case 1:
    new_state = red_up;
    break;
  case 2:
    new_state = red_down;
    break;
  case 3:
    new_state = blue_up;
    break;
  case  4:
    new_state = blue_down;
    break;
  case 5:
    new_state = red_up | blue_up;
    break;
  case 6:
    new_state = red_up | blue_down;
    break;
  case 7:
    new_state = red_down |  blue_up;
    break;
  case 8:
    new_state = red_down | blue_down;
    break;
  case 9:
    new_state = beacon;
    break;
  case 10:
    new_state = red_up | red_down;
    break;
  case 11:
    new_state = blue_up | blue_down;
    break;
  }

  if (((new_state & red_up) != (_state & red_up)) &&
      static_cast<bool>(on_red_up))
    on_red_up(new_state & red_up);

  if (((new_state & red_down) != (_state & red_down)) &&
      static_cast<bool>(on_red_down))
    on_red_down(new_state & red_down);

  if (((new_state & blue_up) != (_state & blue_up)) &&
      static_cast<bool>(on_blue_up))
    on_blue_up(new_state & blue_up);

  if (((new_state & blue_down) != (_state & blue_down)) &&
      static_cast<bool>(on_blue_down))
    on_blue_down(new_state & blue_down);

  if (((new_state & beacon) != (_state & beacon)) &&
      static_cast<bool>(on_beacon))
    on_beacon(new_state & beacon);

  if ((new_state != _state) &&
          static_cast<bool>(on_state_change))
      on_state_change(new_state);

  _state = new_state;
}

//-----------------------------------------------------------------------------

lego_port::lego_port(address_type address)
{
  connect({{ "address", { address } }});
}

//-----------------------------------------------------------------------------

bool lego_port::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
{
  static const std::string _strClassDir { SYS_ROOT "/lego-port/" };
  static const std::string _strPattern  { "port" };

  try
  {
    return device::connect(_strClassDir, _strPattern, match);
  }
  catch (...) { }

  _path.clear();

  return false;
}

//-----------------------------------------------------------------------------

} // namespace ev3dev

// vim: sw=2
