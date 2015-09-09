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
    // Sections of the following code were auto-generated based on spec v0.9.3-pre, rev 2. 
//~autogen
//-----------------------------------------------------------------------------

#include "ev3dev.h"

#include <iostream>
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
#define SYS_ROOT "/sys"
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

#define SYS_SOUND  SYS_ROOT "/devices/platform/snd-legoev3/"

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
lru_cache<std::string, std::ifstream> ifstream_cache(FSTREAM_CACHE_SIZE);
lru_cache<std::string, std::ofstream> ofstream_cache(FSTREAM_CACHE_SIZE);
std::mutex ofstream_cache_lock;
std::mutex ifstream_cache_lock;

//-----------------------------------------------------------------------------

std::ofstream &ofstream_open(const std::string &path)
{
  std::lock_guard<std::mutex> lock(ofstream_cache_lock);
  std::ofstream &file = ofstream_cache[path];
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
  std::lock_guard<std::mutex> lock(ifstream_cache_lock);
  std::ifstream &file = ifstream_cache[path];
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

bool device::connect(const std::string &dir,
                     const std::string &pattern,
                     const std::map<std::string,
                                    std::set<std::string>> &match) noexcept
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
    } else {
      throw system_error(make_error_code(errc::no_such_device), _path+name);
    }
  }
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

const sensor::sensor_type sensor::ev3_touch       { "lego-ev3-touch" };
const sensor::sensor_type sensor::ev3_color       { "lego-ev3-color" };
const sensor::sensor_type sensor::ev3_ultrasonic  { "lego-ev3-us" };
const sensor::sensor_type sensor::ev3_gyro        { "lego-ev3-gyro" };
const sensor::sensor_type sensor::ev3_infrared    { "lego-ev3-ir" };

const sensor::sensor_type sensor::nxt_touch       { "lego-nxt-touch" };
const sensor::sensor_type sensor::nxt_light       { "lego-nxt-light" };
const sensor::sensor_type sensor::nxt_sound       { "lego-nxt-sound" };
const sensor::sensor_type sensor::nxt_ultrasonic  { "lego-nxt-us" };
const sensor::sensor_type sensor::nxt_i2c_sensor  { "nxt-i2c-sensor" };
const sensor::sensor_type sensor::nxt_analog      { "nxt-analog" };

//-----------------------------------------------------------------------------

sensor::sensor(port_type port)
{
  connect({{ "port_name", { port }}});
}

//-----------------------------------------------------------------------------

sensor::sensor(port_type port, const std::set<sensor_type> &types)
{
  connect({{ "port_name", { port }},
           { "driver_name", types }});
}

//-----------------------------------------------------------------------------

bool sensor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
{
  static const std::string _strClassDir { SYS_ROOT "/class/lego-sensor/" };
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

i2c_sensor::i2c_sensor(port_type port_) :
  sensor(port_, { nxt_i2c_sensor })
{
}

//-----------------------------------------------------------------------------

i2c_sensor::i2c_sensor(port_type port_, address_type address_)
{
  connect({{ "port_name", { port_ }},
           { "driver_name",      { nxt_i2c_sensor }},
           { "address",   { address_ }}});
}

//-----------------------------------------------------------------------------

touch_sensor::touch_sensor(port_type port_) :
  sensor(port_, { ev3_touch, nxt_touch })
{
}

//-----------------------------------------------------------------------------

//~autogen cpp_generic-define-property-value classes.colorSensor>currentClass

const std::string color_sensor::mode_col_reflect{ "COL-REFLECT" };
const std::string color_sensor::mode_col_ambient{ "COL-AMBIENT" };
const std::string color_sensor::mode_col_color{ "COL-COLOR" };
const std::string color_sensor::mode_ref_raw{ "REF-RAW" };
const std::string color_sensor::mode_rgb_raw{ "RGB-RAW" };

//~autogen

color_sensor::color_sensor(port_type port_) :
  sensor(port_, { ev3_color })
{
}

//-----------------------------------------------------------------------------

//~autogen cpp_generic-define-property-value classes.ultrasonicSensor>currentClass

const std::string ultrasonic_sensor::mode_us_dist_cm{ "US-DIST-CM" };
const std::string ultrasonic_sensor::mode_us_dist_in{ "US-DIST-IN" };
const std::string ultrasonic_sensor::mode_us_listen{ "US-LISTEN" };
const std::string ultrasonic_sensor::mode_us_si_cm{ "US-SI-CM" };
const std::string ultrasonic_sensor::mode_us_si_in{ "US-SI-IN" };

//~autogen

ultrasonic_sensor::ultrasonic_sensor(port_type port_) :
  sensor(port_, { ev3_ultrasonic, nxt_ultrasonic })
{
}

//-----------------------------------------------------------------------------

//~autogen cpp_generic-define-property-value classes.gyroSensor>currentClass

const std::string gyro_sensor::mode_gyro_ang{ "GYRO-ANG" };
const std::string gyro_sensor::mode_gyro_rate{ "GYRO-RATE" };
const std::string gyro_sensor::mode_gyro_fas{ "GYRO-FAS" };
const std::string gyro_sensor::mode_gyro_g_a{ "GYRO-G&A" };
const std::string gyro_sensor::mode_gyro_cal{ "GYRO-CAL" };

//~autogen

gyro_sensor::gyro_sensor(port_type port_) :
  sensor(port_, { ev3_gyro })
{
}

//-----------------------------------------------------------------------------

//~autogen cpp_generic-define-property-value classes.infraredSensor>currentClass

const std::string infrared_sensor::mode_ir_prox{ "IR-PROX" };
const std::string infrared_sensor::mode_ir_seek{ "IR-SEEK" };
const std::string infrared_sensor::mode_ir_remote{ "IR-REMOTE" };
const std::string infrared_sensor::mode_ir_rem_a{ "IR-REM-A" };
const std::string infrared_sensor::mode_ir_cal{ "IR-CAL" };

//~autogen

infrared_sensor::infrared_sensor(port_type port_) :
  sensor(port_, { ev3_infrared })
{
}

//-----------------------------------------------------------------------------

//~autogen cpp_generic-define-property-value classes.soundSensor>currentClass

const std::string sound_sensor::mode_db{ "DB" };
const std::string sound_sensor::mode_dba{ "DBA" };

//~autogen

sound_sensor::sound_sensor(port_type port_) :
  sensor(port_, { nxt_sound, nxt_analog })
{
    if (connected() && driver_name() == nxt_analog) {
        device port;
        port.connect(SYS_ROOT "/class/lego-port/", "port", {{"port_name", {port_name()}}});

        if (port.connected()) {
            port.set_attr_string("set_device", nxt_sound);

            if (port.get_attr_string("status") != nxt_sound) {
                // Failed to load lego-nxt-sound friver. Wrong port?
                _path.clear();
            }
        } else {
            _path.clear();
        }
    }
}

//-----------------------------------------------------------------------------

//~autogen cpp_generic-define-property-value classes.lightSensor>currentClass

const std::string light_sensor::mode_reflect{ "REFLECT" };
const std::string light_sensor::mode_ambient{ "AMBIENT" };

//~autogen

light_sensor::light_sensor(port_type port_) :
  sensor(port_, { nxt_light })
{
}

//-----------------------------------------------------------------------------

const motor::motor_type motor::motor_large  { "lego-ev3-l-motor" };
const motor::motor_type motor::motor_medium { "lego-ev3-m-motor" };

//~autogen cpp_generic-define-property-value classes.motor>currentClass

const std::string motor::command_run_forever{ "run-forever" };
const std::string motor::command_run_to_abs_pos{ "run-to-abs-pos" };
const std::string motor::command_run_to_rel_pos{ "run-to-rel-pos" };
const std::string motor::command_run_timed{ "run-timed" };
const std::string motor::command_run_direct{ "run-direct" };
const std::string motor::command_stop{ "stop" };
const std::string motor::command_reset{ "reset" };
const std::string motor::encoder_polarity_normal{ "normal" };
const std::string motor::encoder_polarity_inversed{ "inversed" };
const std::string motor::polarity_normal{ "normal" };
const std::string motor::polarity_inversed{ "inversed" };
const std::string motor::speed_regulation_on{ "on" };
const std::string motor::speed_regulation_off{ "off" };
const std::string motor::stop_command_coast{ "coast" };
const std::string motor::stop_command_brake{ "brake" };
const std::string motor::stop_command_hold{ "hold" };

//~autogen

//-----------------------------------------------------------------------------

motor::motor(port_type port)
{
  connect({{ "port_name", { port } }});
}

//-----------------------------------------------------------------------------

motor::motor(port_type port, const motor_type &t)
{
  connect({{ "port_name", { port } }, { "driver_name", { t }}});
}

//-----------------------------------------------------------------------------

bool motor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
{
  static const std::string _strClassDir { SYS_ROOT "/class/tacho-motor/" };
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

medium_motor::medium_motor(port_type port_) : motor(port_, motor_medium)
{
}

//-----------------------------------------------------------------------------

large_motor::large_motor(port_type port_) : motor(port_, motor_large)
{
}

//-----------------------------------------------------------------------------

dc_motor::dc_motor(port_type port)
{
  static const std::string _strClassDir { SYS_ROOT "/class/dc-motor/" };
  static const std::string _strPattern  { "motor" };

  connect(_strClassDir, _strPattern, {{ "port_name", { port }}});
}

//~autogen cpp_generic-define-property-value classes.dcMotor>currentClass

const std::string dc_motor::command_run_forever{ "run-forever" };
const std::string dc_motor::command_run_timed{ "run-timed" };
const std::string dc_motor::command_run_direct{ "run-direct" };
const std::string dc_motor::command_stop{ "stop" };
const std::string dc_motor::polarity_normal{ "normal" };
const std::string dc_motor::polarity_inversed{ "inversed" };
const std::string dc_motor::stop_command_coast{ "coast" };
const std::string dc_motor::stop_command_brake{ "brake" };

//~autogen

//-----------------------------------------------------------------------------

servo_motor::servo_motor(port_type port)
{
  static const std::string _strClassDir { SYS_ROOT "/class/servo-motor/" };
  static const std::string _strPattern  { "motor" };

  connect(_strClassDir, _strPattern, {{ "port_name", { port }}});
}

//~autogen cpp_generic-define-property-value classes.servoMotor>currentClass

const std::string servo_motor::command_run{ "run" };
const std::string servo_motor::command_float{ "float" };
const std::string servo_motor::polarity_normal{ "normal" };
const std::string servo_motor::polarity_inversed{ "inversed" };

//~autogen

//-----------------------------------------------------------------------------

led::led(std::string name)
{
  static const std::string _strClassDir { SYS_ROOT "/class/leds/" };
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

led led::red_right   { "ev3-right0:red:ev3dev"   };
led led::red_left    { "ev3-left0:red:ev3dev"    };
led led::green_right { "ev3-right1:green:ev3dev" };
led led::green_left  { "ev3-left1:green:ev3dev"  };

//-----------------------------------------------------------------------------

void led::mix_colors(float red, float green) {
  red_right.set_brightness_pct(red);
  red_left.set_brightness_pct(red);

  green_right.set_brightness_pct(green);
  green_left.set_brightness_pct(green);
}

//-----------------------------------------------------------------------------

void led::all_off  () {
  red_right.set_brightness(0);
  red_left.set_brightness(0);
  green_right.set_brightness(0);
  green_left.set_brightness(0);
}

//-----------------------------------------------------------------------------

power_supply power_supply::battery { "" };

//-----------------------------------------------------------------------------

power_supply::power_supply(std::string name)
{
  static const std::string _strClassDir { SYS_ROOT "/class/power_supply/" };

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
    _fd( new file_descriptor("/dev/input/by-path/platform-gpio-keys.0-event", O_RDONLY) )
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
  // bit in bytes is 1 when released and 0 when pressed
  return !(_buf[_bit / bits_per_long] & 1 << (_bit % bits_per_long));
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
  std::array<bool, 6> changed = {
    back. process(),
    left. process(),
    right.process(),
    up.   process(),
    down. process(),
    enter.process()
  };
  return std::any_of(changed.begin(), changed.end(), [](bool c){ return c; });
}

//-----------------------------------------------------------------------------

void sound::beep()
{
  tone(1000, 100);
}

//-----------------------------------------------------------------------------

void sound::tone(unsigned frequency, unsigned ms)
{
  std::ofstream os(SYS_SOUND "/tone");
  if (os.is_open())
  {
    os << frequency;
    if (ms)
      os << " " << ms;
  }
}

//-----------------------------------------------------------------------------

void sound::play(const std::string &soundfile, bool bSynchronous)
{
  std::string cmd("aplay -q ");
  cmd.append(soundfile);
  if (!bSynchronous)
  {
    cmd.append(" &");
  }

  std::system(cmd.c_str());
}

//-----------------------------------------------------------------------------

void sound::speak(const std::string &text, bool bSynchronous)
{
  std::string cmd("espeak -a 200 --stdout \"");
  cmd.append(text);
  cmd.append("\" | aplay -q");
  if (!bSynchronous)
  {
    cmd.append(" &");
  }

  std::system(cmd.c_str());
}

//-----------------------------------------------------------------------------

unsigned sound::volume()
{
  unsigned result = 0;

  std::ifstream is(SYS_SOUND "/volume");
  if (is.is_open())
  {
    is >> result;
  }

  return result;
}

//-----------------------------------------------------------------------------

void sound::set_volume(unsigned v)
{
  std::ofstream os(SYS_SOUND "/volume");
  if (os.is_open())
  {
    os << v;
  }
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

lego_port::lego_port(port_type port)
{
  connect({{ "port_name", { port } }});
}

//-----------------------------------------------------------------------------

bool lego_port::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
{
  static const std::string _strClassDir { SYS_ROOT "/class/lego-port/" };
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
