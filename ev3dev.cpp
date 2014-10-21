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

#include "ev3dev.h"

#include <iostream>
#include <fstream>
#include <map>
#include <system_error>
#include <dirent.h>
#include <string.h>

#include <cstdio>
#include <fstream>
#include <iostream>

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>

#ifndef SYS_ROOT
#define SYS_ROOT "/sys"
#endif

#ifndef NO_LINUX_HEADERS
#include <linux/fb.h>
#include <linux/input.h>
#else
#define KEY_CNT 8
#endif

#define SYS_BUTTON SYS_ROOT "/devices/platform/ev3dev/button"
#define SYS_SOUND  SYS_ROOT "/devices/platform/snd-legoev3/"
#define SYS_POWER  SYS_ROOT "/class/power_supply/legoev3-battery/"

//-----------------------------------------------------------------------------

namespace ev3dev {

//-----------------------------------------------------------------------------

int device::get_attr_int(const std::string &name) const
{
  using namespace std;
  
  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");
  
  ifstream is((_path+name).c_str());
  if (is.is_open())
  {
    int result = 0;
    is >> result;
    return result;
  }
  
  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

void device::set_attr_int(const std::string &name, int value)
{
  using namespace std;
  
  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ofstream os((_path+name).c_str());
  if (os.is_open())
  {
    os << value;
    return;
  }
  
  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

std::string device::get_attr_string(const std::string &name) const
{
  using namespace std;
  
  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ifstream is((_path+name).c_str());
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

  ofstream os((_path+name).c_str());
  if (os.is_open())
  {
    os << value;
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
  
  ifstream is((_path+name).c_str());
  if (is.is_open())
  {
    string result;
    getline(is, result);
    return result;
  }
  
  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

const sensor::sensor_type sensor::ev3_touch       { "lego-ev3-touch" };
const sensor::sensor_type sensor::ev3_color       { "ev3-uart-29" };
const sensor::sensor_type sensor::ev3_ultrasonic  { "ev3-uart-30" };
const sensor::sensor_type sensor::ev3_gyro        { "ev3-uart-32" };
const sensor::sensor_type sensor::ev3_infrared    { "ev3-uart-33" };
  
const sensor::sensor_type sensor::nxt_touch       { "lego-nxt-touch" };
const sensor::sensor_type sensor::nxt_light       { "lego-nxt-light" };
const sensor::sensor_type sensor::nxt_sound       { "lego-nxt-sound" };
const sensor::sensor_type sensor::nxt_ultrasonic  { "lego-nxt-ultrasonic" };
const sensor::sensor_type sensor::nxt_temperature { "tmp275" };
  
//-----------------------------------------------------------------------------

sensor::sensor(port_type port_)
{
  init(port_, std::set<sensor_type>());
}

//-----------------------------------------------------------------------------

sensor::sensor(port_type port_, const std::set<sensor_type> &types_)
{
  init(port_, types_);
}

//-----------------------------------------------------------------------------

bool sensor::init(port_type port_, const std::set<sensor_type> &types_) noexcept
{
  using namespace std;
  
  string strClassDir(SYS_ROOT "/class/msensor/");
  
  struct dirent *dp;
  DIR *dfd;
  
  if ((dfd = opendir(strClassDir.c_str())) != NULL)
  {
    while ((dp = readdir(dfd)) != NULL)
    {
      if (strncmp(dp->d_name, "sensor", 6)==0)
      {
        try
        {
          _path = strClassDir + dp->d_name + '/';
          
          _port_name = get_attr_string("port_name");
          if (port_.empty() || (_port_name == port_))
          {
            _type = get_attr_string("name");
            if (types_.empty() || (types_.find(_type) != types_.cend()))
            {
              _device_index = 0;
              for (unsigned i=6; dp->d_name[i]!=0; ++i)
              {
                _device_index *= 10;
                _device_index += dp->d_name[i]-'0';
              }
              
              read_mode_values();
            
              return true;
            }
          }
        }
        catch (...) { }
        
        _path.clear();
        _port_name.clear();
        _type.clear();
      }
    }
    
    closedir(dfd);
  }
  else
    cerr << "no msensor dir!" << endl;
  
  return false;
}

//-----------------------------------------------------------------------------

const std::string &sensor::type_name() const
{
  if (_type.empty())
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
    { nxt_temperature, "NXT temperature" }
  };
  
  auto s = lookup_table.find(_type);
  if (s != lookup_table.end())
    return s->second;
  
  return _type;
}

//-----------------------------------------------------------------------------

int sensor::value(unsigned index) const
{
  if (index >= _nvalues)
    throw std::invalid_argument("index");
    
  char svalue[8] = "value0";
  svalue[7] += index;
  
  return get_attr_int(svalue);
}

//-----------------------------------------------------------------------------
  
float sensor::float_value(unsigned index) const
{
  return value(index) * _dp_scale;
}

//-----------------------------------------------------------------------------
  
const mode_set &sensor::modes() const
{
  return _modes;
}

//-----------------------------------------------------------------------------
  
const mode_type &sensor::mode() const
{
  return _mode;
}

//-----------------------------------------------------------------------------

void sensor::read_mode_values()
{
  using namespace std;

  _mode = get_attr_string("mode");

  _modes.clear();

  string s = get_attr_line("modes");
  size_t pos, last_pos = 0;
  string m;
  do {
    pos = s.find(' ', last_pos);
    
    if (pos != string::npos)
    {
      m = s.substr(last_pos, pos-last_pos);
      last_pos = pos+1;
    }
    else
      m = s.substr(last_pos);
    
    if (!m.empty())
    {
      _modes.insert(m);
    }
  } while (pos!=string::npos);
  
  _nvalues = get_attr_int("num_values");
  _dp_scale = 1.f;
  for (unsigned dp = get_attr_int("dp"); dp; --dp)
  {
    _dp_scale /= 10.f;
  }
}

//-----------------------------------------------------------------------------

void sensor::set_mode(const mode_type &mode_)
{
  if (mode_ != _mode)
  {
    set_attr_string("mode", mode_);
    const_cast<sensor*>(this)->read_mode_values();
  }
}

//-----------------------------------------------------------------------------

touch_sensor::touch_sensor(port_type port_) :
  sensor(port_, { ev3_touch })
{
}

//-----------------------------------------------------------------------------

const mode_type color_sensor::mode_reflect { "COL-REFLECT" };
const mode_type color_sensor::mode_ambient { "COL-AMBIENT" };
const mode_type color_sensor::mode_color   { "COL-COLOR"   };

color_sensor::color_sensor(port_type port_) :
  sensor(port_, { ev3_color })
{
}

//-----------------------------------------------------------------------------

const mode_type ultrasonic_sensor::mode_dist_cm   { "US-DIST-CM" };
const mode_type ultrasonic_sensor::mode_dist_in   { "US-DIST-IN" };
const mode_type ultrasonic_sensor::mode_listen    { "US-LISTEN"  };
const mode_type ultrasonic_sensor::mode_single_cm { "US-SI-CM"   };
const mode_type ultrasonic_sensor::mode_single_in { "US-SI-IN"   };

ultrasonic_sensor::ultrasonic_sensor(port_type port_) :
  sensor(port_, { ev3_ultrasonic })
{
}

//-----------------------------------------------------------------------------

const mode_type gyro_sensor::mode_angle           { "GYRO-ANG"  };
const mode_type gyro_sensor::mode_speed           { "GYRO-RATE" };
const mode_type gyro_sensor::mode_angle_and_speed { "GYRO-G&A"  };

gyro_sensor::gyro_sensor(port_type port_) :
  sensor(port_, { ev3_gyro })
{
}

//-----------------------------------------------------------------------------

const mode_type infrared_sensor::mode_proximity { "IR-PROX"   };
const mode_type infrared_sensor::mode_ir_seeker { "IR-SEEK"   };
const mode_type infrared_sensor::mode_ir_remote { "IR-REMOTE" };

infrared_sensor::infrared_sensor(port_type port_) :
  sensor(port_, { ev3_infrared })
{
}

//-----------------------------------------------------------------------------
  
const motor::motor_type motor::motor_large  { "tacho"     };
const motor::motor_type motor::motor_medium { "minitacho" };

const mode_type motor::mode_off { "off" };
const mode_type motor::mode_on  { "on"  };

const mode_type motor::run_mode_forever  { "forever"  };
const mode_type motor::run_mode_time     { "time"     };
const mode_type motor::run_mode_position { "position" };
  
const mode_type motor::stop_mode_coast { "coast" };
const mode_type motor::stop_mode_brake { "brake" };
const mode_type motor::stop_mode_hold  { "hold"  };

const mode_type motor::position_mode_absolute { "absolute" };
const mode_type motor::position_mode_relative { "relative" };

//-----------------------------------------------------------------------------

motor::motor(port_type p)
{
  init(p, std::string());
}

//-----------------------------------------------------------------------------

motor::motor(port_type p, const motor_type &t)
{
  init(p, t);
}

//-----------------------------------------------------------------------------

bool motor::init(port_type port_, const motor_type &type_) noexcept
{
  using namespace std;
  
  string strClassDir(SYS_ROOT "/class/tacho-motor/");
  
  struct dirent *dp;
  DIR *dfd;
  
  if ((dfd = opendir(strClassDir.c_str())) != NULL)
  {
    while ((dp = readdir(dfd)) != NULL)
    {
      if (strncmp(dp->d_name, "tacho-motor", 11)==0)
      {
        try
        {
          _path = strClassDir + dp->d_name + '/';
          
          _port_name = get_attr_string("port_name");
          if (port_.empty() || (_port_name == port_))
          {
            _type = get_attr_string("type");
            if (type_.empty() || (_type == type_))
            {
              _device_index = 0;
              for (unsigned i=11; dp->d_name[i]!=0; ++i)
              {
                _device_index *= 10;
                _device_index += dp->d_name[i]-'0';
              }
              
              return true;
            }
          }
        }
        catch (...) { }
        
        _path.clear();
        _port_name.clear();
        _type.clear();
      }
    }
    closedir(dfd);
  }
  
  return false;
}

//-----------------------------------------------------------------------------

motor::motor_type motor::type() const
{
  return get_attr_string("type");
}
  
//-----------------------------------------------------------------------------
  
void motor::run(bool bRun)
{
  set_attr_int("run", bRun);
}
  
//-----------------------------------------------------------------------------
  
void motor::stop()
{
  set_attr_int("run", 0);
}
  
//-----------------------------------------------------------------------------
  
void motor::reset()
{
  set_attr_int("reset", 1);
}
  
//-----------------------------------------------------------------------------
  
bool motor::running() const
{
  return get_attr_int("run")!=0;
}

//-----------------------------------------------------------------------------

mode_type motor::state() const
{
  return get_attr_string("state");
}

//-----------------------------------------------------------------------------

int motor::duty_cycle() const
{
  return get_attr_int("duty_cycle");
}

//-----------------------------------------------------------------------------

int motor::pulses_per_second() const
{
  return get_attr_int("pulses_per_second");
}

//-----------------------------------------------------------------------------

int motor::position() const
{
  return get_attr_int("position");
}

//-----------------------------------------------------------------------------

void motor::set_position(int pos)
{
  set_attr_int("position", pos);
}

//-----------------------------------------------------------------------------

mode_type motor::run_mode() const
{
  return get_attr_string("run_mode");
}

//-----------------------------------------------------------------------------

void motor::set_run_mode(const mode_type &value)
{
  set_attr_string("run_mode", value);
}

//-----------------------------------------------------------------------------

mode_type motor::stop_mode() const
{
  return get_attr_string("stop_mode");
}

//-----------------------------------------------------------------------------

void motor::set_stop_mode(const mode_type &value)
{
  set_attr_string("stop_mode", value);
}

//-----------------------------------------------------------------------------

mode_type motor::regulation_mode() const
{
  return get_attr_string("regulation_mode");
}

//-----------------------------------------------------------------------------

void motor::set_regulation_mode(const mode_type &value)
{
  set_attr_string("regulation_mode", value);
}

//-----------------------------------------------------------------------------

mode_type motor::position_mode() const
{
  return get_attr_string("position_mode");
}

//-----------------------------------------------------------------------------

void motor::set_position_mode(const mode_type &value)
{
  set_attr_string("position_mode", value);
}

//-----------------------------------------------------------------------------

int motor::duty_cycle_setpoint() const
{
  return get_attr_int("duty_cycle_sp");
}

//-----------------------------------------------------------------------------

void motor::set_duty_cycle_setpoint(int value)
{
  set_attr_int("duty_cycle_sp", value);
}

//-----------------------------------------------------------------------------

int motor::pulses_per_second_setpoint() const
{
  return get_attr_int("pulses_per_second_sp");
}

//-----------------------------------------------------------------------------

void motor::set_pulses_per_second_setpoint(int value)
{
  set_attr_int("pulses_per_second_sp", value);
}

//-----------------------------------------------------------------------------

int  motor::time_setpoint() const
{
  return get_attr_int("time_sp");
}

//-----------------------------------------------------------------------------

void motor::set_time_setpoint(int value)
{
  set_attr_int("time_sp", value);
}

//-----------------------------------------------------------------------------

int  motor::position_setpoint() const
{
  return get_attr_int("position_sp");
}
  
//-----------------------------------------------------------------------------

void motor::set_position_setpoint(int value)
{
  set_attr_int("position_sp", value);
}

//-----------------------------------------------------------------------------

int  motor::ramp_up() const
{
  return get_attr_int("ramp_up_sp");
}

//-----------------------------------------------------------------------------

void motor::set_ramp_up(int value)
{
  set_attr_int("ramp_up_sp", value);
}

//-----------------------------------------------------------------------------

int motor::ramp_down() const
{
  return get_attr_int("ramp_down_sp");
}

//-----------------------------------------------------------------------------

void motor::set_ramp_down(int value)
{
  set_attr_int("ramp_down_sp", value);
}

//-----------------------------------------------------------------------------

int motor::speed_regulation_p() const
{
  return get_attr_int("speed_regulation_p");
}

//-----------------------------------------------------------------------------

void motor::set_speed_regulation_p(int value)
{
  set_attr_int("speed_regulation_p", value);
}

//-----------------------------------------------------------------------------

int motor::speed_regulation_i() const
{
  return get_attr_int("speed_regulation_i");
}

//-----------------------------------------------------------------------------

void motor::set_speed_regulation_i(int value)
{
  set_attr_int("speed_regulation_i", value);
}

//-----------------------------------------------------------------------------

int motor::speed_regulation_d() const
{
  return get_attr_int("speed_regulation_d");
}

//-----------------------------------------------------------------------------

void motor::set_speed_regulation_d(int value)
{
  set_attr_int("speed_regulation_d", value);
}

//-----------------------------------------------------------------------------

int motor::speed_regulation_k() const
{
  return get_attr_int("speed_regulation_k");
}

//-----------------------------------------------------------------------------

void motor::set_speed_regulation_k(int value)
{
  set_attr_int("speed_regulation_k", value);
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

led::led(const std::string &name)
{
  std::string p(SYS_ROOT "/class/leds/ev3:" + name);
  
  DIR *dfd;
  if ((dfd = opendir(p.c_str())) != NULL)
  {
    _path = p + '/';
    closedir(dfd);
  }
}

//-----------------------------------------------------------------------------

int led::level() const
{
  return get_attr_int("brightness");
}

//-----------------------------------------------------------------------------

void led::on()
{
  set_attr_int("brightness", 1);
}

//-----------------------------------------------------------------------------

void led::off()
{
  set_attr_int("brightness", 0);
}

//-----------------------------------------------------------------------------

void led::flash(unsigned interval_ms)
{
  static const mode_type timer("timer");
  set_trigger(timer);
  if (interval_ms)
  {
    set_on_delay (interval_ms);
    set_off_delay(interval_ms);
  }
}

//-----------------------------------------------------------------------------

void led::set_on_delay(unsigned ms)
{
  set_attr_int("delay_on", ms);
}

//-----------------------------------------------------------------------------

void led::set_off_delay(unsigned ms)
{
  set_attr_int("delay_off", ms);
}

//-----------------------------------------------------------------------------

mode_type led::trigger() const
{
  using namespace std;
  
  ifstream is((_path+"/trigger").c_str());
  if (is.is_open())
  {
    string s;
    getline(is, s);
    
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
  }
  
  return mode_type("none");
}

//-----------------------------------------------------------------------------

mode_set led::triggers() const
{
  using namespace std;
  
  mode_set result;
  
  ifstream is((_path+"/trigger").c_str());
  if (is.is_open())
  {
    string s;
    getline(is, s);
    
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
        }
        result.insert(t);
      }
    } while (pos!=string::npos);
  }
  
  return result;
}

//-----------------------------------------------------------------------------

void led::set_trigger(const mode_type &trigger_)
{
  set_attr_string("trigger", trigger_);
}

//-----------------------------------------------------------------------------

led led::red_right   { "red:right"   };
led led::red_left    { "red:left"    };
led led::green_right { "green:right" };
led led::green_left  { "green:left"  };

//-----------------------------------------------------------------------------

void led::red_on   () { red_right  .on();  red_left  .on();  }
void led::red_off  () { red_right  .off(); red_left  .off(); }
void led::green_on () { green_right.on();  green_left.on();  }
void led::green_off() { green_right.off(); green_left.off(); }
void led::all_on   () { red_on();  green_on();  }
void led::all_off  () { red_off(); green_off(); }

//-----------------------------------------------------------------------------

button::button(int bit)
{
	_bits_per_long = sizeof(long) * 8;
	_buf_size=(KEY_CNT + _bits_per_long - 1) / _bits_per_long;
	_buf = new unsigned long [_buf_size];
	_bit = bit;
	_fd = open("/dev/input/by-path/platform-gpio-keys.0-event", O_RDONLY);
}

//-----------------------------------------------------------------------------

bool button::pressed() const
{
 #ifndef NO_LINUX_HEADERS
	if (ioctl(_fd, EVIOCGKEY(_buf_size), _buf) < 0)
	{
		// handle error
	}
 #endif
	// bit in bytes is 1 when released and 0 when pressed
	return !(_buf[_bit / _bits_per_long] & 1 << (_bit % _bits_per_long));
}

//-----------------------------------------------------------------------------
#ifndef NO_LINUX_HEADERS
button button::back (KEY_ESC);
button button::left (KEY_LEFT);
button button::right(KEY_RIGHT);
button button::up   (KEY_UP);
button button::down (KEY_DOWN);
button button::enter(KEY_ENTER);
#endif
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
  std::string cmd("aplay ");
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
  cmd.append("\" | aplay");
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

float battery::voltage()
{
  unsigned result = 0;
  
  std::ifstream is(SYS_POWER "/voltage_now");
  if (is.is_open())
  {
    is >> result;
  }
  
  return (result / 1000000.f);
}

//-----------------------------------------------------------------------------

float battery::current()
{
  unsigned result = 0;
  
  std::ifstream is(SYS_POWER "/current_now");
  if (is.is_open())
  {
    is >> result;
  }
  
  return (result / 1000.f);
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
  
  _state = new_state;
}

//-----------------------------------------------------------------------------
  
} // namespace ev3dev

//-----------------------------------------------------------------------------
