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

#include "ev3dev.h"

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#ifndef SYS_ROOT
#define SYS_ROOT "/sys"
#endif

#ifndef NO_LINUX_HEADERS
#include <linux/fb.h>
#endif

#define SYS_BUTTON SYS_ROOT "/devices/platform/ev3dev/button"
#define SYS_SOUND  SYS_ROOT "/devices/platform/snd-legoev3/"
#define SYS_POWER  SYS_ROOT "/class/power_supply/legoev3-battery/"

//-----------------------------------------------------------------------------

namespace ev3dev {

//-----------------------------------------------------------------------------

int device::get_attr_int(const std::string &name) const
{
  int result = 0;
  
  std::ifstream is((_path+name).c_str());
  if (is.is_open())
  {
    is >> result;
  }
  
  return result;
}

//-----------------------------------------------------------------------------

void device::set_attr_int(const std::string &name, int value)
{
  std::ofstream os((_path+name).c_str());
  if (os.is_open())
  {
    os << value;
  }
}

//-----------------------------------------------------------------------------

std::string device::get_attr_string(const std::string &name) const
{
  std::string result;
  
  std::ifstream is((_path+name).c_str());
  if (is.is_open())
  {
    is >> result;
  }
  
  return result;
}

//-----------------------------------------------------------------------------

void device::set_attr_string(const std::string &name, const std::string &value)
{
  std::ofstream os((_path+name).c_str());
  if (os.is_open())
  {
    os << value;
  }
}

//-----------------------------------------------------------------------------

sensor::sensor() :
  _port(0),
  _type(0),
  _nvalues(0)
{
}

//-----------------------------------------------------------------------------
  
const mode_set &sensor::modes() const
{
  if (_modes.empty())
    const_cast<sensor*>(this)->read_modes();
  
  return _modes;
}

//-----------------------------------------------------------------------------
  
const mode_type &sensor::mode() const
{
  if (_mode.empty())
    const_cast<sensor*>(this)->read_modes();
  
  return _mode;
}

//-----------------------------------------------------------------------------

void sensor::read_modes()
{
}

//-----------------------------------------------------------------------------

const std::string &sensor::as_string(unsigned type)
{
  switch (type)
  {
  case nxt_touch:
    {
      static const std::string s("NXT touch");
      return s;
    }
  case nxt_light:
    {
      static const std::string s("NXT light");
      return s;
    }
  case nxt_sound:
    {
      static const std::string s("NXT sound");
      return s;
    }
  case nxt_color:
    {
      static const std::string s("NXT color");
      return s;
    }
  case nxt_ultrasonic:
    {
      static const std::string s("NXT ultrasonic");
      return s;
    }
  case nxt_temperature:
    {
      static const std::string s("NXT temperature");
      return s;
    }
  case ev3_touch:
    {
      static const std::string s("EV3 touch");
      return s;
    }
  case ev3_color:
    {
      static const std::string s("EV3 color");
      return s;
    }
  case ev3_ultrasonic:
    {
      static const std::string s("EV3 ultrasonic");
      return s;
    }
  case ev3_gyro:
    {
      static const std::string s("EV3 gyro");
      return s;
    }
  case ev3_infrared:
    {
      static const std::string s("EV3 infrared");
      return s;
    }
  default:
    break;
  }

  static const std::string s("<no string available>");
  return s;
}

//-----------------------------------------------------------------------------

msensor::msensor(unsigned type_, unsigned port_)
{
  init(type_, port_);
}

//-----------------------------------------------------------------------------

int msensor::value(unsigned index) const
{
  char svalue[8] = "value0";
  svalue[7] += index;
  
  return get_attr_int(svalue);
}

//-----------------------------------------------------------------------------

void msensor::set_mode(const mode_type &mode_)
{
  if (mode_ != _mode)
  {
    set_attr_string("mode", mode_);
    _mode.clear();
  }
}

//-----------------------------------------------------------------------------

bool msensor::init(unsigned type_, unsigned port_)
{
  if ((type_ == 0) && (port_ == 0))
    return false;
  
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
        string strDir = strClassDir + dp->d_name;
        ifstream is((strDir+"/type_id").c_str());
        if (is.is_open())
        {
          int type = 0;
          is >> type;
          if (is.bad() || (type_ && type != type_))
            continue;
          
          is.close();
          is.open((strDir+"/port_name").c_str());
          char c;
          int port = 0;
          is >> c >> c >> port;
          if (is.bad() || (port_ && (port != port_)))
            continue;
          
          is.close();
          is.open((strDir+"/num_values").c_str());
          int nvalues = 0;
          is >> nvalues;
          is.close();
          
          _port    = port;
          _type    = type;
          _nvalues = nvalues;
          _path    = strDir + '/';
          
          read_modes();
          
          return true;
        }
      }
    }
    closedir(dfd);
  }
  
  return false;
}

//-----------------------------------------------------------------------------

void msensor::read_modes()
{
  using namespace std;
  
  _modes.clear();
  _mode.clear();
  
  ifstream is((_path+"/mode").c_str());
  if (is.is_open())
  {
    string s;
    getline(is, s);
    
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
        if (*m.begin()=='[')
        {
          _mode = m.substr(1, m.length()-2);
          m = _mode;
        }
        _modes.insert(m);
      }
    } while (pos!=string::npos);
  }
}

//-----------------------------------------------------------------------------

touch_sensor     ::touch_sensor     (unsigned port_) : msensor(ev3_touch,      port_) { }
color_sensor     ::color_sensor     (unsigned port_) : msensor(ev3_color,      port_) { }
ultrasonic_sensor::ultrasonic_sensor(unsigned port_) : msensor(ev3_ultrasonic, port_) { }
gyro_sensor      ::gyro_sensor      (unsigned port_) : msensor(ev3_gyro,       port_) { }
infrared_sensor  ::infrared_sensor  (unsigned port_) : msensor(ev3_infrared,   port_) { }

//-----------------------------------------------------------------------------
  
motor::motor_type motor::motor_large ("tacho");
motor::motor_type motor::motor_medium("minitacho");

mode_type motor::mode_off("off");
mode_type motor::mode_on("on");

mode_type motor::run_mode_forever ("forever");
mode_type motor::run_mode_time    ("time");
mode_type motor::run_mode_position("position");
  
mode_type motor::polarity_mode_positive("positive");
mode_type motor::polarity_mode_negative("negative");
  
mode_type motor::position_mode_absolute("absolute");
mode_type motor::position_mode_relative("relative");

//-----------------------------------------------------------------------------

motor::motor(const motor_type &t, unsigned p) :
  _port(0)
{
  init(t, p);
}

//-----------------------------------------------------------------------------

bool motor::init(const motor_type &type_, unsigned port_)
{
  if (type_.empty() && (port_ == 0))
    return false;
  
  using namespace std;
  
  string strClassDir(SYS_ROOT "/class/tacho-motor/");
  
  struct dirent *dp;
  DIR *dfd;
  
  if ((dfd = opendir(strClassDir.c_str())) != NULL)
  {
    while ((dp = readdir(dfd)) != NULL)
    {
      if (strncmp(dp->d_name, "out", 3)==0)
      {
        _port = dp->d_name[3]-'A'+1;
        if ((_port < 1) || (_port > 4))
          continue;
        
        if (port_ && (_port != port_))
          continue;
        
        string strDir = strClassDir + dp->d_name;
        ifstream is((strDir+"/type").c_str());
        if (is.is_open())
        {
          is >> _type;
          if (is.bad() || (!type_.empty() && _type != type_))
            continue;
          is.close();
          
          _path = strDir + '/';
          
          return true;
        }
      }
    }
    closedir(dfd);
  }
  
  _port = 0;
  _type.clear();
  
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

int motor::power() const
{
  return get_attr_int("power");
}

//-----------------------------------------------------------------------------

int motor::speed() const
{
  return get_attr_int("speed");
}

//-----------------------------------------------------------------------------

int motor::position() const
{
  return get_attr_int("position");
}

//-----------------------------------------------------------------------------

int motor::pulses_per_second() const
{
  return get_attr_int("pulses_per_second");
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

mode_type motor::brake_mode() const
{
  return get_attr_string("brake_mode");
}

//-----------------------------------------------------------------------------

void motor::set_brake_mode(const mode_type &value)
{
  set_attr_string("brake_mode", value);
}

//-----------------------------------------------------------------------------

mode_type motor::hold_mode() const
{
  return get_attr_string("hold_mode");
}
  
//-----------------------------------------------------------------------------

void motor::set_hold_mode(const mode_type &value)
{
  set_attr_string("hold_mode", value);
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

mode_type motor::polarity_mode() const
{
  return get_attr_string("polarity_mode");
}
//-----------------------------------------------------------------------------

void motor::set_polarity_mode(const mode_type &value)
{
  set_attr_string("polarity_mode", value);
}

//-----------------------------------------------------------------------------

int motor::speed_setpoint() const
{
  return get_attr_int("speed_setpoint");
}
//-----------------------------------------------------------------------------

void motor::set_speed_setpoint(int value)
{
  set_attr_int("speed_setpoint", value);
}

//-----------------------------------------------------------------------------

int  motor::time_setpoint() const
{
  return get_attr_int("time_setpoint");
}

//-----------------------------------------------------------------------------

void motor::set_time_setpoint(int value)
{
  set_attr_int("time_setpoint", value);
}

//-----------------------------------------------------------------------------

int  motor::position_setpoint() const
{
  return get_attr_int("position_setpoint");
}
  
//-----------------------------------------------------------------------------

void motor::set_position_setpoint(int value)
{
  set_attr_int("position_setpoint", value);
}

//-----------------------------------------------------------------------------

int  motor::ramp_up() const
{
  return get_attr_int("ramp_up");
}

//-----------------------------------------------------------------------------

void motor::set_ramp_up(int value)
{
  set_attr_int("ramp_up", value);
}

//-----------------------------------------------------------------------------

int motor::ramp_down() const
{
  return get_attr_int("ramp_down");
}

//-----------------------------------------------------------------------------

void motor::set_ramp_down(int value)
{
  set_attr_int("ramp_down", value);
}

//-----------------------------------------------------------------------------

medium_motor::medium_motor(unsigned port_) : motor(motor_medium, port_)
{
}

//-----------------------------------------------------------------------------

large_motor::large_motor(unsigned port_) : motor(motor_large, port_)
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

led led::red_right  ("red:right");
led led::red_left   ("red:left");
led led::green_right("green:right");
led led::green_left ("green:left");

//-----------------------------------------------------------------------------

void led::red_on   () { red_right  .on();  red_left  .on();  }
void led::red_off  () { red_right  .off(); red_left  .off(); }
void led::green_on () { green_right.on();  green_left.on();  }
void led::green_off() { green_right.off(); green_left.off(); }
void led::all_on   () { red_on();  green_on();  }
void led::all_off  () { red_off(); green_off(); }

//-----------------------------------------------------------------------------
  
button::button(const std::string &name)
{
  _path = std::string(SYS_BUTTON + name);
}
  
//-----------------------------------------------------------------------------

bool button::pressed() const
{
  std::ifstream is(_path.c_str());
  
  int result = 0;
  
  if (is.is_open())
  {
    is >> result;
  }
  
  return (result != 0);
}

//-----------------------------------------------------------------------------

button button::back ("back");
button button::left ("left");
button button::right("right");
button button::up   ("up");
button button::down ("down");
button button::enter("enter");

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
  
} // namespace ev3dev

//-----------------------------------------------------------------------------
