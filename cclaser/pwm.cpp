/*
 *
 * Copyright (C) 2018 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

 /*   
  * Temporary library for sysfs pwm control header
  * TO DO: implement device search or use udev
  */


#include "pwm.h"

#include <dirent.h>  //DIR, opendir, closedir
#include <fstream>   //ifstream, ofstream
#include <stdexcept> //runtime error
	
using namespace std;

const int PERIOD_UNKNOWN = 0;

bool device::connect(const std::string& dir)
{
    DIR* dfd;

    if((dfd = opendir(dir.c_str())) != NULL)
	{
	    m_path = dir + "/";
	    closedir(dfd);
	    return true;
	}

    return false;
}

int device::get_attr_int(const std::string& name) const
{
    if(m_path.empty())
		throw runtime_error("no device connected");

    string path = m_path + name;

    ifstream is(path.c_str());

    if(is.is_open())
	{
	    int result = 0;
	    is >> result;
	    return result;
	}
    throw runtime_error("no such device: " + path);
}

void device::set_attr_int(const std::string& name, int value)
{
    if(m_path.empty())
		throw runtime_error("no device connected");

    string path = m_path + name;

    ofstream os(path.c_str());

    if(os.is_open())
	{
	    if(!(os << value))
		throw runtime_error("unable to write to: " + path);
	    return;
	}
    throw runtime_error("no such device: " + path);
}

std::string device::get_attr_string(const std::string& name) const
{
    if(m_path.empty())
		throw runtime_error("no device connected");

    string path = m_path + name;

    ifstream is(path.c_str());
    if(is.is_open())
	{
	    string result;
	    is >> result;
	    return result;
	}

    throw runtime_error("no such device: " + path);
}

void device::set_attr_string(const std::string& name, const std::string& value)
{
    if(m_path.empty())
		throw runtime_error("no device connected");

    string path = m_path + name;

    ofstream os(path.c_str());
    if(os.is_open())
	{
	    if(!(os << value))
		throw runtime_error("unable to write to: " + path);
	    return;
	}

    throw runtime_error("no such device: " + path);
}

pwm::pwm()
    : m_period(PERIOD_UNKNOWN)
{
}

pwm::~pwm()
{
    if(!connected())
		return;
    try
	{
	    set_duty_cycle(0);
	    set_enable(0);
	}
    catch(...)
	{
	}
}

int pwm::get_period() const
{
    return get_attr_int("period");
}

void pwm::set_period(int period)
{
    set_attr_int("period", period);
}

int pwm::get_duty_cycle() const
{
    return get_attr_int("duty_cycle");
}

void pwm::set_duty_cycle(int duty_cycle)
{
    set_attr_int("duty_cycle", duty_cycle);
}

int pwm::get_duty_cycle_pct()
{
    if(m_period == PERIOD_UNKNOWN)
		m_period = get_period();
    return get_duty_cycle() * 100 / m_period;
}

void pwm::set_duty_cycle_pct(int duty_cycle_pct)
{
    if(m_period == PERIOD_UNKNOWN)
		m_period = get_period();
    set_duty_cycle(duty_cycle_pct * m_period / 100);
}

bool pwm::get_enable() const
{
    return get_attr_int("period");
}

void pwm::set_enable(bool enable)
{
    set_attr_int("enable", enable);
}
