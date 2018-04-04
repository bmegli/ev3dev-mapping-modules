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


#ifndef PWM_H
#define PWM_H

#include <string>

class device
{
public:
    bool connect(const std::string& dir);

    inline bool connected() const {return !m_path.empty();}

    int get_attr_int(const std::string& name) const;
    void set_attr_int(const std::string& name, int value);
    std::string get_attr_string(const std::string& name) const;
    void set_attr_string(const std::string& name, const std::string& value);

protected:
    std::string m_path;
};

class pwm : public device
{
public:
    pwm();
    ~pwm();

    int get_period() const;
    void set_period(int period);

    int get_duty_cycle() const;
    void set_duty_cycle(int duty_cycle);

    int get_duty_cycle_pct();
    void set_duty_cycle_pct(int duty_cycle_pct);

    bool get_enable() const;
    void set_enable(bool enable);

protected:
    int m_period;
};

#endif // PWM_H
