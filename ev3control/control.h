/*
 * ev3dev-mapping ev3control Control class header file
 *
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#pragma once

#include <sys/types.h> //pid_t

#include <string> //string
#include <map> //map
#include <list> //list
#include <vector> //vector

enum ModuleState {MODULE_DISABLED=0, MODULE_ENABLED=1, MODULE_FAILED=2}; 

struct Module
{
	ModuleState state;
	int creation_delay_ms;
	int write_fd;
	pid_t pid;
	int32_t return_value;
};

struct FailedModule
{
	std::string name;
	int status;
};

class Control
{
private:
	std::map<std::string, Module> modules;
	
	std::vector<char *> PrepareExecvArgumentList(const std::string &module_call, std::string &out_argv_string);
public:
	Control();
	~Control();
	
	bool ContainsModule(const std::string &name, Module *module);
	void InsertModule(const std::string &name, int creation_delay_ms);
	void EnableModule(const std::string &name, const std::string &call);
	void DisableModule(const std::string &name);
	std::list<std::string> DisableModules();

	ModuleState CheckModuleState(const std::string &name);
	std::list<FailedModule> CheckModulesStates();	
};

