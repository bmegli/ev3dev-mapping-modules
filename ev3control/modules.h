#pragma once

#include <sys/types.h> //pid_t
#include <map>
#include <string>

const int MAX_ROBOT_MODULE_ARGUMENTS=10;

const int MODULE_NAME_WITH0_MAX=20;
const int MODULE_CALL_WITH0_MAX=128;

enum ModuleState {MODULE_DISABLED=0, MODULE_ENABLED=1, MODULE_FAILED=2}; 

struct robot_module
{
	std::string unique_name;
	std::string call;
	ModuleState state;
	int creation_sleep_ms;
	int write_fd;
	pid_t pid;
};

void EnableModule(std::map<std::string, robot_module> &modules,const std::string &which);
void DisableModule(std::map<std::string, robot_module> &modules, const std::string &which);
void DisableModules(std::map<std:: string, robot_module> &modules);
bool EnabledModulesRunning(std::map<std::string, robot_module> &modules);
