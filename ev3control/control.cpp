/*
 * ev3dev-mapping ev3control Control class implementation file
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

#include "control.h"

#include "shared/misc.h"

#include <sys/wait.h> //wait
#include <unistd.h> //close
#include <fcntl.h> //O_NONBLOCK
#include <string.h> //strtok

using namespace std;

int32_t ModuleReturnValue(int32_t status)
{
	if( WIFEXITED(status) )
		return WEXITSTATUS(status);
	return -1;
}

Control::Control()
{
}

Control::~Control()
{
	DisableModules();
}

bool Control::ContainsModule(const std::string& name, Module* module)
{
	map<std::string, Module>::iterator it=modules.find(name);
	if(it == modules.end())
		return false;
	
	*module=it->second;
	return true;
}

void Control::InsertModule(const std::string& name, int creation_delay_ms)
{
	Module m={MODULE_DISABLED, creation_delay_ms, 0, 0, 0};
	modules.insert( pair<string, Module>(name, m ) );
}

void Control::EnableModule(const std::string& name, const std::string& module_call)
{
	Module module;

	if( !ContainsModule(name, &module) )
		Die("Control: request to enable module but no such module\n");

	if(module.state == MODULE_ENABLED)
		Die("Control: request to enable module but module already enabled\n");
	
	int pipe_read_write[2];
	if( pipe2(pipe_read_write, O_NONBLOCK) == -1 )
		DieErrno("PrepareChild() pipe failed");

	pid_t pid=fork();
	
	if(pid==-1)
		DieErrno("Cotrol: EnableModule fork failed\n");
	if(pid==0) //child
	{
		close(pipe_read_write[1]);
		if( dup2(pipe_read_write[0], STDIN_FILENO) == -1)
			DieErrno("Control: EnableModule child dup2 failed\n");

		//close the descriptors for older children		
		
		map<string, Module>::iterator it;
		
		for(it=modules.begin();it!=modules.end();++it)
			if(it->second.state == MODULE_ENABLED)
				close(it->second.write_fd);
	
		string argv_str;
		vector<char *> argv=PrepareExecvArgumentList(module_call, argv_str);
	
		if( execv(argv[0], &argv[0]) == -1 )
			DieErrno("Control: EnableModule execv failed\n");
	}
	
	// parent pid points to child
	close(pipe_read_write[0]);
	
	module.state=MODULE_ENABLED;
	module.pid=pid;
	module.write_fd=pipe_read_write[1];
	module.return_value=0;
	modules[name]=module;
	Sleep(module.creation_delay_ms);
}

vector<char*> Control::PrepareExecvArgumentList(const string& module_call, string &out_argv_string)
{
	out_argv_string=module_call;
	char *argv=(char*)out_argv_string.c_str();
	vector<char *> result;
	char *token=strtok(argv, " ");
	
	while(token!=NULL)
	{
		result.push_back(token);
		token=strtok(NULL, " ");
	}
	
	result.push_back(NULL);
		
	return result;
}


void Control::DisableModule(const std::string &name)
{
	Module module;

	if( !ContainsModule(name, &module) )
		Die("Control: request to disable module but no such module\n");
	
	if(module.state != MODULE_ENABLED)
		Die("Control: request to disable module but module is not enabled\n");
		
	close(module.write_fd);
		
	int status;	
		
	pid_t pid=waitpid(module.pid, &status, 0);		
	
	if(pid == -1)
		DieErrno("Control: DisableModule waitpid error for module\n");
		
	module.state=MODULE_DISABLED;
	module.write_fd=0;
	module.pid=0;
	
	modules[name]=module;
}

std::list<std::string> Control::DisableModules()
{
	map<string, Module>::iterator it;
	list<std::string> disabled;
	
	for(it=modules.begin();it!=modules.end();++it)
	{
		Module module=it->second;
		
		if(module.state != MODULE_ENABLED)
			continue;

		DisableModule(it->first);
		disabled.push_back(it->first);
	}	
	return disabled;
}

ModuleState Control::CheckModuleState(const std::string& name)
{
	map<std::string, Module>::iterator it=modules.find(name);
	if( it == modules.end() )
		Die("Control: request to check module state but module is not present\n");
	
	Module module=it->second;

	// if not enabled just return state
	if(module.state != MODULE_ENABLED )
		return module.state;
		
	// otherwise check state and return it
	
	int status;
	int ret=waitpid(module.pid, &status, WNOHANG );

	if(ret == 0) //still running
		return MODULE_ENABLED;

	if(ret == module.pid)
	{
		close(module.write_fd);
		module.state=MODULE_FAILED;
		module.write_fd=0;
		module.pid=0;
		module.return_value=ModuleReturnValue(status);
		modules[name]=module;
		return MODULE_FAILED;
	}

	DieErrno("Control: CheckModuleState waitpid failed\n");
	return module.state;
}

std::list<FailedModule> Control::CheckModulesStates()
{
	list<FailedModule> failed;
	
	int status, ret;
	
	map<string, Module>::iterator it;
		
	for(it=modules.begin();it!=modules.end();++it)
	{
		Module module=it->second;
		
		if(module.state != MODULE_ENABLED)
			continue;
			
		ret=waitpid(module.pid, &status, WNOHANG );

		if(ret == 0) //still running
			continue;
		if(ret == -1)
			DieErrno("Control: CheckModuleStates waitpid error\n");
		if(ret == module.pid)
		{
			close(module.write_fd);
			module.state=MODULE_FAILED;
			module.write_fd=0;
			module.pid=0;
			module.return_value = ModuleReturnValue(status);
			modules[it->first]=module;
			
			FailedModule fail={it->first, module.return_value};
			
			failed.push_back(fail);
		}
	}	
	return failed;
}
