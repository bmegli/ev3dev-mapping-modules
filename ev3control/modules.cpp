#include "modules.h"

#include "shared/misc.h"

#include <stdio.h> //fprintf
#include <unistd.h> //fork
#include <fcntl.h> //O_NONBLOCK
#include <sys/wait.h> //wait
#include <errno.h> //errno
#include <string.h> //memcpy
#include <map>
#include <string>

using namespace std;

void PrepareExecvArgumentList(const robot_module &module, const char* argv[MAX_ROBOT_MODULE_ARGUMENTS], char call[MODULE_CALL_WITH0_MAX])
{
	memcpy(call, module.call.c_str(), MODULE_CALL_WITH0_MAX);
		
	char *token=strtok(call, " ");
	int t;
	
	for(t=0; token!=NULL; ++t)
	{
		argv[t]=token;
		token=strtok(NULL, " ");
	}
	
	argv[t]=NULL; //the last one
	
}

// Processes to be created with fork/pipe/execv
void EnableModule(map<string, robot_module> &modules,const string &which)
{
	if( modules.find(which) == modules.end() )
	{
		fprintf(stderr, "Request to enable module but no such module\n");
		return;
	}

	robot_module module=modules[which];

	if(module.state == MODULE_ENABLED)
	{
		fprintf(stderr, "Request to enable module but module is already enabled\n");
		return;
	}
	
	int pipe_read_write[2];
	if( pipe2(pipe_read_write, O_NONBLOCK) == -1 )
		DieErrno("PrepareChild() pipe failed");

	pid_t pid=fork();
	
	if(pid==-1)
		DieErrno("PrepareChild() fork failed");
	if(pid==0) //child
	{
		close(pipe_read_write[1]);
		if( dup2(pipe_read_write[0], STDIN_FILENO) == -1)
			DieErrno("PrepareChild() child dup2 failed");

		//close the descriptors for older children		
		
		map<string, robot_module>::iterator it;
		
		for(it=modules.begin();it!=modules.end();++it)
			if(it->second.state == MODULE_ENABLED)
				close(it->second.write_fd);
	
		const char *argv[MAX_ROBOT_MODULE_ARGUMENTS];
		char call[MODULE_CALL_WITH0_MAX]; //strtok destroys the data, we need a copy
	
		PrepareExecvArgumentList(module, argv, call);
	
		if( execv(argv[0], (char* const*)argv) == -1 )
			DieErrno("PrepareChild() execv failed");
	}
	
	// parent pid points to child
	close(pipe_read_write[0]);
	
	module.state=MODULE_ENABLED;
	module.pid=pid;
	module.write_fd=pipe_read_write[1];
	modules[which]=module;
	Sleep(module.creation_sleep_ms);
	
	printf("Enabled module: %s\n", module.unique_name.c_str());
}
void DisableModule(map<string, robot_module> &modules,const string &which)
{
	if( modules.find(which) == modules.end() )
	{
		fprintf(stderr, "Request to disable module but no such module\n");
		return;
	}

	robot_module module=modules[which];
	
	if(module.state != MODULE_ENABLED)
	{
		fprintf(stderr, "Module not enabled, can't disable\n");
		return;
	}
	
	close(module.write_fd);
		
	int status;	
		
	pid_t pid=waitpid(module.pid, &status, 0);		
	if(pid == -1)
		perror("DisableModule() waitpid error");
		
	module.state=MODULE_DISABLED;
	module.write_fd=0;
	module.pid=0;
	modules[which]=module;
	printf("Disabled module: %s\n", module.unique_name.c_str());
}


void DisableModules(map<string, robot_module> &modules)
{		
	map<string, robot_module>::iterator it;
	
	for(it=modules.begin();it!=modules.end();++it)
	{
		robot_module module=it->second;
		
		if(module.state != MODULE_ENABLED)
			continue;
		DisableModule(modules, module.unique_name);		
	}	
}

bool EnabledModulesRunning(map<string, robot_module> &modules)
{
	int status, ret;
	
	map<string, robot_module>::iterator it;
	
	bool all_ok=true;
	
	for(it=modules.begin();it!=modules.end();++it)
	{
		robot_module module=it->second;
		
		if(module.state!=MODULE_ENABLED)
			continue;
			
		ret=waitpid(module.pid, &status, WNOHANG );

		if(ret == 0) //still running
			continue;

		if(ret == module.pid)
		{
			fprintf(stderr, "Module %s is terminated, should be running!\n", module.unique_name.c_str());
			close(module.write_fd);
			module.state=MODULE_FAILED;
			module.write_fd=0;
			module.pid=0;
			modules[module.unique_name]=module;
			
			all_ok = false;
		}	
		//otherwise error, for now print it
		
		perror("EnabledModulesRunning, waitpid failed");
	}
	
	return all_ok;
}