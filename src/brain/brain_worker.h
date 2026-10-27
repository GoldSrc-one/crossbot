#pragma once

#include <string>
#include <vector>

#include "brain_config.h"

namespace BrainWorker
{

struct Job
{
	int brain = 0;
	unsigned int id = 0;
	unsigned int epoch = 0;
	BrainConfig::Config config;
	std::string jevBody;
	std::vector<std::string> bots;
	std::string context;
	std::string tracePath;
};

struct Call
{
	std::string name;
	std::string args;
};

struct Result
{
	int brain = 0;
	unsigned int id = 0;
	unsigned int epoch = 0;

	bool jevOk = false;
	float act = 0.f;
	float remember = 0.f;
	float ignoreSeconds = 0.f;
	float recheckSeconds = 0.f;
	std::vector<float> involve;

	bool llmCalled = false;
	bool llmOk = false;
	std::vector<Call> calls;
	std::string content;

	std::string error;
	float jevSeconds = 0.f;
	float llmSeconds = 0.f;
	int jevTokens = 0;
	int llmIn = 0;
	int llmOut = 0;
	std::string jevRaw;
	std::string llmRaw;
};

void Submit(Job &&job);

bool Poll(Result &out);

int Busy();

void Stop();

}
