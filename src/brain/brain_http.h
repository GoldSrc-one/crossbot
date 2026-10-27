#pragma once

#include <string>

namespace BrainHttp
{

struct Response
{
	int status = 0;
	std::string body;
	std::string type;
	std::string error;
	float seconds = 0.f;
};

bool Post(const std::string &url, const std::string &key, const std::string &body, float timeout, Response &out);

void Shutdown();

}
