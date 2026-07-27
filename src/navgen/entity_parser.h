#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace navgen
{

struct Entity
{
	std::unordered_map<std::string, std::string> keyvalues;

	const std::string *Find(const std::string &key) const;
	std::string Get(const std::string &key, const std::string &def = "") const;
};

std::vector<Entity> ParseEntities(const std::string &entityText);

}

