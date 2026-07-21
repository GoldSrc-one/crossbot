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

// Parses the raw LUMP_ENTITIES text block (the classic Quake/GoldSrc
// `{ "key" "value" ... }` grammar -- no vendored parser exists for this
// anywhere in external/hlsdk) into a flat list of entities. Malformed
// trailing content is skipped defensively rather than thrown on, since a
// truncated/odd entity lump shouldn't take down the whole tool.
std::vector<Entity> ParseEntities(const std::string &entityText);

} // namespace navgen
