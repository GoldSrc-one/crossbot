#include "entity_parser.h"

#include <cctype>

namespace navgen
{

const std::string *Entity::Find(const std::string &key) const
{
	auto it = keyvalues.find(key);
	return it == keyvalues.end() ? nullptr : &it->second;
}

std::string Entity::Get(const std::string &key, const std::string &def) const
{
	const std::string *v = Find(key);
	return v ? *v : def;
}

namespace
{

// Reads one double-quoted token starting at text[pos] (which must be '"'),
// advancing pos past the closing quote. No escape-sequence handling -- the
// GoldSrc entity-lump grammar doesn't use any (values may contain literal
// backslashes, e.g. Windows paths, with no special meaning).
std::string ReadQuoted(const std::string &text, size_t &pos)
{
	++pos; // skip opening quote
	size_t start = pos;
	while (pos < text.size() && text[pos] != '"')
		++pos;
	std::string result = text.substr(start, pos - start);
	if (pos < text.size())
		++pos; // skip closing quote
	return result;
}

} // namespace

std::vector<Entity> ParseEntities(const std::string &entityText)
{
	std::vector<Entity> entities;
	size_t pos = 0;
	size_t len = entityText.size();

	while (pos < len)
	{
		while (pos < len &&
			(std::isspace(static_cast<unsigned char>(entityText[pos])) || entityText[pos] == '\0'))
			++pos;
		if (pos >= len)
			break;

		if (entityText[pos] != '{')
		{
			++pos; // skip unexpected token outside a block
			continue;
		}
		++pos; // skip '{'

		Entity ent;
		while (pos < len)
		{
			while (pos < len && std::isspace(static_cast<unsigned char>(entityText[pos])))
				++pos;
			if (pos >= len)
				break;
			if (entityText[pos] == '}')
			{
				++pos;
				break;
			}
			if (entityText[pos] != '"')
			{
				++pos; // skip unexpected character between keyvalue pairs
				continue;
			}
			std::string key = ReadQuoted(entityText, pos);

			while (pos < len && std::isspace(static_cast<unsigned char>(entityText[pos])))
				++pos;
			if (pos >= len || entityText[pos] != '"')
				break; // malformed keyvalue pair; stop this entity here

			std::string value = ReadQuoted(entityText, pos);
			ent.keyvalues[key] = value;
		}

		entities.push_back(std::move(ent));
	}

	return entities;
}

} // namespace navgen
