#include <cstdio>
#include <cstdlib>
#include <cstring>

#define JSMN_STATIC
#include <jsmn.h>

#include "json.h"

namespace
{
	Json::Type MapType(jsmntype_t type)
	{
		switch (type)
		{
		case JSMN_OBJECT: return Json::T_OBJECT;
		case JSMN_ARRAY: return Json::T_ARRAY;
		case JSMN_STRING: return Json::T_STRING;
		case JSMN_PRIMITIVE: return Json::T_PRIMITIVE;
		default: break;
		}
		return Json::T_UNDEFINED;
	}

	int HexDigit(char c)
	{
		if (c >= '0' && c <= '9')
			return c - '0';
		if (c >= 'a' && c <= 'f')
			return c - 'a' + 10;
		if (c >= 'A' && c <= 'F')
			return c - 'A' + 10;
		return -1;
	}

	void AppendUtf8(std::string &out, unsigned int cp)
	{
		if (cp < 0x80)
		{
			out += static_cast<char>(cp);
		}
		else if (cp < 0x800)
		{
			out += static_cast<char>(0xC0 | (cp >> 6));
			out += static_cast<char>(0x80 | (cp & 0x3F));
		}
		else if (cp < 0x10000)
		{
			out += static_cast<char>(0xE0 | (cp >> 12));
			out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
			out += static_cast<char>(0x80 | (cp & 0x3F));
		}
		else
		{
			out += static_cast<char>(0xF0 | (cp >> 18));
			out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
			out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
			out += static_cast<char>(0x80 | (cp & 0x3F));
		}
	}

	bool ReadHex4(const char *p, const char *end, unsigned int &out)
	{
		if (end - p < 4)
			return false;
		out = 0;
		for (int i = 0; i < 4; ++i)
		{
			int d = HexDigit(p[i]);
			if (d < 0)
				return false;
			out = (out << 4) | static_cast<unsigned int>(d);
		}
		return true;
	}
}

namespace Json
{

bool Doc::Parse(const char *src, size_t len)
{
	text.assign(src ? src : "", src ? len : 0);
	toks.clear();

	jsmn_parser parser;
	jsmn_init(&parser);
	int need = jsmn_parse(&parser, text.data(), text.size(), nullptr, 0);
	if (need <= 0)
		return false;

	std::vector<jsmntok_t> raw(static_cast<size_t>(need));
	jsmn_init(&parser);
	int got = jsmn_parse(&parser, text.data(), text.size(), raw.data(), static_cast<unsigned int>(need));
	if (got != need)
		return false;

	toks.resize(raw.size());
	for (size_t i = 0; i < raw.size(); ++i)
	{
		toks[i].type = MapType(raw[i].type);
		toks[i].start = raw[i].start;
		toks[i].end = raw[i].end;
		toks[i].size = raw[i].size;
	}
	return true;
}

Type Doc::TypeOf(int tok) const
{
	if (tok < 0 || tok >= static_cast<int>(toks.size()))
		return T_UNDEFINED;
	return toks[tok].type;
}

int Doc::Count(int tok) const
{
	Type t = TypeOf(tok);
	return (t == T_OBJECT || t == T_ARRAY) ? toks[tok].size : 0;
}

int Doc::Skip(int tok) const
{
	if (tok < 0 || tok >= static_cast<int>(toks.size()))
		return static_cast<int>(toks.size());
	int children = toks[tok].size;
	int i = tok + 1;
	for (int k = 0; k < children; ++k)
		i = Skip(i);
	return i;
}

int Doc::Member(int obj, const char *name) const
{
	if (TypeOf(obj) != T_OBJECT || !name)
		return -1;

	size_t len = strlen(name);
	int i = obj + 1;
	for (int c = 0; c < toks[obj].size; ++c)
	{
		int key = i;
		i = Skip(key);
		const Tok &k = toks[key];
		if (k.type == T_STRING && static_cast<size_t>(k.end - k.start) == len &&
			strncmp(text.data() + k.start, name, len) == 0)
			return key + 1;
	}
	return -1;
}

int Doc::Key(int obj, int index) const
{
	if (TypeOf(obj) != T_OBJECT || index < 0 || index >= toks[obj].size)
		return -1;
	int i = obj + 1;
	for (int c = 0; c < index; ++c)
		i = Skip(i);
	return i;
}

int Doc::Element(int arr, int index) const
{
	if (TypeOf(arr) != T_ARRAY || index < 0 || index >= toks[arr].size)
		return -1;
	int i = arr + 1;
	for (int c = 0; c < index; ++c)
		i = Skip(i);
	return i;
}

std::string Doc::Raw(int tok) const
{
	if (TypeOf(tok) == T_UNDEFINED)
		return std::string();
	return text.substr(toks[tok].start, toks[tok].end - toks[tok].start);
}

std::string Doc::Str(int tok) const
{
	Type t = TypeOf(tok);
	if (t != T_STRING && t != T_PRIMITIVE)
		return std::string();

	const char *p = text.data() + toks[tok].start;
	const char *end = text.data() + toks[tok].end;
	if (t == T_PRIMITIVE)
		return std::string(p, end);

	std::string out;
	out.reserve(static_cast<size_t>(end - p));
	while (p < end)
	{
		char c = *p++;
		if (c != '\\' || p >= end)
		{
			out += c;
			continue;
		}

		char e = *p++;
		switch (e)
		{
		case 'n': out += '\n'; break;
		case 't': out += '\t'; break;
		case 'r': out += '\r'; break;
		case 'b': out += '\b'; break;
		case 'f': out += '\f'; break;
		case 'u':
		{
			unsigned int cp = 0;
			if (!ReadHex4(p, end, cp))
				break;
			p += 4;
			if (cp >= 0xD800 && cp < 0xDC00 && end - p >= 6 && p[0] == '\\' && p[1] == 'u')
			{
				unsigned int lo = 0;
				if (ReadHex4(p + 2, end, lo) && lo >= 0xDC00 && lo < 0xE000)
				{
					cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
					p += 6;
				}
			}
			AppendUtf8(out, cp);
			break;
		}
		default: out += e; break;
		}
	}
	return out;
}

bool Doc::Num(int tok, float &out) const
{
	Type t = TypeOf(tok);
	if (t != T_PRIMITIVE && t != T_STRING)
		return false;
	std::string s = Str(tok);
	if (s.empty())
		return false;
	char *end = nullptr;
	double v = strtod(s.c_str(), &end);
	if (end == s.c_str())
		return false;
	out = static_cast<float>(v);
	return true;
}

bool Doc::Bool(int tok, bool &out) const
{
	if (TypeOf(tok) != T_PRIMITIVE)
		return false;
	char c = text[static_cast<size_t>(toks[tok].start)];
	if (c == 't' || c == 'f')
	{
		out = c == 't';
		return true;
	}
	float v = 0.f;
	if (!Num(tok, v))
		return false;
	out = v != 0.f;
	return true;
}

bool Doc::IsNull(int tok) const
{
	return TypeOf(tok) == T_PRIMITIVE && text[static_cast<size_t>(toks[tok].start)] == 'n';
}

std::string Doc::GetStr(int obj, const char *name, const char *def) const
{
	int tok = Member(obj, name);
	if (tok < 0 || IsNull(tok))
		return def ? def : "";
	return Str(tok);
}

float Doc::GetNum(int obj, const char *name, float def) const
{
	float v = def;
	return Num(Member(obj, name), v) ? v : def;
}

bool Doc::GetBool(int obj, const char *name, bool def) const
{
	bool v = def;
	return Bool(Member(obj, name), v) ? v : def;
}

std::string Escape(const char *s)
{
	std::string out;
	if (!s)
		return out;
	for (const unsigned char *p = reinterpret_cast<const unsigned char *>(s); *p; ++p)
	{
		switch (*p)
		{
		case '"': out += "\\\""; break;
		case '\\': out += "\\\\"; break;
		case '\n': out += "\\n"; break;
		case '\r': out += "\\r"; break;
		case '\t': out += "\\t"; break;
		default:
			if (*p < 0x20)
			{
				char buf[8];
				snprintf(buf, sizeof(buf), "\\u%04x", *p);
				out += buf;
			}
			else
			{
				out += static_cast<char>(*p);
			}
			break;
		}
	}
	return out;
}

std::string Quote(const char *s)
{
	return "\"" + Escape(s) + "\"";
}

}
