#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace Json
{

enum Type
{
	T_UNDEFINED = 0,
	T_OBJECT,
	T_ARRAY,
	T_STRING,
	T_PRIMITIVE,
};

struct Tok
{
	Type type = T_UNDEFINED;
	int start = 0;
	int end = 0;
	int size = 0;
};

class Doc
{
public:
	bool Parse(const char *text, size_t len);
	bool Parse(const std::string &text) { return Parse(text.data(), text.size()); }

	int Root() const { return toks.empty() ? -1 : 0; }
	Type TypeOf(int tok) const;
	int Count(int tok) const;
	int Member(int obj, const char *name) const;
	int Element(int arr, int index) const;
	int Key(int obj, int index) const;
	int Skip(int tok) const;

	std::string Str(int tok) const;
	std::string Raw(int tok) const;
	bool Num(int tok, float &out) const;
	bool Bool(int tok, bool &out) const;
	bool IsNull(int tok) const;

	std::string GetStr(int obj, const char *name, const char *def = "") const;
	float GetNum(int obj, const char *name, float def) const;
	bool GetBool(int obj, const char *name, bool def) const;

private:
	std::string text;
	std::vector<Tok> toks;
};

std::string Escape(const char *s);

std::string Quote(const char *s);

}
