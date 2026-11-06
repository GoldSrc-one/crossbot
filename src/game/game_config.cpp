#include <extdll.h>
#include <meta_api.h>

#include <stdio.h>
#include <string.h>

#include "../brain/json.h"
#include "game_config.h"

namespace
{
	constexpr int kMaxBaseDepth = 4;
	constexpr int kMaxWeaponId = 255;

	const char *const kAmbushNames[AMBUSH_COUNT] = { "grenade", "snark", "penguin", "grapple", "flash", "smoke" };
	const char *const kDeployNames[DEPLOY_COUNT] = { "tripmine", "satchel" };
	const char *const kItemKindNames[kItemKindCount] = { "none", "weapon", "ammo", "pickup", "healthcharger",
		"recharge", "healvolume", "station" };

	struct Loader
	{
		const Json::Doc &doc;
		GameConfig &cfg;
		std::string &why;
		std::string where;

		bool Fail(const std::string &what)
		{
			why = where + ": " + what;
			return false;
		}

		const char *Keep(const std::string &s)
		{
			cfg.strings.push_back(s);
			return cfg.strings.back().c_str();
		}

		bool Str(int tok, const char *&out)
		{
			if (doc.IsNull(tok))
			{
				out = nullptr;
				return true;
			}
			if (doc.TypeOf(tok) != Json::T_STRING)
				return Fail("expected a string");
			out = Keep(doc.Str(tok));
			return true;
		}

		bool Num(int tok, float &out)
		{
			if (doc.TypeOf(tok) != Json::T_PRIMITIVE || !doc.Num(tok, out))
				return Fail("expected a number");
			return true;
		}

		bool Int(int tok, int &out)
		{
			float v = 0.f;
			if (!Num(tok, v))
				return false;
			out = static_cast<int>(v);
			if (static_cast<float>(out) != v)
				return Fail("expected an integer");
			return true;
		}

		bool Bool(int tok, bool &out)
		{
			if (doc.TypeOf(tok) != Json::T_PRIMITIVE || !doc.Bool(tok, out))
				return Fail("expected true or false");
			return true;
		}

		template <typename E>
		bool Enum(int tok, const char *(*name)(E), int count, E &out)
		{
			std::string s = doc.Str(tok);
			for (int i = 0; i < count; ++i)
			{
				if (s == name(static_cast<E>(i)))
				{
					out = static_cast<E>(i);
					return true;
				}
			}
			return Fail("unknown value '" + s + "'");
		}

		bool Pick(int tok, const char *const *names, int count, int &out)
		{
			std::string s = doc.Str(tok);
			for (int i = 0; i < count; ++i)
			{
				if (s == names[i])
				{
					out = i;
					return true;
				}
			}
			return Fail("unknown value '" + s + "'");
		}

		bool Object(int tok)
		{
			if (doc.TypeOf(tok) != Json::T_OBJECT)
				return Fail("expected an object");
			return true;
		}

		bool Array(int tok)
		{
			if (doc.TypeOf(tok) != Json::T_ARRAY)
				return Fail("expected an array");
			return true;
		}

		template <typename F>
		bool Members(int obj, F &&each)
		{
			if (!Object(obj))
				return false;
			std::string outer = where;
			for (int i = 0; i < doc.Count(obj); ++i)
			{
				int key = doc.Key(obj, i);
				std::string name = doc.Str(key);
				where = outer + "." + name;
				if (!each(name, key + 1))
					return false;
			}
			where = outer;
			return true;
		}

		bool Quirks(int tok, unsigned int &out)
		{
			if (!Array(tok))
				return false;
			out = WPN_Q_NONE;
			for (int i = 0; i < doc.Count(tok); ++i)
			{
				std::string s = doc.Str(doc.Element(tok, i));
				int k = 0;
				while (k < kWeaponQuirkNameCount && s != kWeaponQuirkNames[k].name)
					++k;
				if (k == kWeaponQuirkNameCount)
					return Fail("unknown quirk '" + s + "'");
				out |= kWeaponQuirkNames[k].bit;
			}
			return true;
		}

		bool Mode(int obj, WeaponMode &m)
		{
			return Members(obj, [&](const std::string &key, int v) {
				if (key == "fire") return Enum(v, WeaponFireName, FIRE_CHARGE + 1, m.fire);
				if (key == "aim") return Enum(v, WeaponAimName, AIM_SURFACE + 1, m.aim);
				if (key == "toggle") return Enum(v, WeaponToggleName, TOGGLE_LASER + 1, m.toggle);
				if (key == "ammo") return Enum(v, WeaponAmmoName, AMMO_SECONDARY + 1, m.ammo);
				if (key == "ammoPerShot") return Int(v, m.ammoPerShot);
				if (key == "speed") return Num(v, m.speed);
				if (key == "gravity") return Num(v, m.gravity);
				if (key == "range") return Num(v, m.range);
				if (key == "splash") return Num(v, m.splash);
				if (key == "chargeFull") return Num(v, m.chargeFull);
				if (key == "chargeMax") return Num(v, m.chargeMax);
				if (key == "quirks") return Quirks(v, m.quirks);
				return Fail("unknown key");
			});
		}

		WeaponSpec *FindWeapon(const std::string &classname)
		{
			for (WeaponSpec &w : cfg.weapons)
				if (w.classname && classname == w.classname)
					return &w;
			return nullptr;
		}

		bool Weapon(const std::string &classname, int obj)
		{
			WeaponSpec *old = FindWeapon(classname);
			WeaponSpec spec = old ? *old : WeaponSpec{};
			if (!old)
				spec.classname = Keep(classname);

			bool ok = Members(obj, [&](const std::string &key, int v) {
				if (key == "id") return Int(v, spec.id);
				if (key == "clipSize") return Int(v, spec.clipSize);
				if (key == "value") return Num(v, spec.value);
				if (key == "rangeClass") return Enum(v, WeaponRangeName, RANGE_LONG + 1, spec.rangeClass);
				if (key == "zoomFov") return Num(v, spec.zoomFov);
				if (key == "primary") return Mode(v, spec.primary);
				if (key == "primaryZoomed") return Mode(v, spec.primaryZoomed);
				if (key == "secondary") return Mode(v, spec.secondary);
				if (key == "setup") return Str(v, spec.setup);
				if (key == "ammoEntity")
				{
					return Members(v, [&](const std::string &pool, int e) {
						WeaponAmmo a = AMMO_NONE;
						if (!Enum(e - 1, WeaponAmmoName, AMMO_SECONDARY + 1, a) || a == AMMO_NONE)
							return Fail("unknown ammo pool");
						return Str(e, spec.ammoEntity[a]);
					});
				}
				return Fail("unknown key");
			});
			if (!ok)
				return false;

			if (spec.id <= 0 || spec.id > kMaxWeaponId)
				return Fail("needs an id between 1 and 255");

			if (old)
				*old = WeaponSpec{};
			if (cfg.weapons.size() <= static_cast<size_t>(spec.id))
				cfg.weapons.resize(static_cast<size_t>(spec.id) + 1);

			WeaponSpec &slot = cfg.weapons[static_cast<size_t>(spec.id)];
			if (slot.classname)
				return Fail(std::string("id ") + std::to_string(spec.id) + " is already " + slot.classname);
			slot = spec;
			return true;
		}

		bool Fall(int v)
		{
			if (doc.TypeOf(v) == Json::T_STRING)
			{
				std::string s = doc.Str(v);
				if (s == "hl")
					cfg.fall = PmSim::kHlFall;
				else if (s == "none")
					cfg.fall = PmSim::kNoFall;
				else
					return Fail("expected \"hl\", \"none\" or an object");
				return true;
			}
			cfg.fall = PmSim::FallRule{};
			return Members(v, [&](const std::string &key, int n) {
				if (key == "safeSpeed") return Num(n, cfg.fall.safeSpeed);
				if (key == "perSpeed") return Num(n, cfg.fall.perSpeed);
				if (key == "health") return Num(n, cfg.fall.health);
				return Fail("unknown key");
			});
		}

		bool Throw(int v)
		{
			return Members(v, [&](const std::string &key, int n) {
				if (key == "style")
				{
					static const char *const kStyles[] = { "hl", "quake" };
					int style = 0;
					if (!Pick(n, kStyles, 2, style))
						return false;
					cfg.throwStyle = static_cast<ThrowStyle>(style);
					return true;
				}
				if (key == "rate") return Num(n, cfg.throwRate);
				if (key == "speedMax") return Num(n, cfg.throwSpeedMax);
				return Fail("unknown key");
			});
		}

		bool Radio(int v)
		{
			if (!Array(v))
				return false;
			cfg.radio.clear();
			std::string outer = where;
			for (int i = 0; i < doc.Count(v); ++i)
			{
				where = outer + "[" + std::to_string(i) + "]";
				RadioInfo info = {};
				bool ok = Members(doc.Element(v, i), [&](const std::string &key, int n) {
					if (key == "command") return Str(n, info.command);
					if (key == "code") return Int(n, info.code);
					if (key == "sound") return Str(n, info.sound);
					if (key == "text") return Str(n, info.text);
					return Fail("unknown key");
				});
				if (!ok)
					return false;
				if (!info.command || !info.text)
					return Fail("needs command and text");
				cfg.radio.push_back(info);
			}
			where = outer;
			return true;
		}

		bool Root(int root)
		{
			return Members(root, [&](const std::string &key, int v) {
				if (key == "base") return true;
				if (key == "profile")
				{
					const char *s = nullptr;
					if (!Str(v, s) || !s)
						return Fail("expected a string");
					cfg.profile = s;
					return true;
				}
				if (key == "name") return Str(v, cfg.name);
				if (key == "deathmatchOnly") return Bool(v, cfg.deathmatchOnly);
				if (key == "itemSlots") return Int(v, cfg.itemSlots);
				if (key == "fullTimeout") return Num(v, cfg.fullTimeout);
				if (key == "unreachableTimeout") return Num(v, cfg.unreachableTimeout);
				if (key == "braceDuck") return Bool(v, cfg.braceDuck);
				if (key == "fall") return Fall(v);
				if (key == "fallCvar") return Str(v, cfg.fallCvar);
				if (key == "throw") return Throw(v);
				if (key == "radio") return Radio(v);
				if (key == "respawnPress")
				{
					static const char *const kPress[] = { "attack", "idle" };
					int press = 0;
					if (!Pick(v, kPress, 2, press))
						return false;
					cfg.respawnPress = static_cast<RespawnPress>(press);
					return true;
				}
				if (key == "absoluteTimers") return Bool(v, cfg.absoluteTimers);
				if (key == "melee")
				{
					if (!Array(v))
						return false;
					cfg.melee.clear();
					for (int i = 0; i < doc.Count(v); ++i)
					{
						int id = 0;
						if (!Int(doc.Element(v, i), id))
							return false;
						cfg.melee.push_back(id);
					}
					return true;
				}
				if (key == "preferred")
				{
					if (!Array(v))
						return false;
					cfg.preferred.clear();
					for (int i = 0; i < doc.Count(v); ++i)
					{
						const char *s = nullptr;
						if (!Str(doc.Element(v, i), s) || !s)
							return Fail("expected a string");
						cfg.preferred.push_back(s);
					}
					return true;
				}
				if (key == "ambush")
				{
					return Members(v, [&](const std::string &, int n) {
						int kind = 0;
						return Pick(n - 1, kAmbushNames, AMBUSH_COUNT, kind) && Int(n, cfg.ambush[kind]);
					});
				}
				if (key == "deploy")
				{
					return Members(v, [&](const std::string &, int n) {
						int kind = 0;
						if (!Pick(n - 1, kDeployNames, DEPLOY_COUNT, kind))
							return false;
						return Members(n, [&](const std::string &field, int f) {
							if (field == "weapon") return Int(f, cfg.deployWeapon[kind]);
							if (field == "entity") return Str(f, cfg.deployEntity[kind]);
							return Fail("unknown key");
						});
					});
				}
				if (key == "respawn")
				{
					return Members(v, [&](const std::string &, int n) {
						int kind = 0;
						return Pick(n - 1, kItemKindNames, kItemKindCount, kind) && Num(n, cfg.respawn[kind]);
					});
				}
				if (key == "weapons")
					return Members(v, [&](const std::string &classname, int n) { return Weapon(classname, n); });
				return Fail("unknown key");
			});
		}
	};

	bool ReadFile(const char *path, std::string &out)
	{
		FILE *f = fopen(path, "rb");
		if (!f)
			return false;
		char buf[4096];
		size_t n = 0;
		while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
			out.append(buf, n);
		fclose(f);
		return true;
	}

	bool LoadFile(const char *gamedir, GameConfig &cfg, std::string &why, int depth)
	{
		char path[512];
		GameConfig_Path(gamedir, path, sizeof(path));

		std::string where = std::string("games/") + gamedir + ".json";
		std::string text;
		if (!ReadFile(path, text))
		{
			why = where + ": cannot read " + path;
			return false;
		}

		Json::Doc doc;
		if (!doc.Parse(text) || doc.TypeOf(doc.Root()) != Json::T_OBJECT)
		{
			why = where + ": not a JSON object";
			return false;
		}

		int base = doc.Member(doc.Root(), "base");
		if (base >= 0)
		{
			if (depth >= kMaxBaseDepth)
			{
				why = where + ": base chain deeper than 4";
				return false;
			}
			if (!LoadFile(doc.Str(base).c_str(), cfg, why, depth + 1))
				return false;
		}

		Loader loader{ doc, cfg, why, where };
		return loader.Root(doc.Root());
	}
}

void GameConfig_Path(const char *gamedir, char *out, size_t size)
{
	char hyperdir[256] = { 0 };
	g_engfuncs.pfnGetGameDir(hyperdir);
	snprintf(out, size, "%s/addons/crossbot/games/%s.json", hyperdir, gamedir);
}

bool GameConfig_Load(const char *gamedir, GameConfig &out, std::string &why)
{
	if (!LoadFile(gamedir, out, why, 0))
		return false;

	if (out.profile.empty())
	{
		why = std::string("games/") + gamedir + ".json: no profile";
		return false;
	}

	for (const WeaponSpec &w : out.weapons)
		if (w.setup)
			out.weaponSetup = true;

	return true;
}
