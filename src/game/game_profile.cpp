#include <extdll.h>
#include <meta_api.h>

#include <map>
#include <memory>
#include <string>

#include "game_profile.h"
#include "cstrike/cs_profile.h"
#include "dmc/dmc_profile.h"
#include "dod/dod_profile.h"
#include "hldm/hldm_profile.h"
#include "ricochet/ricochet_profile.h"
#include "tfc/tfc_profile.h"
#include "ts/ts_profile.h"

namespace
{
	std::unique_ptr<CHLDMProfile> MakeProfile(const std::string &kind)
	{
		if (kind == "hldm")
			return std::make_unique<CHLDMProfile>();
		if (kind == "ricochet")
			return std::make_unique<CRicochetProfile>();
		if (kind == "dmc")
			return std::make_unique<CDMCProfile>();
		if (kind == "tfc")
			return std::make_unique<CTFCProfile>();
		if (kind == "cs")
			return std::make_unique<CCSProfile>();
		if (kind == "dod")
			return std::make_unique<CDODProfile>();
		if (kind == "ts")
			return std::make_unique<CTSProfile>();
		return nullptr;
	}

	void Complain(const char *gamedir, const std::string &why)
	{
		char msg[512];
		snprintf(msg, sizeof(msg), "CrossBot: game '%s': %s\n", gamedir, why.c_str());
		SERVER_PRINT(msg);
	}
}

IGameProfile *GameProfile_Detect(const char *gamedir)
{
	static std::map<std::string, std::unique_ptr<CHLDMProfile>> profiles;

	if (!gamedir || !gamedir[0])
		return nullptr;

	std::string key(gamedir);
	for (char &c : key)
		c = static_cast<char>(tolower(static_cast<unsigned char>(c)));

	auto found = profiles.find(key);
	CHLDMProfile *profile = (found != profiles.end()) ? found->second.get() : nullptr;

	if (!profile)
	{
		GameConfig config;
		std::string why;
		if (!GameConfig_Load(key.c_str(), config, why))
		{
			Complain(gamedir, why);
			return nullptr;
		}

		std::unique_ptr<CHLDMProfile> made = MakeProfile(config.profile);
		if (!made)
		{
			Complain(gamedir, "unknown profile '" + config.profile + "'");
			return nullptr;
		}

		made->config = std::move(config);
		profile = made.get();
		profiles[key] = std::move(made);
	}

	if (profile->config.deathmatchOnly && !gpGlobals->deathmatch_)
	{
		Complain(gamedir, "needs deathmatch 1");
		return nullptr;
	}

	return profile;
}
