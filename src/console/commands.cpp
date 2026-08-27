#include <extdll.h>
#include <meta_api.h>

#include "commands.h"
#include "../bot/bot.h"
#include "../bot/bot_aim.h"
#include "../bot/bot_fire.h"
#include "../bot/bot_items.h"
#include "../bot/bot_manager.h"
#include "../bot/bot_weapons.h"
#include "../game/ent_handle.h"
#include "../game/game_profile.h"
#include "../game/give_item.h"
#include "../game/hldm_weapons.h"
#include "../nav/nav.h"

#if defined(_WIN32)
#define CB_STRICMP _stricmp
#else
#define CB_STRICMP strcasecmp
#endif

namespace
{
	constexpr unsigned int kSuitBit = 1u << 31;

	const char *PlayerNameOf(edict_t *ent) { return CBotPlayerNameOf(ent); }

	bool IsLivePlayer(edict_t *ent) { return CBotIsLivePlayer(ent); }

	edict_t *FindPlayerByName(const char *name)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!IsLivePlayer(ent))
				continue;
			if (CB_STRICMP(PlayerNameOf(ent), name) == 0)
				return ent;
		}
		return nullptr;
	}

	void PrintInventory(edict_t *player, const BotInventory &inv)
	{
		char msg[320];
		snprintf(msg, sizeof(msg), "CrossBot: %s carries %d weapon(s), nextAttack %.2f\n",
			PlayerNameOf(player), inv.count, inv.nextAttack);
		SERVER_PRINT(msg);

		unsigned int walked = 0;
		for (int i = 0; i < inv.count; ++i)
		{
			const BotWeapon &w = inv.weapons[i];
			walked |= 1u << w.id;

			int primary = (w.primaryAmmoType >= 0 && w.primaryAmmoType < MAX_AMMO_SLOTS)
				? inv.ammo[w.primaryAmmoType] : -1;
			int secondary = (w.secondaryAmmoType >= 0 && w.secondaryAmmoType < MAX_AMMO_SLOTS)
				? inv.ammo[w.secondaryAmmoType] : -1;

			snprintf(msg, sizeof(msg),
				"CrossBot:  %c %-20s id %2d clip %3d ammo1 %d:%-4d ammo2 %d:%-4d next %.2f/%.2f%s\n",
				i == inv.activeIndex ? '*' : ' ', CBotClassOf(w.ent), w.id, w.clip,
				w.primaryAmmoType, primary, w.secondaryAmmoType, secondary,
				w.nextPrimaryAttack, w.nextSecondaryAttack, w.reloading ? " reloading" : "");
			SERVER_PRINT(msg);
		}

		for (int i = 0; i < MAX_AMMO_SLOTS; ++i)
		{
			if (inv.ammo[i] == 0)
				continue;
			snprintf(msg, sizeof(msg), "CrossBot:  ammo[%d] = %d\n", i, inv.ammo[i]);
			SERVER_PRINT(msg);
		}

		unsigned int pevBits = static_cast<unsigned int>(player->v.weapons);
		snprintf(msg, sizeof(msg), "CrossBot:  walked mask 0x%08x, pev->weapons 0x%08x -- %s\n",
			walked, pevBits, walked == (pevBits & ~kSuitBit) ? "match" : "MISMATCH");
		SERVER_PRINT(msg);
	}

	void Cmd_CrossBotWeapons()
	{
		if (CMD_ARGC() > 1)
		{
			edict_t *ent = FindPlayerByName(CMD_ARGV(1));
			if (!ent)
			{
				char msg[128];
				snprintf(msg, sizeof(msg), "CrossBot: no player named '%s'\n", CMD_ARGV(1));
				SERVER_PRINT(msg);
				return;
			}

			if (CBot *bot = BotManager::FindBot(ent))
			{
				PrintInventory(ent, bot->inventory);
			}
			else
			{
				BotInventory inv;
				BotWeapons::Read(ent, inv);
				PrintInventory(ent, inv);
			}
			return;
		}

		int shown = 0;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!IsLivePlayer(ent))
				continue;

			CBot *bot = BotManager::FindBot(ent);
			if (!bot)
				continue;

			PrintInventory(ent, bot->inventory);
			++shown;
		}

		if (shown == 0)
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	bool EquipPlayer(edict_t *player, const char *classname)
	{
		char msg[192];

		if (!GiveItem::Give(player, classname))
		{
			snprintf(msg, sizeof(msg), "CrossBot: the engine would not create '%s'\n", classname);
			SERVER_PRINT(msg);
			return false;
		}

		snprintf(msg, sizeof(msg), "CrossBot: gave %s to %s\n", classname, PlayerNameOf(player));
		SERVER_PRINT(msg);
		return true;
	}

	void Cmd_CrossBotAimTest()
	{
		if (CMD_ARGC() < 3)
		{
			SERVER_PRINT("CrossBot: usage: crossbot_aim_test <bot> <yawDelta> [pitchDelta] [fov]\n");
			return;
		}

		edict_t *ent = FindPlayerByName(CMD_ARGV(1));
		CBot *bot = ent ? BotManager::FindBot(ent) : nullptr;
		if (!bot)
		{
			char msg[128];
			snprintf(msg, sizeof(msg), "CrossBot: no bot named '%s'\n", CMD_ARGV(1));
			SERVER_PRINT(msg);
			return;
		}

		float yawDelta = static_cast<float>(atof(CMD_ARGV(2)));
		float pitchDelta = (CMD_ARGC() > 3) ? static_cast<float>(atof(CMD_ARGV(3))) : 0.f;

		if (CMD_ARGC() > 4)
			ent->v.fov = static_cast<float>(atof(CMD_ARGV(4)));

		BotAim::BeginTest(*bot, yawDelta, pitchDelta);

		char msg[192];
		snprintf(msg, sizeof(msg),
			"CrossBot: %s aim test yaw %+.1f pitch %+.1f from (%.1f, %.1f), fov %.0f\n",
			bot->name, yawDelta, pitchDelta, bot->pEdict->v.v_angle.x, bot->pEdict->v.v_angle.y,
			ent->v.fov > 0.f ? ent->v.fov : 90.f);
		SERVER_PRINT(msg);
	}

	void Cmd_CrossBotAimStatus()
	{
		BotManager::PrintAimStatus();
	}

	void Cmd_CrossBotSenseStatus()
	{
		BotManager::PrintSenseStatus();
	}

	void Cmd_CrossBotVantageStatus()
	{
		if (CMD_ARGC() < 4)
		{
			BotManager::PrintVantageStatus(nullptr);
			return;
		}

		Vector at(static_cast<float>(atof(CMD_ARGV(1))), static_cast<float>(atof(CMD_ARGV(2))),
			static_cast<float>(atof(CMD_ARGV(3))));
		BotManager::PrintVantageStatus(&at);
	}

	void Cmd_CrossBotFireStatus()
	{
		BotManager::PrintFireStatus();
	}

	void Cmd_CrossBotWeaponUsage()
	{
		BotManager::PrintWeaponUsage();
	}

	void Cmd_CrossBotFirePin()
	{
		if (CMD_ARGC() < 2)
		{
			const char *pin = BotFire::Pinned();
			char msg[128];
			snprintf(msg, sizeof(msg), "CrossBot: fire pin is %s %s %s\n", pin ? pin : "off",
				BotFire::PinnedMode(), BotFire::PinnedZoom());
			SERVER_PRINT(msg);
			SERVER_PRINT("usage: crossbot_fire_pin <classname|off> [primary|secondary] [zoom|nozoom]\n");
			return;
		}

		const char *arg = CMD_ARGV(1);
		bool off = strcmp(arg, "off") == 0;

		int mode = 0;
		if (CMD_ARGC() >= 3)
		{
			const char *want = CMD_ARGV(2);
			if (strcmp(want, "primary") == 0)
				mode = 1;
			else if (strcmp(want, "secondary") == 0)
				mode = 2;
			else
			{
				SERVER_PRINT("usage: crossbot_fire_pin <classname|off> [primary|secondary] [zoom|nozoom]\n");
				return;
			}
		}

		int zoom = -1;
		if (CMD_ARGC() >= 4)
		{
			const char *want = CMD_ARGV(3);
			if (strcmp(want, "zoom") == 0)
				zoom = 1;
			else if (strcmp(want, "nozoom") == 0)
				zoom = 0;
			else if (strcmp(want, "any") != 0)
			{
				SERVER_PRINT("usage: crossbot_fire_pin <classname|off> [primary|secondary] [zoom|nozoom]\n");
				return;
			}
		}

		BotFire::Pin(off ? nullptr : arg, mode, zoom);

		char msg[128];
		snprintf(msg, sizeof(msg), "CrossBot: fire pin is %s %s %s\n", off ? "off" : arg,
			BotFire::PinnedMode(), BotFire::PinnedZoom());
		SERVER_PRINT(msg);
	}

	void AppendQuirks(unsigned int quirks, char *out, size_t cap)
	{
		static const struct { unsigned int bit; const char *name; } kNames[] = {
			{ WPN_Q_HIDDEN_STATE,  "hiddenstate" },
			{ WPN_Q_NO_UNDERWATER, "nounderwater" },
			{ WPN_Q_SLOW_IN_WATER, "slowinwater" },
			{ WPN_Q_THROW_ARC,     "throwarc" },
			{ WPN_Q_INHERIT_VEL,   "inheritvel" },
			{ WPN_Q_KEEP_AIMING,   "keepaiming" },
			{ WPN_Q_HOMING,        "homing" },
		};

		out[0] = '\0';
		size_t used = 0;

		for (size_t i = 0; i < sizeof(kNames) / sizeof(kNames[0]); ++i)
		{
			if ((quirks & kNames[i].bit) == 0)
				continue;

			int n = snprintf(out + used, cap - used, "%s%s", used ? "," : "", kNames[i].name);
			if (n < 0 || static_cast<size_t>(n) >= cap - used)
				return;
			used += static_cast<size_t>(n);
		}

		if (used == 0)
			snprintf(out, cap, "-");
	}

	void PrintWeaponMode(const char *label, const WeaponMode &mode)
	{
		if (mode.fire == FIRE_NONE && mode.toggle == TOGGLE_NONE)
			return;

		char quirks[128];
		AppendQuirks(mode.quirks, quirks, sizeof(quirks));

		char msg[320];
		snprintf(msg, sizeof(msg),
			"CrossBot:    %-2s %-6s %-7s toggle %-5s ammo %-4s x%d speed %-6.0f grav %.2f range %-6.0f splash %-5.0f charge %.2f/%.2f %s\n",
			label, WeaponFireName(mode.fire), WeaponAimName(mode.aim),
			WeaponToggleName(mode.toggle), WeaponAmmoName(mode.ammo), mode.ammoPerShot,
			mode.speed, mode.gravity, mode.range, mode.splash,
			mode.chargeFull, mode.chargeMax, quirks);
		SERVER_PRINT(msg);
	}

	void PrintWeaponSpec(const WeaponSpec &spec)
	{
		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot:  id %2d %-20s clip %d\n",
			spec.id, spec.classname, spec.clipSize);
		SERVER_PRINT(msg);

		PrintWeaponMode("P", spec.primary);
		PrintWeaponMode("Pz", spec.primaryZoomed);
		PrintWeaponMode("S", spec.secondary);
	}

	void Cmd_CrossBotWeaponSpec()
	{
		if (!g_pGameProfile)
		{
			SERVER_PRINT("CrossBot: no game profile detected.\n");
			return;
		}

		if (CMD_ARGC() > 1)
		{
			edict_t *ent = FindPlayerByName(CMD_ARGV(1));
			if (!ent)
			{
				char msg[128];
				snprintf(msg, sizeof(msg), "CrossBot: no player named '%s'\n", CMD_ARGV(1));
				SERVER_PRINT(msg);
				return;
			}

			BotInventory inv;
			if (CBot *bot = BotManager::FindBot(ent))
				inv = bot->inventory;
			else
				BotWeapons::Read(ent, inv);

			char msg[192];
			snprintf(msg, sizeof(msg), "CrossBot: %s carries %d weapon(s), fov %.0f\n",
				PlayerNameOf(ent), inv.count, ent->v.fov);
			SERVER_PRINT(msg);

			for (int i = 0; i < inv.count; ++i)
			{
				const BotWeapon &w = inv.weapons[i];
				const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);

				if (!spec)
				{
					snprintf(msg, sizeof(msg), "CrossBot:  %c id %2d %-20s NO SPEC\n",
						i == inv.activeIndex ? '*' : ' ', w.id, CBotClassOf(w.ent));
					SERVER_PRINT(msg);
					continue;
				}

				const char *live = CBotClassOf(w.ent);
				bool match = strcmp(live, spec->classname) == 0;

				snprintf(msg, sizeof(msg), "CrossBot:  %c live %-20s spec %-20s %s\n",
					i == inv.activeIndex ? '*' : ' ', live, spec->classname,
					match ? "match" : "MISMATCH");
				SERVER_PRINT(msg);

				PrintWeaponSpec(*spec);
			}
			return;
		}

		int shown = 0;
		int broken = 0;

		for (int slot = 0; slot < kHLDMWeaponSlots; ++slot)
		{
			const WeaponSpec *raw = HLDM_WeaponSlot(slot);
			if (!raw || !raw->classname)
				continue;

			if (raw->id != slot)
			{
				char msg[192];
				snprintf(msg, sizeof(msg),
					"CrossBot: TABLE BROKEN -- slot %d holds id %d (%s)\n",
					slot, raw->id, raw->classname);
				SERVER_PRINT(msg);
				++broken;
				continue;
			}

			PrintWeaponSpec(*raw);
			++shown;
		}

		char msg[192];
		snprintf(msg, sizeof(msg), "CrossBot: %d weapon spec(s), %d broken slot(s)\n", shown, broken);
		SERVER_PRINT(msg);
	}

	void Cmd_CrossBotEquip()
	{
		if (CMD_ARGC() < 2)
		{
			SERVER_PRINT("usage: crossbot_equip <classname> [name]\n");
			return;
		}

		const char *classname = CMD_ARGV(1);

		if (CMD_ARGC() > 2)
		{
			edict_t *ent = FindPlayerByName(CMD_ARGV(2));
			if (!ent)
			{
				char msg[128];
				snprintf(msg, sizeof(msg), "CrossBot: no player named '%s'\n", CMD_ARGV(2));
				SERVER_PRINT(msg);
				return;
			}
			EquipPlayer(ent, classname);
			return;
		}

		int bots = 0;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!IsLivePlayer(ent) || !BotManager::FindBot(ent))
				continue;
			++bots;
			if (!EquipPlayer(ent, classname))
				return;
		}

		if (bots == 0)
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void Cmd_CrossBotAdd()
	{
		const char *name = CMD_ARGC() > 1 ? CMD_ARGV(1) : nullptr;
		BotManager::AddBot(name);
	}

	void Cmd_CrossBotNavGoal()
	{
		if (CMD_ARGC() < 4)
		{
			SERVER_PRINT("usage: crossbot_nav_goal <x> <y> <z>\n");
			return;
		}

		Vector point(static_cast<float>(atof(CMD_ARGV(1))), static_cast<float>(atof(CMD_ARGV(2))),
			static_cast<float>(atof(CMD_ARGV(3))));

		if (!BotManager::SetGoalForAllBots(point))
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void Cmd_CrossBotTeleport()
	{
		if (CMD_ARGC() < 4)
		{
			SERVER_PRINT("usage: crossbot_teleport <x> <y> <z>\n");
			return;
		}

		Vector point(static_cast<float>(atof(CMD_ARGV(1))), static_cast<float>(atof(CMD_ARGV(2))),
			static_cast<float>(atof(CMD_ARGV(3))));

		if (!BotManager::TeleportAllBots(point))
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void Cmd_CrossBotKill()
	{
		if (!BotManager::KillAllBots())
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void Cmd_CrossBotNavStatus()
	{
		BotManager::PrintNavStatus();
	}

	void Cmd_CrossBotNavWaypoints()
	{
		BotManager::PrintWaypoints();
	}

	void Cmd_CrossBotListSpawns()
	{
		static const char *kSpawnClassnames[] = {"info_player_start", "info_player_deathmatch"};
		for (const char *classname : kSpawnClassnames)
		{
			edict_t *ent = nullptr;
			while (true)
			{
				ent = FIND_ENTITY_BY_STRING(ent, "classname", classname);
				if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
					break;

				char msg[128];
				snprintf(msg, sizeof(msg), "CrossBot: spawn %s (%.1f,%.1f,%.1f)\n", classname,
					ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
				SERVER_PRINT(msg);
			}
		}
	}

	void Cmd_CrossBotListTargets()
	{
		edict_t *ent = nullptr;
		while (true)
		{
			ent = FIND_ENTITY_BY_STRING(ent, "classname", "info_target");
			if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
				break;

			char msg[128];
			snprintf(msg, sizeof(msg), "CrossBot: target info_target (%.1f,%.1f,%.1f)\n",
				ent->v.origin.x, ent->v.origin.y, ent->v.origin.z);
			SERVER_PRINT(msg);
		}
	}

	void Cmd_CrossBotListItems()
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
		{
			SERVER_PRINT("CrossBot: no nav mesh loaded\n");
			return;
		}

		BotItems::PrintCandidates(*mesh);
	}

	void Cmd_CrossBotNavPenalties()
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
		{
			SERVER_PRINT("CrossBot: no nav mesh loaded\n");
			return;
		}

		size_t shown = 0;
		for (size_t i = 0; i < mesh->conns.size(); ++i)
		{
			const Nav::Connection &c = mesh->conns[i];
			if (c.runtimePenalty <= 0.f)
				continue;

			char msg[160];
			snprintf(msg, sizeof(msg),
				"CrossBot: conn %zu %u->%u type %u cost %.2fs penalty %.2fs\n", i, c.areaA, c.areaB,
				static_cast<unsigned>(c.connType), c.costSeconds, c.runtimePenalty);
			SERVER_PRINT(msg);
			++shown;
		}

		char summary[96];
		snprintf(summary, sizeof(summary), "CrossBot: %zu/%zu connections penalized\n", shown,
			mesh->conns.size());
		SERVER_PRINT(summary);
	}
}

namespace CrossBotCommands
{

void Register()
{
	ADD_SERVER_COMMAND("crossbot_add", Cmd_CrossBotAdd);
	ADD_SERVER_COMMAND("crossbot_nav_goal", Cmd_CrossBotNavGoal);
	ADD_SERVER_COMMAND("crossbot_teleport", Cmd_CrossBotTeleport);
	ADD_SERVER_COMMAND("crossbot_kill", Cmd_CrossBotKill);
	ADD_SERVER_COMMAND("crossbot_nav_status", Cmd_CrossBotNavStatus);
	ADD_SERVER_COMMAND("crossbot_list_spawns", Cmd_CrossBotListSpawns);
	ADD_SERVER_COMMAND("crossbot_list_targets", Cmd_CrossBotListTargets);
	ADD_SERVER_COMMAND("crossbot_waypoints", Cmd_CrossBotNavWaypoints);
	ADD_SERVER_COMMAND("crossbot_nav_penalties", Cmd_CrossBotNavPenalties);
	ADD_SERVER_COMMAND("crossbot_list_items", Cmd_CrossBotListItems);
	ADD_SERVER_COMMAND("crossbot_weapons", Cmd_CrossBotWeapons);
	ADD_SERVER_COMMAND("crossbot_weaponspec", Cmd_CrossBotWeaponSpec);
	ADD_SERVER_COMMAND("crossbot_equip", Cmd_CrossBotEquip);
	ADD_SERVER_COMMAND("crossbot_aim_test", Cmd_CrossBotAimTest);
	ADD_SERVER_COMMAND("crossbot_aim_status", Cmd_CrossBotAimStatus);
	ADD_SERVER_COMMAND("crossbot_sense_status", Cmd_CrossBotSenseStatus);
	ADD_SERVER_COMMAND("crossbot_vantage_status", Cmd_CrossBotVantageStatus);
	ADD_SERVER_COMMAND("crossbot_fire_status", Cmd_CrossBotFireStatus);
	ADD_SERVER_COMMAND("crossbot_fire_pin", Cmd_CrossBotFirePin);
	ADD_SERVER_COMMAND("crossbot_weapon_usage", Cmd_CrossBotWeaponUsage);
}

}
