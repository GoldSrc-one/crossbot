#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>

#include "commands.h"
#include "../bot/bot.h"
#include "../bot/bot_aim.h"
#include "../bot/bot_deploy.h"
#include "../bot/bot_difficulty.h"
#include "../bot/bot_fire.h"
#include "../bot/bot_fun.h"
#include "../bot/bot_items.h"
#include "../bot/bot_manager.h"
#include "../bot/bot_move_nav.h"
#include "../bot/bot_strategic.h"
#include "../bot/bot_weapons.h"
#include "../game/crossplay.h"
#include "../game/ent_handle.h"
#include "../game/game_profile.h"
#include "../game/gamedata.h"
#include "../game/give_item.h"
#include "../game/hldm_weapons.h"
#include "../game/objectives.h"
#include "../nav/bsp_trace.h"
#include "../nav/nav.h"
#include "../nav/nav_water.h"

namespace
{
	constexpr unsigned int kSuitBit = 1u << 31;

	using XPlay::ScopedGame;

	edict_t *FindPlayerByName(const char *name)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!CBotIsLivePlayer(ent))
				continue;
			if (CB_STRICMP(CBotPlayerNameOf(ent), name) == 0)
				return ent;
		}
		return nullptr;
	}

	void ComplainNoName(const char *kind, const char *name)
	{
		char msg[128];
		snprintf(msg, sizeof(msg), "CrossBot: no %s named '%s'\n", kind, name);
		SERVER_PRINT(msg);
	}

	edict_t *FindPlayerOrComplain(const char *name)
	{
		edict_t *ent = FindPlayerByName(name);
		if (!ent)
			ComplainNoName("player", name);
		return ent;
	}

	CBot *FindBotOrComplain(const char *name, edict_t *&ent)
	{
		ent = FindPlayerByName(name);
		CBot *bot = ent ? BotManager::FindBot(ent) : nullptr;
		if (!bot)
			ComplainNoName("bot", name);
		return bot;
	}

	template <typename ActFn>
	void ForEachBot(ActFn act)
	{
		int bots = 0;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!CBotIsLivePlayer(ent) || !BotManager::FindBot(ent))
				continue;
			++bots;
			if (!act(ent))
				return;
		}

		if (bots == 0)
			SERVER_PRINT("CrossBot: no bots connected.\n");
	}

	void PrintInventory(edict_t *player, const BotInventory &inv)
	{
		char msg[320];
		snprintf(msg, sizeof(msg), "CrossBot: %s carries %d weapon(s), nextAttack %.2f\n",
			CBotPlayerNameOf(player), inv.count, inv.nextAttack);
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

		if (g_offsets.plr_iPowerups >= 0 && CBotPrivateData(player))
		{
			snprintf(msg, sizeof(msg), "CrossBot:  powerups 0x%x, +4 = %d\n",
				GameData::Peek<int>(CBotPrivateData(player), g_offsets.plr_iPowerups),
				GameData::Peek<int>(CBotPrivateData(player), g_offsets.plr_iPowerups + 4));
			SERVER_PRINT(msg);
		}

		int walkedActive = (inv.activeIndex >= 0 && inv.activeIndex < inv.count)
			? inv.weapons[inv.activeIndex].id : 0;
		int peeked = BotWeapons::ActiveWeaponId(player);
		snprintf(msg, sizeof(msg), "CrossBot:  walked active id %d, peeked active id %d -- %s\n",
			walkedActive, peeked, walkedActive == peeked ? "match" : "MISMATCH");
		SERVER_PRINT(msg);
	}

	void Cmd_CrossBotWeapons()
	{
		if (CMD_ARGC() > 1)
		{
			edict_t *ent = FindPlayerOrComplain(CMD_ARGV(1));
			if (!ent)
				return;

			ScopedGame ctx(ent);
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
			if (!CBotIsLivePlayer(ent))
				continue;

			CBot *bot = BotManager::FindBot(ent);
			if (!bot)
				continue;

			ScopedGame ctx(bot->gameDll);
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

		snprintf(msg, sizeof(msg), "CrossBot: gave %s to %s\n", classname, CBotPlayerNameOf(player));
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

		edict_t *ent = nullptr;
		CBot *bot = FindBotOrComplain(CMD_ARGV(1), ent);
		if (!bot)
			return;

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

	void Cmd_CrossBotMonsters()
	{
		BotManager::PrintMonsters();
	}

	void Cmd_CrossBotDifficultyStatus()
	{
		BotDiff::PrintStatus();
	}

	const Vector *ParsePointOrNull(Vector &storage)
	{
		if (CMD_ARGC() < 4)
			return nullptr;

		storage = Vector(static_cast<float>(atof(CMD_ARGV(1))), static_cast<float>(atof(CMD_ARGV(2))),
			static_cast<float>(atof(CMD_ARGV(3))));
		return &storage;
	}

	void Cmd_CrossBotVantageStatus()
	{
		Vector at;
		BotManager::PrintVantageStatus(ParsePointOrNull(at));
	}

	void Cmd_CrossBotCampStatus()
	{
		Vector at;
		BotManager::PrintCampStatus(ParsePointOrNull(at));
	}

	void Cmd_CrossBotTeamsStatus()
	{
		BotManager::PrintTeamsStatus();
	}

	void Cmd_CrossBotCombatStatus()
	{
		BotManager::PrintCombatStatus();
	}

	void Cmd_CrossBotUserMsgs()
	{
		BotDaze::PrintUserMsgs();
	}

	void Cmd_CrossBotDazeStatus()
	{
		ForEachBot([](edict_t *ent) { BotDaze::PrintStatus(*BotManager::FindBot(ent)); return true; });
	}

	void Cmd_CrossBotDazeTest()
	{
		if (CMD_ARGC() < 4)
		{
			SERVER_PRINT("usage: crossbot_daze_test <bot|*> flash <alpha> [duration] [hold]\n"
				"       crossbot_daze_test <bot|*> conc <scale>\n");
			return;
		}

		bool flash = strcmp(CMD_ARGV(2), "flash") == 0;
		if (!flash && strcmp(CMD_ARGV(2), "conc") != 0)
		{
			SERVER_PRINT("CrossBot: crossbot_daze_test kind must be flash or conc\n");
			return;
		}

		int value = atoi(CMD_ARGV(3));
		float duration = (CMD_ARGC() > 4) ? static_cast<float>(atof(CMD_ARGV(4))) : 12.f;
		float hold = (CMD_ARGC() > 5) ? static_cast<float>(atof(CMD_ARGV(5))) : 2.67f;

		auto act = [=](edict_t *ent) {
			CBot *bot = BotManager::FindBot(ent);
			if (flash)
				BotDaze::OnScreenFade(*bot, static_cast<int>(duration * 4096.f),
					static_cast<int>(hold * 4096.f), 0, 255, 255, 255, value);
			else
				BotDaze::OnConcuss(*bot, value);
			return true;
		};

		if (strcmp(CMD_ARGV(1), "*") == 0)
		{
			ForEachBot(act);
			return;
		}

		edict_t *ent = nullptr;
		if (FindBotOrComplain(CMD_ARGV(1), ent))
			act(ent);
	}

	void Cmd_CrossBotFireStatus()
	{
		BotManager::PrintFireStatus();
	}

	void Cmd_CrossBotWeaponCarry()
	{
		BotManager::PrintWeaponCarry();
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
			{ WPN_Q_STICKY,        "sticky" },
			{ WPN_Q_HOMING,        "homing" },
		};

		out[0] = '\0';
		size_t used = 0;

		for (size_t i = 0; i < ARRAYSIZE(kNames); ++i)
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
		snprintf(msg, sizeof(msg), "CrossBot:  id %2d %-20s clip %d value %.2f range %-5s zoomfov %.0f\n",
			spec.id, spec.classname, spec.clipSize, spec.value,
			WeaponRangeName(spec.rangeClass), spec.zoomFov);
		SERVER_PRINT(msg);

		PrintWeaponMode("P", spec.primary);
		PrintWeaponMode("Pz", spec.primaryZoomed);
		PrintWeaponMode("S", spec.secondary);
	}

	void Cmd_CrossBotWeaponSpec()
	{
		if (CMD_ARGC() > 1)
		{
			edict_t *ent = FindPlayerOrComplain(CMD_ARGV(1));
			if (!ent)
				return;

			ScopedGame ctx(ent);
			if (!g_pGameProfile)
			{
				char msg[160];
				snprintf(msg, sizeof(msg), "CrossBot: '%s' has no game profile.\n",
					XPlay::GameDir(XPlay::OwnerOf(ent)));
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
				CBotPlayerNameOf(ent), inv.count, ent->v.fov);
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
			edict_t *ent = FindPlayerOrComplain(CMD_ARGV(2));
			if (!ent)
				return;
			EquipPlayer(ent, classname);
			return;
		}

		ForEachBot([classname](edict_t *ent) { return EquipPlayer(ent, classname); });
	}

	bool SpawnNear(edict_t *player, const char *classname, float ahead)
	{
		Vector eye = player->v.origin + player->v.view_ofs;

		Vector forward, right, up;
		g_engfuncs.pfnAngleVectors(player->v.v_angle, forward, right, up);

		Vector want = eye + forward * ahead;

		TraceResult tr;
		TRACE_LINE(eye, want, ignore_monsters, player, &tr);

		edict_t *ent = GiveItem::Spawn(XPlay::OwnerOf(player), classname, tr.vecEndPos);

		char msg[192];
		if (!ent)
		{
			snprintf(msg, sizeof(msg), "CrossBot: could not spawn %s near %s\n", classname,
				CBotPlayerNameOf(player));
			SERVER_PRINT(msg);
			return false;
		}

		snprintf(msg, sizeof(msg), "CrossBot: spawned %s #%d at (%.0f,%.0f,%.0f) near %s\n",
			classname, g_engfuncs.pfnIndexOfEdict(ent), tr.vecEndPos.x, tr.vecEndPos.y,
			tr.vecEndPos.z, CBotPlayerNameOf(player));
		SERVER_PRINT(msg);
		return true;
	}

	void Cmd_CrossBotSpawn()
	{
		if (CMD_ARGC() < 2)
		{
			SERVER_PRINT("usage: crossbot_spawn <classname> [name] [distance]\n");
			return;
		}

		const char *classname = CMD_ARGV(1);
		float ahead = (CMD_ARGC() > 3) ? static_cast<float>(atof(CMD_ARGV(3))) : 96.f;

		if (CMD_ARGC() > 2)
		{
			edict_t *ent = FindPlayerOrComplain(CMD_ARGV(2));
			if (!ent)
				return;
			SpawnNear(ent, classname, ahead);
			return;
		}

		ForEachBot([classname, ahead](edict_t *ent) { return SpawnNear(ent, classname, ahead); });
	}

	void PrintLoadedGames()
	{
		char line[256];
		for (int i = 0; i < XPlay::NumGames(); ++i)
		{
			snprintf(line, sizeof(line), "  %s%s\n", XPlay::GameDir(i),
				XPlay::Supported(i) ? "" : " (not playable)");
			SERVER_PRINT(line);
		}
	}

	void Cmd_CrossBotAdd()
	{
		const char *dir = CMD_ARGC() > 1 ? CMD_ARGV(1) : nullptr;
		const char *name = CMD_ARGC() > 2 ? CMD_ARGV(2) : nullptr;
		const char *arg = CMD_ARGC() > 3 ? CMD_ARGV(3) : nullptr;

		if (arg && (!arg[0] || strcmp(arg, "*") == 0))
			arg = nullptr;

		int iGame;
		if (!dir || !dir[0] || strcmp(dir, "*") == 0)
		{
			iGame = XPlay::PickRandomSupported();
			if (iGame < 0)
			{
				SERVER_PRINT("CrossBot: no playable game is loaded.\n");
				PrintLoadedGames();
				return;
			}
		}
		else
		{
			iGame = XPlay::FindGame(dir);
			if (iGame < 0)
			{
				char msg[192];
				snprintf(msg, sizeof(msg),
					"usage: crossbot_add [gamedir|*] [name] [model|team] -- "
					"'%s' is not a loaded game. Loaded:\n", dir);
				SERVER_PRINT(msg);
				PrintLoadedGames();
				return;
			}
		}

		BotManager::AddBot(iGame, name, arg);
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

	int ListGame()
	{
		int owner = XPlay::MapOwner();
		if (XPlay::Supported(owner))
			return owner;

		for (int i = 0; i < XPlay::NumGames(); ++i)
		{
			if (XPlay::Supported(i))
				return i;
		}

		return -1;
	}

	const Nav::NavMesh *ListMesh(int &iGame)
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
		{
			SERVER_PRINT("CrossBot: no nav mesh loaded\n");
			return nullptr;
		}

		iGame = ListGame();
		if (iGame < 0)
		{
			SERVER_PRINT("CrossBot: no playable game is loaded.\n");
			PrintLoadedGames();
			return nullptr;
		}

		return mesh;
	}

	template <typename PrintFn>
	void ListCandidates(PrintFn print)
	{
		int iGame = -1;
		const Nav::NavMesh *mesh = ListMesh(iGame);
		if (!mesh)
			return;

		ScopedGame ctx(iGame);
		print(*mesh);
	}

	void Cmd_CrossBotListItems()
	{
		ListCandidates(BotItems::PrintCandidates);
	}

	void Cmd_CrossBotListFun()
	{
		ListCandidates(BotFun::PrintCandidates);
	}

	void Cmd_CrossBotListGoals()
	{
		ListCandidates(BotStrategic::PrintCandidates);
	}

	void Cmd_CrossBotListObjectives()
	{
		Objectives::PrintStatus();
		ForEachBot([](edict_t *ent) { BotStrategic::PrintIntents(*BotManager::FindBot(ent)); return true; });
	}

	void Cmd_CrossBotListGates()
	{
		ListCandidates(BotDeploy::PrintGates);
	}

#ifdef CROSSBOT_BSPTRACE
	void Cmd_CrossBotBspTrace()
	{
		int argc = g_engfuncs.pfnCmd_Argc();
		if (argc > 1 && strcmp(g_engfuncs.pfnCmd_Argv(1), "reset") == 0)
		{
			BspTrace::ResetStats();
			SERVER_PRINT("CrossBot: bsp trace stats reset\n");
			return;
		}
		BspTrace::PrintStats();
	}
#endif

	void Cmd_CrossBotAirmoveCache()
	{
		const NavPlan::AirmoveCache &cache = Bot_AirmoveCache();

		uint64_t hits = cache.Hits();
		uint64_t misses = cache.Misses();
		uint64_t queries = hits + misses;
		float rate = queries ? (100.f * static_cast<float>(hits) / static_cast<float>(queries)) : 0.f;

		char msg[288];
		snprintf(msg, sizeof(msg),
			"CrossBot: airmove cache %zu/%zu slots | %llu hit / %llu miss (%.1f%%) | "
			"%llu inserts / %llu evicted / %llu uncacheable | %llu verified / %llu MISMATCH (%llu ent)\n",
			cache.Occupancy(), cache.Capacity(),
			static_cast<unsigned long long>(hits), static_cast<unsigned long long>(misses), rate,
			static_cast<unsigned long long>(cache.Inserts()),
			static_cast<unsigned long long>(cache.Evictions()),
			static_cast<unsigned long long>(cache.Uncacheable()),
			static_cast<unsigned long long>(cache.Verified()),
			static_cast<unsigned long long>(cache.Mismatches()),
			static_cast<unsigned long long>(cache.MismatchEntity()));
		SERVER_PRINT(msg);
	}

	void Cmd_CrossBotDeployStatus()
	{
		BotManager::PrintDeployStatus();
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

	void Cmd_CrossBotNavWater()
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
		{
			SERVER_PRINT("CrossBot: no nav mesh loaded\n");
			return;
		}

		if (mesh->waterVolumes.empty())
		{
			SERVER_PRINT("CrossBot: this map has no water\n");
			return;
		}

		const std::vector<float> &surfaces = Nav::WaterSurfaces(*mesh);

		for (size_t i = 0; i < mesh->waterVolumes.size(); ++i)
		{
			const Nav::WaterVolume &vol = mesh->waterVolumes[i];
			float live = (i < surfaces.size()) ? surfaces[i] : vol.surfaceZ;

			char msg[240];
			snprintf(msg, sizeof(msg),
				"CrossBot: water %zu model *%u surface %.1f (baked %.1f, travel %.1f)%s\n", i,
				vol.modelIndex, live, vol.surfaceZ, vol.travel.z, vol.moving ? " moving" : "");
			SERVER_PRINT(msg);
		}

		size_t wetAreas = 0;
		for (const Nav::Area &area : mesh->areas)
		{
			if (Nav::AreaSubmerged(*mesh, area))
				++wetAreas;
		}

		size_t blocked = 0;
		size_t swimLive = 0;
		for (const Nav::Connection &c : mesh->conns)
		{
			bool submerged = Nav::ConnSubmerged(*mesh, c);
			if (crossbot_nav::ConnBlockedByWater(c.connType, submerged))
				++blocked;
			else if (c.connType == crossbot_nav::CONN_SWIM)
				++swimLive;
		}

		char summary[200];
		snprintf(summary, sizeof(summary),
			"CrossBot: %zu/%zu areas submerged, %zu/%zu connections switched off by water, "
			"%zu swim links live\n",
			wetAreas, mesh->areas.size(), blocked, mesh->conns.size(), swimLive);
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
	ADD_SERVER_COMMAND("crossbot_nav_water", Cmd_CrossBotNavWater);
	ADD_SERVER_COMMAND("crossbot_list_items", Cmd_CrossBotListItems);
	ADD_SERVER_COMMAND("crossbot_list_fun", Cmd_CrossBotListFun);
	ADD_SERVER_COMMAND("crossbot_list_goals", Cmd_CrossBotListGoals);
	ADD_SERVER_COMMAND("crossbot_list_objectives", Cmd_CrossBotListObjectives);
	ADD_SERVER_COMMAND("crossbot_list_gates", Cmd_CrossBotListGates);
	ADD_SERVER_COMMAND("crossbot_deploy_status", Cmd_CrossBotDeployStatus);
	ADD_SERVER_COMMAND("crossbot_airmove_cache_stats", Cmd_CrossBotAirmoveCache);
#ifdef CROSSBOT_BSPTRACE
	ADD_SERVER_COMMAND("crossbot_bsptrace_stats", Cmd_CrossBotBspTrace);
#endif
	ADD_SERVER_COMMAND("crossbot_weapons", Cmd_CrossBotWeapons);
	ADD_SERVER_COMMAND("crossbot_weaponspec", Cmd_CrossBotWeaponSpec);
	ADD_SERVER_COMMAND("crossbot_equip", Cmd_CrossBotEquip);
	ADD_SERVER_COMMAND("crossbot_spawn", Cmd_CrossBotSpawn);
	ADD_SERVER_COMMAND("crossbot_aim_test", Cmd_CrossBotAimTest);
	ADD_SERVER_COMMAND("crossbot_aim_status", Cmd_CrossBotAimStatus);
	ADD_SERVER_COMMAND("crossbot_sense_status", Cmd_CrossBotSenseStatus);
	ADD_SERVER_COMMAND("crossbot_monsters", Cmd_CrossBotMonsters);
	ADD_SERVER_COMMAND("crossbot_difficulty_status", Cmd_CrossBotDifficultyStatus);
	ADD_SERVER_COMMAND("crossbot_vantage_status", Cmd_CrossBotVantageStatus);
	ADD_SERVER_COMMAND("crossbot_camp_status", Cmd_CrossBotCampStatus);
	ADD_SERVER_COMMAND("crossbot_teams_status", Cmd_CrossBotTeamsStatus);
	ADD_SERVER_COMMAND("crossbot_combat_status", Cmd_CrossBotCombatStatus);
	ADD_SERVER_COMMAND("crossbot_usermsgs", Cmd_CrossBotUserMsgs);
	ADD_SERVER_COMMAND("crossbot_daze_status", Cmd_CrossBotDazeStatus);
	ADD_SERVER_COMMAND("crossbot_daze_test", Cmd_CrossBotDazeTest);
	ADD_SERVER_COMMAND("crossbot_fire_status", Cmd_CrossBotFireStatus);
	ADD_SERVER_COMMAND("crossbot_fire_pin", Cmd_CrossBotFirePin);
	ADD_SERVER_COMMAND("crossbot_weapon_carry", Cmd_CrossBotWeaponCarry);
}

}
