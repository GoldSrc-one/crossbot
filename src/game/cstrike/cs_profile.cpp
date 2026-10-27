#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>
#include <in_buttons.h>

#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <vector>

#include "../../bot/bot.h"
#include "../cvars.h"
#include "../../bot/bot_ability.h"
#include "../../bot/bot_aim.h"
#include "../../bot/bot_combat.h"
#include "../../bot/bot_log.h"
#include "../../bot/bot_move.h"
#include "../../bot/bot_timing.h"
#include "../../bot/bot_weapons.h"
#include "../crossplay.h"
#include "../ent_handle.h"
#include "../fake_cmd.h"
#include "../gamedata.h"
#include "cs_profile.h"
#include "cs_weapons.h"

namespace
{
	constexpr float kJoinInterval = 1.2f;
	constexpr int kJoinTries = 3;
	constexpr int kJoined = 0;
	constexpr int kReadingText = 2;
	constexpr int kMenuChooseTeam = 1;
	constexpr int kMenuIGChooseTeam = 2;
	constexpr int kMenuChooseAppearance = 3;
	constexpr int kTeamTerrorist = 1;
	constexpr int kTeamCT = 2;
	constexpr int kTeamAuto = 5;
	constexpr int kModelCount = 4;

	constexpr int kSignalBomb = 1 << 1;
	constexpr int kSignalStateOffset = 4;

	constexpr float kPlantTimeout = 5.f;
	constexpr float kPlantRetry = 3.f;
	constexpr float kSelectInterval = 0.5f;

	constexpr float kCSThrowRate = 6.f;
	constexpr float kCSThrowSpeedMax = 750.f;

	constexpr float kFallSafeSpeed = 500.f;
	constexpr float kFallDamage = 100.f / 600.f * 1.25f;

	constexpr int kMaxPlayers = 32;

	constexpr int kShotsCap = 15;
	constexpr float kShotsDelay = 0.4f;
	constexpr float kShotsDecay = 0.0225f;

	int g_team[kMaxPlayers + 1];
	int g_joinTries[kMaxPlayers + 1];
	float g_nextJoin[kMaxPlayers + 1];

	int PlayerSlot(edict_t *pEdict)
	{
		int index = g_engfuncs.pfnIndexOfEdict(pEdict);
		return (index >= 1 && index <= kMaxPlayers) ? index : 0;
	}

	void IdleMove(edict_t *pEdict, unsigned short buttons)
	{
		byte msec = Bot_FrameMsec();
		BotTiming::BeginMove(pEdict, msec);
		g_engfuncs.pfnRunPlayerMove(pEdict, pEdict->v.v_angle, 0.f, 0.f, 0.f, buttons, 0, msec);
		BotTiming::EndMove(pEdict);
	}

	bool Flag(void *priv, int offset)
	{
		return offset >= 0 && GameData::Peek<bool>(priv, offset);
	}

	void *Bucket(void *player, int slot)
	{
		return GameData::Peek<void *>(player, g_offsets.plr_rgpPlayerItems + slot * static_cast<int>(sizeof(void *)));
	}

	bool Own(const edict_t *ent)
	{
		return XPlay::OwnerOf(ent) == XPlay::Current();
	}

	int BoxWeapon(void *box)
	{
		if (g_offsets.box_rgpPlayerItems < 0)
			return 0;

		for (int i = 0; i < kCSItemSlots; ++i)
		{
			void *item = GameData::Peek<void *>(box, g_offsets.box_rgpPlayerItems + i * static_cast<int>(sizeof(void *)));
			if (item)
				return GameData::Peek<int>(item, g_offsets.item_iId);
		}
		return 0;
	}

	bool Fits(void *owner, int id, bool vip)
	{
		CSInventorySlot slot = CS_InventorySlot(id);
		if (vip)
			return slot == CS_SLOT_PISTOL && !Bucket(owner, slot);

		bool shield = Flag(owner, g_offsets.plr_bOwnsShield);
		if (id == CS_SHIELD)
		{
			void *pistol = Bucket(owner, CS_SLOT_PISTOL);
			return !shield && !Bucket(owner, CS_SLOT_PRIMARY) &&
				!(pistol && GameData::Peek<int>(pistol, g_offsets.item_iId) == CS_ELITE);
		}
		if (slot == CS_SLOT_PRIMARY)
			return !shield && !Bucket(owner, slot);
		if (slot != CS_SLOT_GRENADE)
			return !Bucket(owner, slot);

		for (void *item = Bucket(owner, slot); item; item = GameData::Peek<void *>(item, g_offsets.item_pNext))
		{
			if (GameData::Peek<int>(item, g_offsets.item_iId) != id)
				continue;

			int ammo = GameData::Peek<int>(item, g_offsets.wpn_iPrimaryAmmoType);
			if (ammo < 0 || ammo >= MAX_AMMO_SLOTS)
				return true;
			return GameData::Peek<int>(owner, g_offsets.plr_rgAmmo + ammo * static_cast<int>(sizeof(int))) < CS_GrenadeMax(id);
		}
		return true;
	}

	int ItemWeapon(void *priv, const char *classname)
	{
		if (strcmp(classname, "weaponbox") == 0)
			return BoxWeapon(priv);
		if (strcmp(classname, "armoury_entity") == 0)
			return g_offsets.arm_iItem >= 0 ? CS_ArmouryWeapon(GameData::Peek<int>(priv, g_offsets.arm_iItem)) : 0;
		return CS_WeaponId(classname);
	}

	int Find(const BotInventory &inv, int id)
	{
		for (int i = 0; i < inv.count; ++i)
		{
			if (inv.weapons[i].id == id)
				return i;
		}
		return -1;
	}

	bool InBombZone(edict_t *pEdict)
	{
		void *priv = CBotPrivateData(pEdict);
		if (!priv || g_offsets.plr_signals < 0)
			return false;
		return (GameData::Peek<int>(priv, g_offsets.plr_signals + kSignalStateOffset) & kSignalBomb) != 0;
	}

	bool StopPlant(BotAbility::State &st, const char *why, float retry)
	{
		st.task = BotAbility::TASK_NONE;
		st.why = why;
		st.nextTry = gpGlobals->time + retry;
		return false;
	}
}

const char *CCSProfile::GetName() const
{
	return "CS";
}

bool CCSProfile::Detect(const char *gamedir) const
{
	return gamedir && (CB_STRICMP(gamedir, "cstrike") == 0 || CB_STRICMP(gamedir, "czero") == 0);
}

PmSim::FallRule CCSProfile::FallDamage() const
{
	if (CBotCvarFloat("mp_falldamage") == 0.f)
		return PmSim::kNoFall;
	return { kFallSafeSpeed, kFallDamage };
}

void CCSProfile::ApplyAddArg(int, edict_t *pEdict, const char *arg, char *out, unsigned int size) const
{
	int team = kTeamAuto;
	if (arg && arg[0] && strcmp(arg, "*") != 0)
	{
		if (CB_STRICMP(arg, "t") == 0 || CB_STRICMP(arg, "terrorist") == 0)
			team = kTeamTerrorist;
		else if (CB_STRICMP(arg, "ct") == 0)
			team = kTeamCT;
		else
			SERVER_PRINT("CrossBot: unknown CS team -- joining automatically. Teams: t ct *\n");
	}

	int slot = PlayerSlot(pEdict);
	if (slot)
	{
		g_team[slot] = team;
		g_joinTries[slot] = 0;
		g_nextJoin[slot] = 0.f;
	}

	if (out && size)
		snprintf(out, size, "%s", team == kTeamTerrorist ? "t" : team == kTeamCT ? "ct" : "*");
}

bool CCSProfile::PreThink(edict_t *pEdict) const
{
	void *priv = CBotPrivateData(pEdict);
	int slot = PlayerSlot(pEdict);
	if (!priv || !slot || g_offsets.plr_iJoiningState < 0 || g_offsets.plr_iMenu < 0)
		return false;

	int join = GameData::Peek<int>(priv, g_offsets.plr_iJoiningState);
	if (join == kJoined)
		return false;

	float now = gpGlobals->time;
	unsigned short buttons = 0;
	if (join == kReadingText)
	{
		buttons = (pEdict->v.button & IN_ATTACK) ? 0 : IN_ATTACK;
	}
	else if (now >= g_nextJoin[slot] || g_nextJoin[slot] - now > kJoinInterval)
	{
		int menu = GameData::Peek<int>(priv, g_offsets.plr_iMenu);
		char cmd[32] = "";
		if (menu == kMenuChooseAppearance)
		{
			snprintf(cmd, sizeof(cmd), "joinclass %d", RANDOM_LONG(1, kModelCount));
		}
		else if (menu == kMenuChooseTeam || menu == kMenuIGChooseTeam)
		{
			int team = (g_joinTries[slot]++ < kJoinTries) ? g_team[slot] : kTeamAuto;
			snprintf(cmd, sizeof(cmd), "jointeam %d", team ? team : kTeamAuto);
		}

		if (cmd[0])
		{
			g_nextJoin[slot] = now + kJoinInterval;
			FakeCmd::Run(pEdict, cmd);
		}
	}

	IdleMove(pEdict, buttons);
	return true;
}

void CCSProfile::Respawn(edict_t *pEdict) const
{
	IdleMove(pEdict, 0);
}

bool CCSProfile::AbilityRun(CBot &bot, const Nav::NavMesh &) const
{
	BotAbility::State &st = bot.ability;
	edict_t *pEdict = bot.pEdict;
	float now = gpGlobals->time;

	bool carrying = Find(bot.inventory, CS_C4) >= 0;
	bool ground = (pEdict->v.flags & FL_ONGROUND) != 0;
	bool enemy = bot.sense.target.Get() != nullptr;

	if (st.task == BotAbility::TASK_PLANT)
	{
		if (!carrying)
		{
			++st.plants;
			BOT_VLOG("CrossBot: %s planted the bomb\n", bot.name);
			return StopPlant(st, "planted", 0.f);
		}
		if (enemy)
			return StopPlant(st, "enemy", kPlantRetry);
		if (!ground || !InBombZone(pEdict))
			return StopPlant(st, "left the zone", 0.f);
		if (now - st.start > kPlantTimeout)
			return StopPlant(st, "timed out", kPlantRetry);
	}
	else
	{
		if (st.task != BotAbility::TASK_NONE || !carrying || enemy || !ground || now < st.nextTry ||
			!InBombZone(pEdict) || BotDirective::NoAutoObjective(bot))
			return false;

		st.task = BotAbility::TASK_PLANT;
		st.start = now;
		st.why = "in the bomb zone";
		BOT_VLOG("CrossBot: %s is planting the bomb\n", bot.name);
	}

	unsigned short buttons = IN_DUCK;
	const BotInventory &inv = bot.inventory;
	if (inv.activeIndex >= 0 && inv.activeIndex < inv.count && inv.weapons[inv.activeIndex].id == CS_C4)
	{
		buttons |= IN_ATTACK;
	}
	else if (now >= st.nextSelect)
	{
		SelectWeapon(pEdict, *CS_WeaponSpec(CS_C4));
		st.nextSelect = now + kSelectInterval;
	}

	Vector angles = Bot_UpdateFacing(bot);
	Bot_RunPlayerMove(bot, angles, 0.f, 0.f, buttons);
	return true;
}

int CCSProfile::Abilities(const AbilityInfo *&list) const
{
	static const AbilityInfo kList[] = {
		{ "plant", nullptr, "plant the C4 where you stand; needs the C4 and a bomb zone" },
	};
	list = kList;
	return static_cast<int>(sizeof(kList) / sizeof(kList[0]));
}

int CCSProfile::RadioCommands(const RadioInfo *&list) const
{
	static const RadioInfo kList[] = {
		{ "coverme", 0, "%!MRAD_COVERME", "Cover me!" },
		{ "takepoint", 0, "%!MRAD_TAKEPOINT", "You take the point." },
		{ "holdpos", 0, "%!MRAD_POSITION", "Hold this position." },
		{ "regroup", 0, "%!MRAD_REGROUP", "Regroup, team." },
		{ "followme", 0, "%!MRAD_FOLLOWME", "Follow me." },
		{ "takingfire", 0, "%!MRAD_HITASSIST", "Taking fire, need assistance!" },
		{ "go", 0, "%!MRAD_GO", "Go go go!" },
		{ "fallback", 0, "%!MRAD_FALLBACK", "Team, fall back!" },
		{ "sticktog", 0, "%!MRAD_STICKTOG", "Stick together, team." },
		{ "getinpos", 0, "%!MRAD_GETINPOS", "Get in position and wait for my go." },
		{ "stormfront", 0, "%!MRAD_STORMFRONT", "Storm the front!" },
		{ "report", 0, "%!MRAD_REPORTIN", "Report in, team." },
		{ "roger", 0, "%!MRAD_ROGER", "Roger that." },
		{ "roger", 0, "%!MRAD_AFFIRM", "Affirmative." },
		{ "enemyspot", 0, "%!MRAD_ENEMYSPOT", "Enemy spotted." },
		{ "needbackup", 0, "%!MRAD_BACKUP", "Need backup." },
		{ "sectorclear", 0, "%!MRAD_CLEAR", "Sector clear." },
		{ "inposition", 0, "%!MRAD_INPOS", "I'm in position." },
		{ "reportingin", 0, "%!MRAD_REPRTINGIN", "Reporting in." },
		{ "getout", 0, "%!MRAD_BLOW", "Get out of there, it's gonna blow!" },
		{ "negative", 0, "%!MRAD_NEGATIVE", "Negative." },
		{ "enemydown", 0, "%!MRAD_ENEMYDOWN", "Enemy down." },
	};
	list = kList;
	return static_cast<int>(sizeof(kList) / sizeof(kList[0]));
}

bool CCSProfile::StartAbility(CBot &bot, const char *name, const char *, edict_t *) const
{
	BotAbility::State &st = bot.ability;
	if (CB_STRICMP(name, "plant") != 0)
		return false;
	if (st.task != BotAbility::TASK_NONE)
	{
		st.why = "busy";
		return false;
	}
	if (Find(bot.inventory, CS_C4) < 0)
	{
		st.why = "no C4";
		return false;
	}
	if (!InBombZone(bot.pEdict))
	{
		st.why = "not in a bomb zone";
		return false;
	}

	st.task = BotAbility::TASK_PLANT;
	st.start = gpGlobals->time;
	st.why = "ordered to plant";
	BOT_VLOG("CrossBot: %s is planting the bomb on orders\n", bot.name);
	return true;
}

unsigned short CCSProfile::FireToggle(const CBot &bot, bool firing) const
{
	void *priv = CBotPrivateData(bot.pEdict);
	if (!priv || !Flag(priv, g_offsets.plr_bOwnsShield))
		return 0;

	bool drawn = Flag(priv, g_offsets.plr_bShieldDrawn);
	bool want = BotCombat::Active(bot) && !firing;
	if (drawn == want)
		return 0;

	BOT_VLOG("CrossBot: %s %s its shield\n", bot.name, drawn ? "lowers" : "raises");
	return IN_ATTACK2;
}

bool CCSProfile::SpreadWide(edict_t *player, int id, void *weapon, float limit, float &ready) const
{
	const CSSpread *s = CS_Spread(id);
	if (!s || !player || !weapon || !(player->v.flags & FL_ONGROUND))
		return false;

	float a = s->standA;
	float b = s->standB;
	if (player->v.velocity.Length2D() > s->moveSpeed)
	{
		a = s->moveA;
		b = s->moveB;
	}
	else if (player->v.flags & FL_DUCKING)
	{
		a = s->duckA;
		b = s->duckB;
	}

	float now = gpGlobals->time;
	if (s->recover > 0.f)
	{
		if (g_offsets.wpn_flLastFire < 0)
			return false;
		if (a + b * (1.f - s->base) > limit || a + b * (1.f - s->floor) <= limit)
			return false;

		ready = GameData::Peek<float>(weapon, g_offsets.wpn_flLastFire) + s->recover;
		return now < ready;
	}

	if (g_offsets.wpn_flAccuracy < 0 || g_offsets.wpn_iShotsFired < 0)
		return false;
	if (a + b * s->base > limit || a + b * GameData::Peek<float>(weapon, g_offsets.wpn_flAccuracy) <= limit)
		return false;

	int shots = std::min(GameData::Peek<int>(weapon, g_offsets.wpn_iShotsFired), kShotsCap);
	ready = now + kShotsDelay + kShotsDecay * static_cast<float>(shots);
	return true;
}

bool CCSProfile::BraceSpeed(edict_t *player, int id, void *weapon, float limit, float &speed) const
{
	speed = FLT_MAX;
	const CSSpread *s = CS_Spread(id);
	if (!s || !player || !weapon || limit <= 0.f)
		return true;

	float acc = g_offsets.wpn_flAccuracy >= 0 ? GameData::Peek<float>(weapon, g_offsets.wpn_flAccuracy) : s->base;
	float cone = s->moveA + s->moveB * (s->recover > 0.f ? 1.f - acc : acc);
	if (!Zoomed(player))
		cone += s->unzoomed;
	if (cone > limit)
		speed = s->moveSpeed;
	return true;
}

bool CCSProfile::ItemUsable(edict_t *player, edict_t *item) const
{
	void *owner = CBotPrivateData(player);
	if (!owner || !item || item->free || !Own(item))
		return true;

	void *priv = CBotPrivateData(item);
	if (!priv)
		return true;

	const char *classname = CBotClassOf(item);
	bool vip = Flag(owner, g_offsets.plr_bIsVIP);
	bool box = strcmp(classname, "weaponbox") == 0;

	if (vip && (box || strcmp(classname, "armoury_entity") == 0))
		return false;
	if (box && Flag(priv, g_offsets.box_bIsBomb))
		return g_offsets.plr_iTeam < 0 || GameData::Peek<int>(owner, g_offsets.plr_iTeam) == kTeamTerrorist;

	int id = ItemWeapon(priv, classname);
	return id == 0 || Fits(owner, id, vip);
}

ItemKind CCSProfile::ClassifyItem(edict_t *ent) const
{
	if (ent && !ent->free && strcmp(CBotClassOf(ent), "armoury_entity") == 0)
	{
		void *priv = Own(ent) ? CBotPrivateData(ent) : nullptr;
		if (priv && g_offsets.arm_iItem >= 0 && CS_ArmouryArmor(GameData::Peek<int>(priv, g_offsets.arm_iItem)))
			return ITEM_PICKUP;
		return ITEM_WEAPON;
	}
	return CHLDMProfile::ClassifyItem(ent);
}

const WeaponSpec *CCSProfile::GetWeaponSpec(int id) const
{
	return CS_WeaponSpec(id);
}

int CCSProfile::WeaponSlotCount() const
{
	return kCSWeaponSlots;
}

const WeaponSpec *CCSProfile::WeaponSlot(int slot) const
{
	return CS_WeaponSlot(slot);
}

int CCSProfile::ItemSlots() const
{
	return kCSItemSlots;
}

bool CCSProfile::ItemPreferred(edict_t *ent, ItemKind kind) const
{
	if (kind != ITEM_WEAPON || !ent || ent->free)
		return false;

	void *priv = Own(ent) ? CBotPrivateData(ent) : nullptr;
	if (!priv)
		return false;

	return CS_InventorySlot(ItemWeapon(priv, CBotClassOf(ent))) == CS_SLOT_PRIMARY;
}

int CCSProfile::MeleeWeapon() const
{
	return CS_KNIFE;
}

bool CCSProfile::IsMeleeWeapon(int id) const
{
	return id == CS_KNIFE;
}

int CCSProfile::AmbushWeapon(AmbushKind kind) const
{
	switch (kind)
	{
	case AMBUSH_GRENADE: return CS_HEGRENADE;
	case AMBUSH_FLASH: return CS_FLASHBANG;
	case AMBUSH_SMOKE: return CS_SMOKEGRENADE;
	default: return 0;
	}
}

int CCSProfile::DeployWeapon(DeployKind) const
{
	return 0;
}

const char *CCSProfile::DeployEntity(DeployKind) const
{
	return nullptr;
}

bool CCSProfile::ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation, float &speed) const
{
	return ThrowArc(mode, viewPitch, kCSThrowRate, kCSThrowSpeedMax, elevation, speed);
}

namespace
{
	constexpr int kTextureNameMax = 16;
	constexpr float kWallbangMinDamage = 10.f;
	constexpr float kFalloffStep = 500.f;
	constexpr int kEntityPower = 42;
	constexpr float kEntityModifier = 0.75f;
	constexpr float kWorldModifier = 0.5f;
	constexpr float kWorldShrink = 0.5f;
	constexpr float kEntityShrink = 0.75f;

	struct Material
	{
		char name[kTextureNameMax + 1];
		char type;
	};

	std::vector<Material> g_materials;
	bool g_materialsLoaded = false;

	void LoadMaterials(const char *gamedir)
	{
		g_materialsLoaded = true;

		char path[256];
		int length = 0;
		byte *file = nullptr;
		const char *dirs[] = { XPlay::HyperDir(), gamedir };
		for (const char *dir : dirs)
		{
			if (file || !dir)
				continue;
			snprintf(path, sizeof(path), "%s/sound/materials.txt", dir);
			file = LOAD_FILE_FOR_ME(path, &length);
		}
		if (!file)
			return;

		const char *p = reinterpret_cast<const char *>(file);
		const char *end = p + length;
		while (p < end)
		{
			const char *eol = p;
			while (eol < end && *eol != '\n')
				++eol;

			const char *c = p;
			while (c < eol && isspace(static_cast<unsigned char>(*c)))
				++c;

			if (c < eol && *c != '/' && isalpha(static_cast<unsigned char>(*c)))
			{
				Material m = {};
				m.type = static_cast<char>(toupper(static_cast<unsigned char>(*c++)));
				while (c < eol && isspace(static_cast<unsigned char>(*c)))
					++c;
				int n = 0;
				while (c < eol && !isspace(static_cast<unsigned char>(*c)) && n < kTextureNameMax)
					m.name[n++] = *c++;
				if (n > 0)
					g_materials.push_back(m);
			}
			p = eol + 1;
		}

		FREE_FILE(file);
	}

	char TextureType(edict_t *shooter, const TraceResult &tr, const Vector &src, const Vector &end)
	{
		if (tr.flFraction >= 1.f)
			return 0;

		if (!g_materialsLoaded)
			LoadMaterials(XPlay::GameDir(XPlay::OwnerOf(shooter)));

		edict_t *hit = tr.pHit ? tr.pHit : INDEXENT(0);
		const char *name = TRACE_TEXTURE(hit, const_cast<float *>(&src.x), const_cast<float *>(&end.x));
		if (!name)
			return 0;

		if (*name == '-' || *name == '+')
			name += 2;
		if (*name == '{' || *name == '!' || *name == '~' || *name == ' ')
			++name;

		for (const Material &m : g_materials)
		{
			if (CB_STRNICMP(name, m.name, kTextureNameMax) == 0)
				return m.type;
		}
		return 'C';
	}

	void ApplyMaterial(char type, int &power, float &modifier)
	{
		switch (type)
		{
		case 'M': power = static_cast<int>(power * 0.15f); modifier = 0.2f; break;
		case 'C': power = static_cast<int>(power * 0.25f); break;
		case 'G': power = static_cast<int>(power * 0.5f); modifier = 0.4f; break;
		case 'V': power = static_cast<int>(power * 0.5f); modifier = 0.45f; break;
		case 'T': power = static_cast<int>(power * 0.65f); modifier = 0.3f; break;
		case 'P': power = static_cast<int>(power * 0.4f); modifier = 0.45f; break;
		case 'W': modifier = 0.6f; break;
		default: break;
		}
	}
}

bool CCSProfile::Penetrates(const CBot &bot, int id, const Vector &eye, const Vector &dir,
	float entry) const
{
	const CSBullet *b = CS_Bullet(id);
	if (!b)
		return false;

	int penetration = b->penetration;
	int power = b->power;
	int damage = b->damage;
	float modifier = kWorldModifier;
	float segment = b->range;
	float base = 0.f;
	Vector src = eye;

	while (penetration != 0)
	{
		Vector end = src + dir * segment;
		TraceResult tr;
		TRACE_LINE(src, end, ignore_monsters, bot.pEdict, &tr);
		ApplyMaterial(TextureType(bot.pEdict, tr, src, end), power, modifier);

		float travelled = tr.flFraction * segment;
		if (base + travelled >= entry)
		{
			float hit = damage * powf(b->rangeMod, (entry - base) / kFalloffStep);
			return hit >= kWallbangMinDamage;
		}
		if (tr.flFraction >= 1.f)
			return false;

		--penetration;
		damage = static_cast<int>(damage * powf(b->rangeMod, travelled / kFalloffStep));
		if (travelled > b->distance)
			penetration = 0;
		float shrink = kWorldShrink;
		if (tr.pHit && tr.pHit->v.solid != SOLID_BSP)
		{
			power = kEntityPower;
			modifier = kEntityModifier;
			shrink = kEntityShrink;
		}
		if (penetration == 0)
			return false;

		src = tr.vecEndPos + dir * static_cast<float>(power);
		base += travelled + power;
		segment = (segment - travelled) * shrink;
		damage = static_cast<int>(damage * modifier);
	}
	return false;
}
