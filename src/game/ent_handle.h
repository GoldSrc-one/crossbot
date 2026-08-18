#pragma once

#include <extdll.h>

struct CBotEntHandle
{
	edict_t *ent = nullptr;
	int serial = 0;

	void Clear()
	{
		ent = nullptr;
		serial = 0;
	}

	void Set(edict_t *e)
	{
		ent = e;
		serial = e ? e->serialnumber : 0;
	}

	edict_t *Get() const
	{
		if (!ent || ent->free || ent->serialnumber != serial)
			return nullptr;
		return ent;
	}

	bool Is(const edict_t *e) const { return e && Get() == e; }
};

const char *CBotClassOf(const edict_t *ent);

const char *CBotPlayerNameOf(const edict_t *ent);

bool CBotIsLivePlayer(const edict_t *ent);
