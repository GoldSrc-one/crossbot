#pragma once

#include "game_profile.h"

class CHLDMProfile : public IGameProfile
{
public:
	const char *GetName() const override;
	unsigned int GetFlags() const override;

	bool Detect() const override;
	bool IsDead(edict_t *pEdict) const override;
	void Respawn(edict_t *pEdict) const override;
};

