#include "game_profile.h"
#include "dmc/dmc_profile.h"
#include "gearbox/gearbox_profile.h"
#include "hldm/hldm_profile.h"
#include "ricochet/ricochet_profile.h"

IGameProfile *GameProfile_Detect(const char *gamedir)
{
	static CHLDMProfile hldmProfile;
	static CRicochetProfile ricochetProfile;
	static CGearboxProfile gearboxProfile;
	static CDMCProfile dmcProfile;

	if (hldmProfile.Detect(gamedir))
		return &hldmProfile;
	if (ricochetProfile.Detect(gamedir))
		return &ricochetProfile;
	if (gearboxProfile.Detect(gamedir))
		return &gearboxProfile;
	if (dmcProfile.Detect(gamedir))
		return &dmcProfile;

	return NULL;
}
