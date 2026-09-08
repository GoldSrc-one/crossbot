#pragma once

namespace PlayerModels
{

// Every player model installed for a game is a directory under models/player holding a .mdl of the
// same name: models/player/gordon/gordon.mdl. Nothing else counts, which is what keeps CrossPlay's
// own models/player out of the list -- its per-game variants (gordon_cs4.mdl, gordon_dod5.mdl) live
// in dirs that have no gordon.mdl beside them.
//
// A game with no models/player of its own falls back to valve's. DMC is why: it is Quake-shaped and
// ships one flat models/player.mdl, no directory at all.

// Fills out with a randomly chosen installed model. False when neither the game's own directory nor
// valve has one, in which case the caller leaves the engine default alone.
bool PickRandom(int iGame, char *out, unsigned int size);

// Is name one of the models PickRandom would have drawn from?
bool Installed(int iGame, const char *name);

// SERVER_PRINTs the list, one indented name per line.
void PrintList(int iGame);

// Drops the cached scan, so models added while the server is up are seen.
void Forget();

}
