#pragma once

namespace navgen
{

// dmodel_t::headnode[] indices. Not stored anywhere in the BSP itself --
// this is a hardcoded GoldSrc engine convention (mirrors dlls/extdef.h's
// point_hull/human_hull/large_hull/head_hull enum, duplicated here so navgen
// stays decoupled from the gamedll headers under external/hlsdk/dlls).
constexpr int kHullPoint = 0; // hitscan/point-trace hull, not used for nav
constexpr int kHullHuman = 1; // standing player: -16,-16,-36 to 16,16,36
constexpr int kHullLarge = 2; // large monster hull, not used for nav
constexpr int kHullDuck  = 3; // ducked player: -16,-16,-18 to 16,16,32

// Hull bounding boxes, for documentation/volume-sanity purposes only -- the
// clipnode planes themselves are already compiled per-hull by hlbsp, so the
// leaf-extraction algorithm never needs to apply these directly.
constexpr double kHumanHullMinX = -16.0, kHumanHullMinY = -16.0, kHumanHullMinZ = -36.0;
constexpr double kHumanHullMaxX = 16.0, kHumanHullMaxY = 16.0, kHumanHullMaxZ = 36.0;
constexpr double kDuckHullMinX = -16.0, kDuckHullMinY = -16.0, kDuckHullMinZ = -18.0;
constexpr double kDuckHullMaxX = 16.0, kDuckHullMaxY = 16.0, kDuckHullMaxZ = 32.0;

} // namespace navgen
