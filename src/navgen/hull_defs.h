#pragma once

namespace navgen
{

constexpr int kHullPoint = 0;
constexpr int kHullHuman = 1;
constexpr int kHullLarge = 2;
constexpr int kHullDuck  = 3;

constexpr double kHumanHullMinX = -16.0, kHumanHullMinY = -16.0, kHumanHullMinZ = -36.0;
constexpr double kHumanHullMaxX = 16.0, kHumanHullMaxY = 16.0, kHumanHullMaxZ = 36.0;
constexpr double kDuckHullMinX = -16.0, kDuckHullMinY = -16.0, kDuckHullMinZ = -18.0;
constexpr double kDuckHullMaxX = 16.0, kDuckHullMaxY = 16.0, kDuckHullMaxZ = 32.0;

constexpr double kDuckClipHullMinX = -16.0, kDuckClipHullMinY = -16.0, kDuckClipHullMinZ = -18.0;
constexpr double kDuckClipHullMaxX = 16.0, kDuckClipHullMaxY = 16.0, kDuckClipHullMaxZ = 18.0;

constexpr double kStandProbeLift = kDuckClipHullMinZ - kHumanHullMinZ;

}

