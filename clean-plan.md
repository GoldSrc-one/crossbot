# CrossBot Code Cleanup Plan 

| Constant | navgen (double) | Nav/NavPlan (float) | Issue |
|----------|-----------------|---------------------|-------|
| Wall normal Z threshold | `kPlaneWallNormalZ` (area_merger.cpp) | `kWallNormalZ` (waypoint_plan.cpp) | **Inconsistent naming** - same concept, different names |
| Floor normal Z threshold | `kFloorNormalZ` defined in **4 files** (airmove_sampler.cpp, area_merger.cpp, main.cpp, floor_support.cpp) | Not used in NavPlan | **Duplicated definition** across navgen files |
| Player stand height | `kStepHeight` = 18.0 (floor_support.h) | `kPlanStandHeight` = 18.f (waypoint_plan.h) | **Inconsistent naming** - same value (18), different names |
| Player hull half-width | `kHumanHullMaxX` = 16.0 (hull_defs.h) | `kPlanClearance` = 16.f (waypoint_plan.h) | **Inconsistent naming** - same value (16), different names |

**Recommendation**: Add shared constants header for navgen that uses double precision, mirroring NavPlan's constants. Or move common constants to `nav_math.h` with `constexpr double` versions.

