#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "mathlib.h"
#include "vector.h"

namespace NavPlan
{

constexpr size_t kAirmoveCacheSlots = 65536;
constexpr size_t kAirmoveCacheMask = kAirmoveCacheSlots - 1;
constexpr size_t kAirmoveCacheProbe = 8;

constexpr uint32_t kAirmoveAgeStep = 16384;
constexpr uint32_t kAirmoveMaxDecay = 8;
constexpr uint8_t kAirmoveMaxFreq = 255;

enum AirmoveOutcome
{
	AIRMOVE_LANDED = 0,
	AIRMOVE_WEDGED,
	AIRMOVE_TIMEOUT
};

struct AirmoveKey
{
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;
	float yaw = 0.f;
	float speed = 0.f;
	uint32_t physics = 0;
	uint8_t jump = 0;
	uint8_t crouch = 0;
};

struct AirmoveResult
{
	Vector end;
	uint8_t outcome = AIRMOVE_TIMEOUT;
};

uint32_t AirmovePhysicsKey(float maxSpeed, float gravity, float airAccelerate, float maxVelocity);

class AirmoveCache
{
public:
	void Clear();
	void ResetCounters();

	bool Lookup(const AirmoveKey &key, AirmoveResult &out);
	void Insert(const AirmoveKey &key, const AirmoveResult &value);
	void NoteUncacheable() { ++m_uncacheable; }

	void SetVerify(bool on) { m_verify = on; }
	bool Verify() const { return m_verify; }
	void NoteVerified(const AirmoveResult &cached, const AirmoveResult &fresh, bool freshTouchedEntity);

	size_t Capacity() const { return kAirmoveCacheSlots; }
	size_t Occupancy() const { return m_occupancy; }
	uint64_t Hits() const { return m_hits; }
	uint64_t Misses() const { return m_misses; }
	uint64_t Inserts() const { return m_inserts; }
	uint64_t Evictions() const { return m_evictions; }
	uint64_t Uncacheable() const { return m_uncacheable; }
	uint64_t Mismatches() const { return m_mismatches; }
	uint64_t MismatchEntity() const { return m_mismatchEntity; }
	uint64_t Verified() const { return m_verified; }

private:
	struct Slot
	{
		AirmoveKey key;
		AirmoveResult value;
		uint32_t stamp = 0;
		uint8_t freq = 0;
		bool used = false;
	};

	uint32_t Score(const Slot &slot) const;

	Slot m_slots[kAirmoveCacheSlots];
	uint32_t m_tick = 0;
	size_t m_occupancy = 0;
	uint64_t m_hits = 0;
	uint64_t m_misses = 0;
	uint64_t m_inserts = 0;
	uint64_t m_evictions = 0;
	uint64_t m_uncacheable = 0;
	uint64_t m_mismatches = 0;
	uint64_t m_mismatchEntity = 0;
	uint64_t m_verified = 0;
	bool m_verify = false;
};

}
