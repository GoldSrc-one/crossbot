#include "airmove_cache.h"

#include <string.h>

namespace NavPlan
{

namespace
{

uint32_t FloatBits(float v)
{
	uint32_t bits;
	memcpy(&bits, &v, sizeof(bits));
	return bits;
}

uint32_t MixWord(uint32_t h, uint32_t w)
{
	h ^= w;
	h *= 16777619u;
	h ^= h >> 13;
	return h;
}

uint32_t HashKey(const AirmoveKey &key)
{
	uint32_t h = 2166136261u;
	h = MixWord(h, FloatBits(key.x));
	h = MixWord(h, FloatBits(key.y));
	h = MixWord(h, FloatBits(key.z));
	h = MixWord(h, FloatBits(key.yaw));
	h = MixWord(h, FloatBits(key.speed));
	h = MixWord(h, key.physics);
	h = MixWord(h, static_cast<uint32_t>(key.jump) | (static_cast<uint32_t>(key.crouch) << 8));
	return h ^ (h >> 16);
}

bool SameKey(const AirmoveKey &a, const AirmoveKey &b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z && a.yaw == b.yaw && a.speed == b.speed &&
		a.physics == b.physics && a.jump == b.jump && a.crouch == b.crouch;
}

}

uint32_t AirmovePhysicsKey(float maxSpeed, float gravity, float airAccelerate, float maxVelocity)
{
	float fields[4] = { maxSpeed, gravity, airAccelerate, maxVelocity };
	uint32_t h = 2166136261u;
	for (int i = 0; i < 4; ++i)
		h = MixWord(h, FloatBits(fields[i]));
	return h ^ (h >> 16);
}

void AirmoveCache::Clear()
{
	for (size_t i = 0; i < kAirmoveCacheSlots; ++i)
	{
		m_slots[i].used = false;
		m_slots[i].freq = 0;
		m_slots[i].stamp = 0;
	}
	m_tick = 0;
	m_occupancy = 0;
	ResetCounters();
}

uint32_t AirmoveCache::Score(const Slot &slot) const
{
	uint32_t decay = (m_tick - slot.stamp) / kAirmoveAgeStep;
	if (decay >= kAirmoveMaxDecay)
		return 0;
	return static_cast<uint32_t>(slot.freq) >> decay;
}

void AirmoveCache::ResetCounters()
{
	m_hits = 0;
	m_misses = 0;
	m_inserts = 0;
	m_evictions = 0;
	m_uncacheable = 0;
	m_mismatches = 0;
	m_mismatchEntity = 0;
	m_verified = 0;
}

bool AirmoveCache::Lookup(const AirmoveKey &key, AirmoveResult &out)
{
	++m_tick;

	size_t base = HashKey(key) & kAirmoveCacheMask;
	for (size_t i = 0; i < kAirmoveCacheProbe; ++i)
	{
		Slot &slot = m_slots[(base + i) & kAirmoveCacheMask];
		if (!slot.used)
			break;
		if (SameKey(slot.key, key))
		{
			out = slot.value;
			slot.stamp = m_tick;
			if (slot.freq < kAirmoveMaxFreq)
				++slot.freq;
			++m_hits;
			return true;
		}
	}

	++m_misses;
	return false;
}

void AirmoveCache::Insert(const AirmoveKey &key, const AirmoveResult &value)
{
	size_t base = HashKey(key) & kAirmoveCacheMask;
	size_t victimIndex = base;
	uint32_t victimScore = 0xFFFFFFFFu;
	uint32_t victimStamp = 0;

	for (size_t i = 0; i < kAirmoveCacheProbe; ++i)
	{
		size_t index = (base + i) & kAirmoveCacheMask;
		Slot &slot = m_slots[index];

		if (!slot.used)
		{
			slot.key = key;
			slot.value = value;
			slot.stamp = m_tick;
			slot.freq = 1;
			slot.used = true;
			++m_occupancy;
			++m_inserts;
			return;
		}

		if (SameKey(slot.key, key))
		{
			slot.value = value;
			return;
		}

		uint32_t score = Score(slot);
		if (score < victimScore || (score == victimScore && slot.stamp < victimStamp))
		{
			victimIndex = index;
			victimScore = score;
			victimStamp = slot.stamp;
		}
	}

	Slot &victim = m_slots[victimIndex];
	victim.key = key;
	victim.value = value;
	victim.stamp = m_tick;
	victim.freq = 1;
	++m_inserts;
	++m_evictions;
}

void AirmoveCache::NoteVerified(const AirmoveResult &cached, const AirmoveResult &fresh,
	bool freshTouchedEntity)
{
	++m_verified;
	if (cached.outcome != fresh.outcome || cached.end.x != fresh.end.x ||
		cached.end.y != fresh.end.y || cached.end.z != fresh.end.z)
	{
		++m_mismatches;
		if (freshTouchedEntity)
			++m_mismatchEntity;
	}
}

}
