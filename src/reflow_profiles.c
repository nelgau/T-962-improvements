#include "LPC214x.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "t962.h"
#include "lcd.h"
#include "nvstorage.h"
#include "eeprom.h"
#include "reflow.h"
#include "log.h"
#include "reflow_profiles.h"
#include "config.h"

/*
 * Reflow profiles may only hold up to NUMPROFILETEMPS (48) entries and
 * must be zero-terminated, i.e. only 47 entries may be used.
 * Each entry corresponds to 10s and temperatures are interpolated in-between.
 */

// Amtech 4300 63Sn/37Pb leaded profile
static const profile am4300profile = {
	"4300 63SN/37PB", {
		 50, 50, 50, 60, 73, 86,100,113,126,140,143,147,150,154,157,161, // 0-150s
		164,168,171,175,179,183,195,207,215,  0,  0,  0,  0,  0,  0,  0, // Adjust peak from 205 to 220C
		  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};

// NC-31 low-temp lead-free profile
static const profile nc31profile = {
	"NC-31 LOW-TEMP LF", {
		 50, 50, 50, 50, 55, 70, 85, 90, 95,100,102,105,107,110,112,115, // 0-150s
		117,120,122,127,132,138,148,158,160,  0,  0,  0,  0,  0,  0,  0, // Adjust peak from 158 to 165C
		  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};

// SynTECH-LF normal temp lead-free profile
static const profile syntechlfprofile = {
	"LF DESIGNED PROF", {
		 25, 25, 40, 55, 70, 85,100,115,130,145,152,155,158,161,164,167,
		170,173,176,179,182,185,188,191,194,197,200,210,220,230,240,240,
		240,240,230,220,210,200,190,180,170,160,  0,  0,  0,  0,  0,  0
	}
};

#ifdef RAMPTEST_PROFILE
// Ramp speed test temp profile
static const profile rampspeed_testprofile = {
	"RAMP SPEED TEST", {
		 50, 50, 50, 50,245,245,245,245,245,245,245,245,245,245,245,245, // 0-150s
		245,245,245,245,245,245,245,245,245,  0,  0,  0,  0,  0,  0,  0, // 160-310s
		  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  // 320-470s
	}
};
#endif

static const profile *rom_profiles[] = {
	&syntechlfprofile,
	&nc31profile,
	&am4300profile,
#ifdef RAMPTEST_PROFILE
	&rampspeed_testprofile,
#endif
};

// current profile index
static uint8_t profileidx = 0;
static int no_of_profiles = ARRAY_SIZE(rom_profiles);

// init locals from EEPROM
void Reflow_InitNV(void) {
	profileidx = NV_GetConfig(REFLOW_PROFILE);
	no_of_profiles = ARRAY_SIZE(rom_profiles) + NV_NoOfProfiles();
}

// both rom and eeprom profiles
int Reflow_NoOfProfiles(void) {
	return no_of_profiles;
}

int Reflow_GetProfileIdx(void) {
	return profileidx;
}

bool Reflow_IdxIsInEEPROM(int idx) {
	if (idx == -1)
		idx = profileidx;
	if (idx < (int) ARRAY_SIZE(rom_profiles) || idx > no_of_profiles)
		return false;
	return true;
}

int Reflow_SelectProfileIdx(int idx) {
	// TODO: probably this should not wrap here but in the interface!
	profileidx = (uint8_t) wrap(idx, 0, no_of_profiles-1);
	NV_SetConfig(REFLOW_PROFILE, profileidx);
	return profileidx;
}

// return 0 if successful
int Reflow_SaveEEProfile(void) {
	// Store profile
	if (Reflow_IdxIsInEEPROM(profileidx))
		return NV_StoreProfile(profileidx - ARRAY_SIZE(rom_profiles));
	return -1;
}

// use selected profile if index is -1
const char* Reflow_GetProfileName(int idx) {
	if (idx > no_of_profiles)
		return "unknown";
	if (idx == -1)
		idx = profileidx;
	if (Reflow_IdxIsInEEPROM(idx))
		return NV_GetProfileName(idx - ARRAY_SIZE(rom_profiles));
	return rom_profiles[idx]->name;
}

// use selected profile if index is -1
void Reflow_SetProfileName(int idx, const char *name) {
	if (idx == -1)
		idx = profileidx;
	if (Reflow_IdxIsInEEPROM(idx))
		return NV_SetProfileName(idx - ARRAY_SIZE(rom_profiles), name);
}

// return temperature at index idx
uint16_t Reflow_GetSetpointAtIdx(uint8_t idx) {
	if (idx > (NUMPROFILETEMPS - 1)) {
		return 0;
	}
	if (Reflow_IdxIsInEEPROM(profileidx))
		return NV_GetSetpoint(profileidx - ARRAY_SIZE(rom_profiles), idx);

	return rom_profiles[profileidx]->temperatures[idx];
}

// this only works for EEPROM profiles
void Reflow_SetSetpointAtIdx(uint8_t idx, uint16_t value) {
	if (idx < NUMPROFILETEMPS && value <= SETPOINT_MAX && Reflow_IdxIsInEEPROM(profileidx))
		NV_SetSetpoint(profileidx - ARRAY_SIZE(rom_profiles), idx, value);
	else
		logx(LOG_WARN, "Reflow_SetSetpoint fails: profileidx=%u, idx=%u, value=%u",
				profileidx, idx, value);
}

/*!
 * return the temperature from the profile for a specific time
 * This returns 0 if the time is not within the time used by the profile
 * and may be used as indication that the profile is done.
 * Note: this returns interpolated values, if the end is reached, the last value
 *   is not interpolated!
 */
float Reflow_GetSetpointAtTime(float time)
{
	// the profile holds temperatures for every 10s
	uint8_t index = (uint8_t) (time / 10);	// up to 2550s ~ 42min
	float rest = fmodf(time, 10.0);			// 0 .. 9

	// safe for large indices!
	float value1 = (float) Reflow_GetSetpointAtIdx(index);
	float value2 = (float) Reflow_GetSetpointAtIdx(index + 1);

	if (value2 == 0)
		return value1;
	return (value1 + ((value2 - value1) * rest) / 10.0);
}
