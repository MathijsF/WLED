#include "wled.h"
#include "lsm303agr_imu.h"

const char LSM303AGRUsermod::_name[]           PROGMEM = "LSM303AGR_IMU";
const char LSM303AGRUsermod::_key_enabled[]    PROGMEM = "enabled";
const char LSM303AGRUsermod::_key_shakeThresh[]PROGMEM = "shake_threshold";
const char LSM303AGRUsermod::_key_ctrlSpeed[]  PROGMEM = "control_speed";
const char LSM303AGRUsermod::_key_flash[]      PROGMEM = "enable_flash";
const char LSM303AGRUsermod::_key_minSpeed[]       PROGMEM = "min_speed";
const char LSM303AGRUsermod::_key_maxSpeed[]       PROGMEM = "max_speed";
const char LSM303AGRUsermod::_key_motionSim[]      PROGMEM = "motion_sim_audio";
const char LSM303AGRUsermod::_key_audioIntensity[] PROGMEM = "audio_intensity";
const char LSM303AGRUsermod::_key_stillThreshold[] PROGMEM = "still_threshold";
const char LSM303AGRUsermod::_key_stillTimeout[]   PROGMEM = "still_timeout_ms";
const char LSM303AGRUsermod::_key_bumpColor[]      PROGMEM = "bump_color";
const char LSM303AGRUsermod::_key_bumpHueShift[]   PROGMEM = "bump_hue_shift";
const char LSM303AGRUsermod::_key_colorFade[]      PROGMEM = "color_fade_alpha";

static LSM303AGRUsermod lsm303agr_imu;
REGISTER_USERMOD(lsm303agr_imu);
