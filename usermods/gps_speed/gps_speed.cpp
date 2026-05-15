#include "wled.h"
#include "gps_speed.h"

const char GPSSpeedUsermod::_name[]           PROGMEM = "GPS_Speed";
const char GPSSpeedUsermod::_key_enabled[]    PROGMEM = "enabled";
const char GPSSpeedUsermod::_key_maxSpeedKmh[]PROGMEM = "max_speed_kmh";
const char GPSSpeedUsermod::_key_minSpeedKmh[]PROGMEM = "min_speed_kmh";
const char GPSSpeedUsermod::_key_minSpeed[]   PROGMEM = "min_speed";
const char GPSSpeedUsermod::_key_maxSpeed[]   PROGMEM = "max_speed";
const char GPSSpeedUsermod::_key_controlSpeed[]    PROGMEM = "control_speed";
const char GPSSpeedUsermod::_key_controlIntensity[]PROGMEM = "control_intensity";
const char GPSSpeedUsermod::_key_minIntensity[]    PROGMEM = "min_intensity";
const char GPSSpeedUsermod::_key_maxIntensity[]    PROGMEM = "max_intensity";

static GPSSpeedUsermod gps_speed;
REGISTER_USERMOD(gps_speed);
