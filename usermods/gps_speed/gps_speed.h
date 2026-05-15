#pragma once
#include "wled.h"
#include <Adafruit_GPS.h>

// Unique usermod ID — also added to wled00/const.h
#define USERMOD_ID_GPS_SPEED 60

/*
 * GPS Speed usermod for WLED
 *
 * Reads true ground speed from an Adafruit Mini GPS PA1010D (product 4415)
 * over I2C and maps it to the WLED effect speed parameter.
 *
 * Shares the I2C bus with other sensors (e.g. LSM303AGR) — just daisy-chain
 * the Stemma QT / Qwiic connectors.
 *
 * GPS I2C address: 0x10
 *
 * um_data exports (USERMOD_ID_GPS_SPEED):
 *   [0] float    speed_kmh  — GPS ground speed in km/h (0 when no fix)
 *   [1] float    heading    — course over ground in degrees (0–360)
 *   [2] uint8_t  satellites — number of tracked satellites
 *   [3] uint8_t  fix        — 1 if GPS has a position fix, 0 otherwise
 *
 * When control_speed is on this usermod owns segment speed.  Turn off
 * control_speed on the LSM303AGR usermod if you prefer GPS-based speed.
 */
class GPSSpeedUsermod : public Usermod {
 private:
  Adafruit_GPS _gps{&Wire};
  bool _enabled     = true;
  bool _initialized = false;

  // GPS readings shared via um_data
  float   _speedKmh  = 0.0f;
  float   _heading   = 0.0f;
  uint8_t _satellites = 0;
  uint8_t _hasFix    = 0;

  um_data_t _um_data;

  // Config
  float   _maxSpeedKmh      = 30.0f;
  float   _minSpeedKmh      = 0.5f;
  uint8_t _minSpeed         = 30;
  uint8_t _maxSpeed         = 230;
  bool    _controlSpeed     = true;
  bool    _controlIntensity = true;
  uint8_t _minIntensity     = 20;
  uint8_t _maxIntensity     = 230;

  static const char _name[];
  static const char _key_enabled[];
  static const char _key_maxSpeedKmh[];
  static const char _key_minSpeedKmh[];
  static const char _key_minSpeed[];
  static const char _key_maxSpeed[];
  static const char _key_controlSpeed[];
  static const char _key_controlIntensity[];
  static const char _key_minIntensity[];
  static const char _key_maxIntensity[];

 public:

  void setup() override {
    if (!_enabled) return;
    if (i2c_scl < 0 || i2c_sda < 0) {
      DEBUG_PRINTLN(F("GPS: I2C pins not configured"));
      return;
    }

    if (_um_data.u_size == 0) {
      _um_data.u_size = 4;
      _um_data.u_type = new um_types_t[4];
      _um_data.u_data = new void*[4];
      _um_data.u_data[0] = &_speedKmh;   _um_data.u_type[0] = UMT_FLOAT;
      _um_data.u_data[1] = &_heading;    _um_data.u_type[1] = UMT_FLOAT;
      _um_data.u_data[2] = &_satellites; _um_data.u_type[2] = UMT_BYTE;
      _um_data.u_data[3] = &_hasFix;     _um_data.u_type[3] = UMT_BYTE;
    }

    if (!_gps.begin(0x10)) {
      DEBUG_PRINTLN(F("GPS: not found on I2C bus (addr 0x10)"));
      return;
    }

    // RMC gives speed + heading, GGA gives fix quality + satellites
    _gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    _gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    _initialized = true;
    DEBUG_PRINTLN(F("GPS: initialised OK"));
  }

  void loop() override {
    if (!_enabled || !_initialized) return;

    // Read bytes from the GPS FIFO — must be called every loop to avoid overflow
    _gps.read();

    if (_gps.newNMEAreceived()) {
      if (_gps.parse(_gps.lastNMEA())) {
        _hasFix     = _gps.fix ? 1 : 0;
        _satellites = _gps.satellites;
        if (_hasFix) {
          _speedKmh = _gps.speed * 1.852f;  // knots → km/h
          _heading  = _gps.angle;
        } else {
          _speedKmh = 0.0f;
        }
      }
    }

    bool stopped = !_hasFix || _speedKmh < _minSpeedKmh;
    long speedScaled = stopped ? 0L : (long)(constrain(_speedKmh, 0.0f, _maxSpeedKmh) * 10.0f);
    long speedMax    = (long)(_maxSpeedKmh * 10.0f);

    if (_controlSpeed) {
      uint8_t newSpeed = stopped
        ? _minSpeed
        : (uint8_t)map(speedScaled, 0L, speedMax, (long)_minSpeed, (long)_maxSpeed);
      for (uint8_t s = 0; s < strip.getSegmentsNum(); s++) {
        Segment& seg = strip.getSegment(s);
        if (seg.isActive()) seg.speed = newSpeed;
      }
    }

    if (_controlIntensity) {
      uint8_t newIntensity = stopped
        ? _minIntensity
        : (uint8_t)map(speedScaled, 0L, speedMax, (long)_minIntensity, (long)_maxIntensity);
      for (uint8_t s = 0; s < strip.getSegmentsNum(); s++) {
        Segment& seg = strip.getSegment(s);
        if (seg.isActive()) seg.intensity = newIntensity;
      }
    }
  }

  bool getUMData(um_data_t **data) override {
    if (!data || !_enabled || !_initialized) return false;
    *data = &_um_data;
    return true;
  }

  void addToJsonInfo(JsonObject& root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    if (!_enabled) {
      user.createNestedArray("GPS").add("disabled");
      return;
    }
    if (i2c_scl < 0 || i2c_sda < 0) {
      user.createNestedArray("GPS").add("I2C not configured - set pins in Config > Hardware");
      return;
    }
    if (!_initialized) {
      user.createNestedArray("GPS").add("not found on I2C bus (addr 0x10, check wiring)");
      return;
    }
    if (!_hasFix) {
      JsonArray a = user.createNestedArray("GPS");
      a.add("acquiring fix...");
      a.add(String(_satellites) + " sats");
      return;
    }

    JsonArray spd = user.createNestedArray("GPS speed");
    spd.add(roundf(_speedKmh * 10.0f) / 10.0f);
    spd.add("km/h");

    JsonArray hdg = user.createNestedArray("GPS heading");
    hdg.add(roundf(_heading));
    hdg.add("\xb0");

    JsonArray sat = user.createNestedArray("GPS satellites");
    sat.add(_satellites);
  }

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_key_enabled)]      = _enabled;
    top[FPSTR(_key_maxSpeedKmh)]  = _maxSpeedKmh;
    top[FPSTR(_key_minSpeedKmh)]  = _minSpeedKmh;
    top[FPSTR(_key_minSpeed)]          = _minSpeed;
    top[FPSTR(_key_maxSpeed)]          = _maxSpeed;
    top[FPSTR(_key_controlSpeed)]      = _controlSpeed;
    top[FPSTR(_key_controlIntensity)]  = _controlIntensity;
    top[FPSTR(_key_minIntensity)]      = _minIntensity;
    top[FPSTR(_key_maxIntensity)]      = _maxIntensity;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[FPSTR(_name)];
    bool complete = !top.isNull();
    complete &= getJsonValue(top[FPSTR(_key_enabled)],      _enabled,       true);
    complete &= getJsonValue(top[FPSTR(_key_maxSpeedKmh)],  _maxSpeedKmh,   30.0f);
    complete &= getJsonValue(top[FPSTR(_key_minSpeedKmh)],  _minSpeedKmh,   0.5f);
    complete &= getJsonValue(top[FPSTR(_key_minSpeed)],     _minSpeed,      (uint8_t)30);
    complete &= getJsonValue(top[FPSTR(_key_maxSpeed)],     _maxSpeed,      (uint8_t)230);
    complete &= getJsonValue(top[FPSTR(_key_controlSpeed)],     _controlSpeed,     true);
    complete &= getJsonValue(top[FPSTR(_key_controlIntensity)], _controlIntensity, true);
    complete &= getJsonValue(top[FPSTR(_key_minIntensity)],     _minIntensity,     (uint8_t)20);
    complete &= getJsonValue(top[FPSTR(_key_maxIntensity)],     _maxIntensity,     (uint8_t)230);
    return complete;
  }

  uint16_t getId() override { return USERMOD_ID_GPS_SPEED; }
};

// Definitions are in gps_speed.cpp
