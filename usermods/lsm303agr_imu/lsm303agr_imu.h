#pragma once
#include "wled.h"
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>

// Unique usermod ID — also added to wled00/const.h
#define USERMOD_ID_LSM303AGR_IMU 59

/*
 * LSM303AGR IMU usermod for WLED
 *
 * Reads 3-axis acceleration from an Adafruit LSM303AGR over I2C and applies
 * two motion-reactive behaviours to the running LED effect:
 *
 *   1. Speed control  — smoothed acceleration magnitude drives the speed
 *                       parameter of every active segment.  Faster movement
 *                       → higher effect speed.
 *
 *   2. Shake flash    — a sudden spike in acceleration triggers a white flash
 *                       that fades back to the current effect over ~400 ms.
 *
 * Sensor data is also exported via um_data so other effects can consume it:
 *   [0] float[3]  accelXYZ  — raw x/y/z in m/s²
 *   [1] float     accelMag  — magnitude in m/s²
 *   [2] float     speedProxy — low-pass-filtered excess above 1 g (m/s²)
 *   [3] uint32_t  sampleCount — increments each read; use to detect fresh data
 *
 * Wiring (QuinLED Dig-next-2 Qwiic / Stemma QT connector):
 *   SDA → GPIO 15   SCL → GPIO 14   VCC → 3.3 V   GND → GND
 *
 * Configure I2C pins once in WLED UI → Config → Hardware → I2C/SPI, or set
 * -D I2CSDAPIN=15 -D I2CSCLPIN=14 in your platformio_override.ini so the
 * correct defaults are baked in on first boot.
 */
class LSM303AGRUsermod : public Usermod {
 private:
  Adafruit_LSM303_Accel_Unified _accel{12345};
  bool _enabled     = true;
  bool _initialized = false;

  // Sensor readings shared via um_data
  float    _accelXYZ[3]  = {0.0f, 0.0f, 0.0f};
  float    _accelMag     = 9.81f;
  float    _speedProxy   = 0.0f;
  uint32_t _sampleCount  = 0;

  um_data_t _um_data;

  // Flash burst state
  bool     _flashing      = false;
  uint32_t _flashStartMs  = 0;
  static constexpr uint32_t FLASH_DURATION_MS = 400;

  // Configurable parameters (editable in WLED Usermod Settings)
  float   _shakeThreshold = 15.0f; // m/s² total magnitude to trigger flash (~1.5 g above gravity)
  float   _filterAlpha    = 0.08f; // low-pass alpha for speed proxy (0=frozen, 1=raw)
  uint8_t _minSpeed       = 30;    // segment speed when stationary
  uint8_t _maxSpeed       = 230;   // segment speed at maximum movement
  bool    _controlSpeed   = true;
  bool    _enableFlash    = true;

  // Motion-as-audio simulation
  bool  _motionSimAudio  = false; // overwrite audioreactive um_data with motion values
  float _audioIntensity  = 1.0f;  // scale factor applied to all audio output (0.1–4.0)

  // Still detection
  float    _stillThreshold  = 0.3f;
  uint16_t _stillTimeoutMs  = 1000;
  uint32_t _lastMovementMs  = 0;

  // Bump → color
  bool    _bumpColor      = true;  // shift hue on each detected bump
  uint8_t _bumpHueShift   = 42;   // hue units to rotate per bump (0-255, 42 ≈ 60°)
  float   _colorFadeAlpha = 0.03f; // interpolation speed toward target hue (0=frozen,1=instant)
  float   _currentHue     = 0.0f;
  float   _targetHue      = 0.0f;
  bool    _colorActive    = false; // don't touch colors until first bump

  uint32_t _lastReadMs = 0;

  // PROGMEM config key strings
  static const char _name[];
  static const char _key_enabled[];
  static const char _key_shakeThresh[];
  static const char _key_ctrlSpeed[];
  static const char _key_flash[];
  static const char _key_minSpeed[];
  static const char _key_maxSpeed[];
  static const char _key_motionSim[];
  static const char _key_audioIntensity[];
  static const char _key_stillThreshold[];
  static const char _key_stillTimeout[];
  static const char _key_bumpColor[];
  static const char _key_bumpHueShift[];
  static const char _key_colorFade[];

 public:

  void setup() override {
    if (!_enabled) return;
    if (i2c_scl < 0 || i2c_sda < 0) {
      DEBUG_PRINTLN(F("LSM303AGR: I2C pins not configured"));
      return;
    }

    // Allocate um_data only once (survives config reloads)
    if (_um_data.u_size == 0) {
      _um_data.u_size = 4;
      _um_data.u_type = new um_types_t[4];
      _um_data.u_data = new void*[4];
      _um_data.u_data[0] = &_accelXYZ;   _um_data.u_type[0] = UMT_FLOAT_ARR;
      _um_data.u_data[1] = &_accelMag;   _um_data.u_type[1] = UMT_FLOAT;
      _um_data.u_data[2] = &_speedProxy; _um_data.u_type[2] = UMT_FLOAT;
      _um_data.u_data[3] = &_sampleCount;_um_data.u_type[3] = UMT_UINT32;
    }

    if (!_accel.begin()) {
      DEBUG_PRINTLN(F("LSM303AGR: sensor not found on I2C bus"));
      return;
    }

    _initialized = true;
    DEBUG_PRINTLN(F("LSM303AGR: initialised OK"));
  }

  void loop() override {
    if (!_enabled || !_initialized || strip.isUpdating()) return;

    uint32_t now = millis();
    if (now - _lastReadMs < 20) return; // ~50 Hz
    _lastReadMs = now;

    sensors_event_t event;
    _accel.getEvent(&event);

    _accelXYZ[0] = event.acceleration.x;
    _accelXYZ[1] = event.acceleration.y;
    _accelXYZ[2] = event.acceleration.z;

    float mag = sqrtf(_accelXYZ[0]*_accelXYZ[0]
                    + _accelXYZ[1]*_accelXYZ[1]
                    + _accelXYZ[2]*_accelXYZ[2]);
    _accelMag = mag;

    // Low-pass filter: track excess acceleration above 1 g (9.81 m/s²)
    float excess = (mag > 9.81f) ? (mag - 9.81f) : 0.0f;
    _speedProxy = _filterAlpha * excess + (1.0f - _filterAlpha) * _speedProxy;

    // Still detection: update timestamp whenever movement exceeds threshold
    if (excess > _stillThreshold) _lastMovementMs = now;
    bool isStill = _lastMovementMs > 0 && (now - _lastMovementMs) > (uint32_t)_stillTimeoutMs;

    _sampleCount++;

    // Shake: raw spike above threshold triggers flash and/or hue shift
    bool newBump = !isStill && mag > _shakeThreshold;
    if (newBump) {
      if (_enableFlash && !_flashing) {
        _flashing     = true;
        _flashStartMs = now;
      }
      if (_bumpColor) {
        _targetHue  += _bumpHueShift;
        if (_targetHue >= 255.0f) _targetHue -= 255.0f;
        _colorActive = true;
      }
    }

    // Speed control: map smoothed excess → minSpeed–maxSpeed, or reset when still
    if (_controlSpeed) {
      uint8_t newSpeed;
      if (isStill) {
        newSpeed = _minSpeed;
      } else {
        long excessScaled = (long)constrain(_speedProxy * 100.0f, 0.0f, 1000.0f);
        newSpeed = (uint8_t)map(excessScaled, 0L, 1000L, (long)_minSpeed, (long)_maxSpeed);
      }
      for (uint8_t s = 0; s < strip.getSegmentsNum(); s++) {
        Segment& seg = strip.getSegment(s);
        if (seg.isActive()) seg.speed = newSpeed;
      }
    }

    // Bump color: smoothly interpolate current hue toward target, apply to all segments
    if (_bumpColor && _colorActive) {
      // Shortest path around the 0-255 hue wheel
      float diff = _targetHue - _currentHue;
      if (diff >  127.5f) diff -= 255.0f;
      if (diff < -127.5f) diff += 255.0f;
      _currentHue += _colorFadeAlpha * diff;
      if (_currentHue <    0.0f) _currentHue += 255.0f;
      if (_currentHue >= 255.0f) _currentHue -= 255.0f;

      // Convert hue to RGB via FastLED CHSV
      CRGB rgb = CHSV((uint8_t)_currentHue, 255, 255);
      uint32_t newColor = RGBW32(rgb.r, rgb.g, rgb.b, 0);
      for (uint8_t s = 0; s < strip.getSegmentsNum(); s++) {
        Segment& seg = strip.getSegment(s);
        if (seg.isActive()) seg.colors[0] = newColor;
      }
    }

    // Motion-as-audio: overwrite audioreactive um_data so AR effects react to movement.
    // Runs after audioreactive's own loop(), so our values win each frame.
    // When isStill, everything is zeroed so effects go silent.
    if (_motionSimAudio) {
      um_data_t *ar;
      if (UsermodManager::getUMData(&ar, USERMOD_ID_AUDIOREACTIVE) && ar->u_size >= 6) {
        if (isStill) {
          *(float*)    ar->u_data[0] = 0.0f;
          *(uint16_t*) ar->u_data[1] = 0;
          uint8_t *fft = (uint8_t*)ar->u_data[2];
          if (fft) memset(fft, 0, 16);
          *(uint8_t*)  ar->u_data[3] = 0;
          *(float*)    ar->u_data[4] = 0.0f;
          *(float*)    ar->u_data[5] = 0.0f;
        } else {
          float sc = _audioIntensity;

          // Volume: smoothed speed proxy → 0–255
          float volSmooth = constrain(_speedProxy * sc * 20.0f, 0.0f, 255.0f);
          float volRaw    = constrain((mag - 9.81f) * sc * 15.0f, 0.0f, 255.0f);

          *(float*)    ar->u_data[0] = volSmooth;
          *(uint16_t*) ar->u_data[1] = (uint16_t)volRaw;

          // FFT bands: distribute x/y/z energy across 16 bins
          // Low  (0-4)  ← lateral (X)
          // Mid  (5-10) ← forward/back (Y)
          // High (11-15)← vertical bounce (Z minus gravity)
          uint8_t *fft = (uint8_t*)ar->u_data[2];
          if (fft) {
            uint8_t ex = (uint8_t)constrain(fabsf(_accelXYZ[0]) * sc * 26.0f, 0, 255);
            uint8_t ey = (uint8_t)constrain(fabsf(_accelXYZ[1]) * sc * 26.0f, 0, 255);
            uint8_t ez = (uint8_t)constrain(fabsf(_accelXYZ[2] - 9.81f) * sc * 26.0f, 0, 255);
            for (int i = 0;  i < 5;  i++) fft[i] = ex;
            for (int i = 5;  i < 11; i++) fft[i] = ey;
            for (int i = 11; i < 16; i++) fft[i] = ez;
          }

          *(uint8_t*)  ar->u_data[3] = _flashing ? 1 : 0;          // samplePeak = shake
          *(float*)    ar->u_data[4] = 80.0f + volSmooth * 50.0f;  // fake FFT_MajorPeak (Hz)
          *(float*)    ar->u_data[5] = volSmooth * 2.0f;            // my_magnitude
          // u_data[6] maxVol and u_data[7] binNum are user-controlled — leave them alone
        }
      }
    }
  }

  // Called every frame after effects render — blend a white flash on top
  void handleOverlayDraw() override {
    if (!_flashing) return;
    uint32_t elapsed = millis() - _flashStartMs;
    if (elapsed >= FLASH_DURATION_MS) {
      _flashing = false;
      return;
    }

    // Flash brightness fades from 255 → 0
    uint8_t flashBri = (uint8_t)(255.0f * (1.0f - (float)elapsed / (float)FLASH_DURATION_MS));

    // Raise each channel to at least flashBri (white flash overlay)
    for (uint16_t i = 0; i < strip.getLengthTotal(); i++) {
      uint32_t c = strip.getPixelColor(i);
      uint8_t  w = (c >> 24) & 0xFF;
      uint8_t  r = max((uint8_t)((c >> 16) & 0xFF), flashBri);
      uint8_t  g = max((uint8_t)((c >>  8) & 0xFF), flashBri);
      uint8_t  b = max((uint8_t)( c        & 0xFF), flashBri);
      strip.setPixelColor(i, RGBW32(r, g, b, w));
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
      user.createNestedArray("LSM303AGR").add("disabled");
      return;
    }
    if (i2c_scl < 0 || i2c_sda < 0) {
      user.createNestedArray("LSM303AGR").add("I2C not configured - set pins in Config > Hardware");
      return;
    }
    if (!_initialized) {
      user.createNestedArray("LSM303AGR").add("sensor not found on I2C bus (check wiring, addr 0x19)");
      return;
    }

    JsonArray accel = user.createNestedArray("Accel xyz").createNestedArray();
    accel.add(roundf(_accelXYZ[0] * 10.0f) / 10.0f);
    accel.add(roundf(_accelXYZ[1] * 10.0f) / 10.0f);
    accel.add(roundf(_accelXYZ[2] * 10.0f) / 10.0f);

    JsonArray mag = user.createNestedArray("Accel mag");
    mag.add(roundf(_accelMag * 10.0f) / 10.0f);
    mag.add("m/s\xb2");

    JsonArray sp = user.createNestedArray("Speed proxy");
    sp.add(roundf(_speedProxy * 10.0f) / 10.0f);
    sp.add("m/s\xb2");
  }

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_key_enabled)]    = _enabled;
    top[FPSTR(_key_shakeThresh)]= _shakeThreshold;
    top[FPSTR(_key_ctrlSpeed)]  = _controlSpeed;
    top[FPSTR(_key_flash)]      = _enableFlash;
    top[FPSTR(_key_minSpeed)]      = _minSpeed;
    top[FPSTR(_key_maxSpeed)]      = _maxSpeed;
    top[FPSTR(_key_motionSim)]      = _motionSimAudio;
    top[FPSTR(_key_audioIntensity)] = _audioIntensity;
    top[FPSTR(_key_stillThreshold)] = _stillThreshold;
    top[FPSTR(_key_stillTimeout)]   = _stillTimeoutMs;
    top[FPSTR(_key_bumpColor)]      = _bumpColor;
    top[FPSTR(_key_bumpHueShift)]   = _bumpHueShift;
    top[FPSTR(_key_colorFade)]      = _colorFadeAlpha;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[FPSTR(_name)];
    bool complete = !top.isNull();
    complete &= getJsonValue(top[FPSTR(_key_enabled)],     _enabled,         true);
    complete &= getJsonValue(top[FPSTR(_key_shakeThresh)], _shakeThreshold,  15.0f);
    complete &= getJsonValue(top[FPSTR(_key_ctrlSpeed)],   _controlSpeed,    true);
    complete &= getJsonValue(top[FPSTR(_key_flash)],       _enableFlash,     true);
    complete &= getJsonValue(top[FPSTR(_key_minSpeed)],       _minSpeed,        (uint8_t)30);
    complete &= getJsonValue(top[FPSTR(_key_maxSpeed)],       _maxSpeed,        (uint8_t)230);
    complete &= getJsonValue(top[FPSTR(_key_motionSim)],      _motionSimAudio,   false);
    complete &= getJsonValue(top[FPSTR(_key_audioIntensity)], _audioIntensity,   1.0f);
    complete &= getJsonValue(top[FPSTR(_key_stillThreshold)], _stillThreshold,   0.3f);
    complete &= getJsonValue(top[FPSTR(_key_stillTimeout)],   _stillTimeoutMs,   (uint16_t)1000);
    complete &= getJsonValue(top[FPSTR(_key_bumpColor)],      _bumpColor,        true);
    complete &= getJsonValue(top[FPSTR(_key_bumpHueShift)],   _bumpHueShift,     (uint8_t)42);
    complete &= getJsonValue(top[FPSTR(_key_colorFade)],      _colorFadeAlpha,   0.03f);
    return complete;
  }

  uint16_t getId() override { return USERMOD_ID_LSM303AGR_IMU; }
};

// Definitions are in lsm303agr_imu.cpp
