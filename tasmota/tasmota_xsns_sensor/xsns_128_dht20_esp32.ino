#ifdef USE_I2C
#ifdef USE_ESP32_DHT20

// State definitions for the non-blocking state machine
enum DHT20State {
    DHT20_IDLE,             // Ready to measure
    DHT20_COMMAND_SENT,     // Command sent, waiting for 80ms
    DHT20_READ_REQUIRED     // Ready to read data
};

// Define a new Sensor ID (128) and use the I2C bus ID (43)
#define XSNS_128                128
#define XI2C_43                 43      // I2C Module Index

#define DHT20_ADDR              0x38    // Fixed I2C address for DHT20
#define DHT20_MAX_SENSORS       1

// Constants for 20-bit data calculation (same as AHT series)
#define DHT_HUMIDITY_CONST      100
#define DHT_TEMPERATURE_CONST   200
#define DHT_TEMPERATURE_OFFSET  50
#define KILOBYTE_CONST          1048576.0f

#define DHT20_CMD_DELAY         20      // Delay after init/reset
#define DHT20_MEAS_TIME_MS      80      // Non-blocking wait time required by sensor
#define DHT20_RST_DELAY         10      // Delay after reset
#define DHT20_READ_INTERVAL_MS  2000    // Rate limit: Take a reading every 2 seconds

// Command set
#define DHTX_CMD                0xBE
const char dhtTypes[] PROGMEM = "DHT20";

uint8_t DHTSetCalCmd[3]      = { DHTX_CMD, 0x08, 0x00 };
uint8_t DHTMeasureCmd[3]     = { 0xAC, 0x33, 0x00 };
uint8_t DHTResetCmd          =   0xBA;

struct {
  uint8_t address = DHT20_ADDR;
  uint8_t count  = 0;

  DHT20State state = DHT20_IDLE;
  unsigned long lastMeasurementStart = 0; // When the measurement command was sent
  unsigned long lastReadTime = 0;         // When the last successful read occurred

} dht20;

struct {
  float   humidity = NAN;
  float   temperature = NAN;
  uint8_t address;
  char    types[6];
} dht20_sensors[DHT20_MAX_SENSORS];

/********************************************************************************************
 * Low-Level I2C Functions
 ********************************************************************************************/

/**
 * @brief Sends the measurement trigger command (non-blocking).
 * @param dht20_idx Sensor index.
 * @return true if command was successfully sent.
 */
bool DHT20TriggerMeasurement(uint8_t dht20_idx) {
  TwoWire &wire = I2cGetWire();
  wire.beginTransmission(dht20_sensors[dht20_idx].address);
  wire.write(DHTMeasureCmd, 3);
  if (wire.endTransmission() != 0) {
    return false;
  }
  return true;
}

/**
 * @brief Reads 6 bytes of data from the DHT20 sensor and calculates T/H values.
 * @param dht20_idx Sensor index.
 * @return true if data is valid and sensor is not busy, false otherwise.
 */
bool DHT20Read(uint8_t dht20_idx) {
  uint8_t data[6];
  TwoWire &wire = I2cGetWire();

  // Request 6 bytes
  if (wire.requestFrom(dht20_sensors[dht20_idx].address, (uint8_t) 6) != 6) {
      return false;
  }

  for(uint8_t i = 0; wire.available() && i < 6; i++) {
    data[i] = wire.read();
  }

  // Bit 7 (0x80) is Busy Status.
  if (data[0] & 0x80) {
    return false;
  }

  // Data Conversion
  uint32_t raw_humidity = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
  uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

  dht20_sensors[dht20_idx].humidity = (float)raw_humidity * DHT_HUMIDITY_CONST / KILOBYTE_CONST;
  dht20_sensors[dht20_idx].temperature = ((float)raw_temperature * DHT_TEMPERATURE_CONST / KILOBYTE_CONST) - DHT_TEMPERATURE_OFFSET;

  // Validate results
  if (!isnan(dht20_sensors[dht20_idx].temperature) && !isnan(dht20_sensors[dht20_idx].humidity) && (dht20_sensors[dht20_idx].humidity > 0)) {
      dht20.lastReadTime = millis(); // Update success time
      return true;
  }
  return false;
}

/********************************************************************************************
 * Non-Blocking State Machine
 ********************************************************************************************/

/**
 * @brief State machine to handle measurement timing without blocking delays.
 */
void DHT20Poll(void) {
  if (dht20.count == 0) return; // No sensor found

  unsigned long currentMillis = millis();
  const uint8_t sensor_idx = 0; // Only support one sensor

  switch (dht20.state) {
    case DHT20_IDLE:
      // Check if enough time has passed
      if (currentMillis - dht20.lastReadTime >= DHT20_READ_INTERVAL_MS) {
        if (DHT20TriggerMeasurement(sensor_idx)) {
          dht20.lastMeasurementStart = currentMillis;
          dht20.state = DHT20_COMMAND_SENT;
        } else {
          // Handle I2C write error
          dht20.lastReadTime = currentMillis; // Prevent immediate retry
        }
      }
      break;

    case DHT20_COMMAND_SENT:
      // Wait 80ms for measurement
      if (currentMillis - dht20.lastMeasurementStart >= DHT20_MEAS_TIME_MS) {
        dht20.state = DHT20_READ_REQUIRED;
      }
      break;

    case DHT20_READ_REQUIRED:
      // Read the data
      if (DHT20Read(sensor_idx)) {
        dht20.state = DHT20_IDLE;
      } else {
        // Read failed
        // Prevent immediate retry
        dht20.state = DHT20_IDLE;
        dht20.lastReadTime = currentMillis;
      }
      break;
  }
}

/**
 * @brief Reads the status byte of the DHT20 sensor.
 * @param dht20_address Sensor I2C address.
 * @return The status byte value.
 */
unsigned char DHT20ReadStatus(uint8_t dht20_address) {
  uint8_t result = 0;
  TwoWire &wire = I2cGetWire();
  
  wire.requestFrom(dht20_address, (uint8_t) 1);
  if (wire.available()) {
    result = wire.read();
  }
  return result;
}

/**
 * @brief Resets the DHT20 sensor via I2C command.
 * @param dht20_address Sensor I2C address.
 */
void DHT20Reset(uint8_t dht20_address) {
  TwoWire &wire = I2cGetWire();
  wire.beginTransmission(dht20_address);
  wire.write(DHTResetCmd);
  wire.endTransmission();
  
  delay(DHT20_RST_DELAY);
}

bool DHT20Init(uint8_t dht20_address) {
  TwoWire &wire = I2cGetWire();
  wire.beginTransmission(dht20_address);
  wire.write(DHTSetCalCmd, 3);
  if (wire.endTransmission() != 0)
    return false;
  delay(DHT20_CMD_DELAY); // Small blocking delay for initialization
  if(DHT20ReadStatus(dht20_address) & 0x08)
    return true;
  return false;
}

void DHT20Detect(void) {
    if (!I2cSetDevice(dht20.address)) {return;}
    if (DHT20Init(dht20.address)) {
      dht20_sensors[0].address = dht20.address;
      GetTextIndexed(dht20_sensors[0].types, sizeof(dht20_sensors[0].types), 0, dhtTypes);
      I2cSetActiveFound(dht20_sensors[0].address, dht20_sensors[0].types);
      dht20.count++;
      dht20.lastReadTime = millis();
    }
}

void DHT20Show(bool json) {
  for (uint32_t i = 0; i < dht20.count; i++) {
    float tem = ConvertTemp(dht20_sensors[i].temperature);
    float hum = ConvertHumidity(dht20_sensors[i].humidity);
    TempHumDewShow(json, true, dht20_sensors[i].types, tem, hum);
  }
}


/*********************************************************************************************
 * Interface
 *********************************************************************************************/

bool Xsns128(uint32_t function)
{
  if (!I2cEnabled(XI2C_43)) { return false; }
  bool result = false;

  if (FUNC_INIT == function) {
    DHT20Detect();
    result = true;
  }
  // Only proceed with other functions if a sensor was detected
  else if (dht20.count) {
    switch (function) {
      case FUNC_EVERY_SECOND:
        DHT20Poll();
        result = true;
        break;
      case FUNC_JSON_APPEND:
        DHT20Show(true); // JSON format
        result = true;
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        DHT20Show(false); // Web/Text format
        result = true;
        break;
#endif
    }
  }
  return result;
}

#endif  // USE_ESP32_DHT20
#endif  // USE_I2C
