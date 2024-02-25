#include <imu_go.h>

namespace SensorGo
{

bool imu_initialized = false;
Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++)
  {
    if (crc & 1)
    {
      crc = (crc >> 1) ^ 0xA001;
    }
    else
    {
      crc = (crc >> 1);
    }
  }
  return crc;
}

void serial_print_motioncal(sensors_event_t& accel_event,
                            sensors_event_t& gyro_event,
                            sensors_event_t& mag_event)
{
  Serial.print("Raw:");
  Serial.print(int(accel_event.acceleration.x * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(accel_event.acceleration.y * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(accel_event.acceleration.z * 8192 / 9.8));
  Serial.print(",");
  Serial.print(int(gyro_event.gyro.x * SENSORS_RADS_TO_DPS * 16));
  Serial.print(",");
  Serial.print(int(gyro_event.gyro.y * SENSORS_RADS_TO_DPS * 16));
  Serial.print(",");
  Serial.print(int(gyro_event.gyro.z * SENSORS_RADS_TO_DPS * 16));
  Serial.print(",");
  Serial.print(int(mag_event.magnetic.x * 10));
  Serial.print(",");
  Serial.print(int(mag_event.magnetic.y * 10));
  Serial.print(",");
  Serial.print(int(mag_event.magnetic.z * 10));
  Serial.println("");
  // unified data
  Serial.print("Uni:");
  Serial.print(accel_event.acceleration.x);
  Serial.print(",");
  Serial.print(accel_event.acceleration.y);
  Serial.print(",");
  Serial.print(accel_event.acceleration.z);
  Serial.print(",");
  Serial.print(gyro_event.gyro.x, 4);
  Serial.print(",");
  Serial.print(gyro_event.gyro.y, 4);
  Serial.print(",");
  Serial.print(gyro_event.gyro.z, 4);
  Serial.print(",");
  Serial.print(mag_event.magnetic.x);
  Serial.print(",");
  Serial.print(mag_event.magnetic.y);
  Serial.print(",");
  Serial.print(mag_event.magnetic.z);
  Serial.println("");
}

// Return char array with key name for calibration data
void get_key(const char* name, char* key) { sprintf(key, "imu_%s", name); }

void write_calibration_data(imu_calibration_data_t& data, const char* name)
{
  // Open preferences with namespace "calibration"
  Preferences preferences;
  preferences.begin("imu", false);

  // Get key name
  char key[32];
  get_key(name, key);

  // Write data to preferences
  preferences.putBytes(key, &data.raw, sizeof(data));

  // Close preferences
  preferences.end();
}

imu_calibration_data_t read_calibration_data(const char* name)
{
  imu_calibration_data_t data;

  // Open preferences with namespace "calibration"
  Preferences preferences;
  preferences.begin("imu", false);

  // Get key name
  char key[32];
  get_key(name, key);

  // Read data from preferences
  preferences.getBytes(key, &data.raw, sizeof(data));

  // Close preferences
  preferences.end();

  return data;
}

Imu::Imu() : filter(0.4) {}

void Imu::init(bool should_calibrate)
{
  // TODO: figure out how to calibrate the IMU

  if (!imu_initialized)
  {
    // Start I2C bus
    Wire.begin();
    // Instantiate IMU objects
    lsm6ds.begin_I2C();
    lis3mdl.begin_I2C();
    imu_initialized = true;
  }

  // Configure IMU rates and ranges
  // Set IMU range
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  // Set IMU data rates
  lsm6ds.setAccelDataRate(LSM6DS_RATE_833_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_833_HZ);

  // Set up magnetometer
  lis3mdl.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  //   lis3mdl.setIntThreshold(500);
  //   lis3mdl.configInterrupt(true, true, true, false, false, true);

  // IMU configured
  delay(1000);

  // Calibrate IMU
  if (should_calibrate)
  {
    calibrate();
  }
  else
  {
    // Read calibration data from flash
    calibration_data = read_calibration_data("A");

    // CALIBRATION DEBUG
    Serial.printf("Accel bias: %f, %f, %f", calibration_data.accel_bias[0],
                  calibration_data.accel_bias[1],
                  calibration_data.accel_bias[2]);
    Serial.println();
    Serial.printf("Gyro bias: %f, %f, %f", calibration_data.gyro_bias[0],
                  calibration_data.gyro_bias[1], calibration_data.gyro_bias[2]);
    Serial.println();
    Serial.printf("Mag bias: %f, %f, %f", calibration_data.mag_hard_iron[0],
                  calibration_data.mag_hard_iron[1],
                  calibration_data.mag_hard_iron[2]);
    Serial.println();
    Serial.printf("Mag scale: %f, %f, %f", calibration_data.mag_soft_iron[0],
                  calibration_data.mag_soft_iron[1],
                  calibration_data.mag_soft_iron[2]);
    Serial.printf("%f %f %f", calibration_data.mag_soft_iron[3],
                  calibration_data.mag_soft_iron[4],
                  calibration_data.mag_soft_iron[5]);
    Serial.printf("%f %f %f", calibration_data.mag_soft_iron[6],
                  calibration_data.mag_soft_iron[7],
                  calibration_data.mag_soft_iron[8]);
  }

  filter.begin(500);
  last_update = micros();
}

// Default initializer
void Imu::init() { init(false); }

void Imu::read()
{
  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel_event, &gyro_event, &temp);
  lis3mdl.getEvent(&mag_event);
}

void Imu::calibrate()
{
  const int LED_PIN = 13;
  pinMode(LED_PIN, OUTPUT);

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }

  delay(1000);

  bool calibration_received = false;

  // Loop until we receive calibration data from motioncal
  size_t loopcount = 0;
  while (!calibration_received)
  {
    // Read data from IMU and print in motioncal format
    read();
    serial_print_motioncal(accel_event, gyro_event, mag_event);

    // Update calibration status
    calibration_received = receive_calibration();
    loopcount++;
  }

  // Save magnetic calibration data to struct
  calibration_data.mag_soft_iron[0] = offsets[10];
  calibration_data.mag_soft_iron[1] = offsets[13];
  calibration_data.mag_soft_iron[2] = offsets[14];
  calibration_data.mag_soft_iron[3] = offsets[13];
  calibration_data.mag_soft_iron[4] = offsets[11];
  calibration_data.mag_soft_iron[5] = offsets[15];
  calibration_data.mag_soft_iron[6] = offsets[14];
  calibration_data.mag_soft_iron[7] = offsets[15];
  calibration_data.mag_soft_iron[8] = offsets[12];

  calibration_data.mag_hard_iron[0] = offsets[6];
  calibration_data.mag_hard_iron[1] = offsets[7];
  calibration_data.mag_hard_iron[2] = offsets[8];

  // Now we need to calibrate the gyro and accel
  // Blink onboard led three times to indicate calibration starting
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }

  delay(1000);

  // Calibrate the gyro and accel
  // Loop 1000 times

  // Create 6 floats to store the running sum of the gyro and accel data
  float gyro_x_sum = 0;
  float gyro_y_sum = 0;
  float gyro_z_sum = 0;
  float accel_x_sum = 0;
  float accel_y_sum = 0;
  float accel_z_sum = 0;

  for (int i = 0; i < 1000; i++)
  {
    // Read data from IMU and print in motioncal format
    read();

    // Add gyro and accel data to running sum
    gyro_x_sum += gyro_event.gyro.x;
    gyro_y_sum += gyro_event.gyro.y;
    gyro_z_sum += gyro_event.gyro.z;

    accel_x_sum += accel_event.acceleration.x;
    accel_y_sum += accel_event.acceleration.y;
    accel_z_sum += accel_event.acceleration.z;

    // Blink onboard led to indicate calibration in progress
    digitalWrite(LED_PIN, HIGH);
    delay(1);
    digitalWrite(LED_PIN, LOW);
    delay(1);
  }

  // Calculate the average gyro and accel data
  calibration_data.gyro_bias[0] = gyro_x_sum / 1000;
  calibration_data.gyro_bias[1] = gyro_y_sum / 1000;
  calibration_data.gyro_bias[2] = gyro_z_sum / 1000;

  calibration_data.accel_bias[0] = accel_x_sum / 1000;
  calibration_data.accel_bias[1] = accel_y_sum / 1000;
  calibration_data.accel_bias[2] = accel_z_sum / 1000;

  // Save calibration data using preferences
  write_calibration_data(calibration_data, "A");
}

bool Imu::receive_calibration()
{
  byte b, i;

  while (Serial.available())
  {
    uint16_t crc;
    b = Serial.read();
    if (calcount == 0 && b != 117)
    {
      // first byte must be 117
      return false;
    }
    if (calcount == 1 && b != 84)
    {
      // second byte must be 84
      calcount = 0;
      return false;
    }
    // store this byte
    caldata[calcount++] = b;
    if (calcount < 68)
    {
      // full calibration message is 68 bytes
      return false;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i = 0; i < 68; i++)
    {
      crc = crc16_update(crc, caldata[i]);
    }
    if (crc == 0)
    {
      // data looks good, use it
      memcpy(offsets, caldata + 2, 16 * 4);

      calcount = 0;
      return true;
    }
    // look for the 117,84 in the data, before discarding
    for (i = 2; i < 67; i++)
    {
      if (caldata[i] == 117 && caldata[i + 1] == 84)
      {
        // found possible start within data
        calcount = 68 - i;
        memmove(caldata, caldata + i, calcount);
        return false;
      }
    }
    // look for 117 in last byte
    if (caldata[67] == 117)
    {
      caldata[0] = 117;
      calcount = 1;
    }
    else
    {
      calcount = 0;
    }
  }
  return false;
}

void Imu::apply_calibration()
{
  // Apply gyro bias
  gyro_event.gyro.x -= calibration_data.gyro_bias[0];
  gyro_event.gyro.y -= calibration_data.gyro_bias[1];
  gyro_event.gyro.z -= calibration_data.gyro_bias[2];

  // Change unit to degrees per second
  gyro_event.gyro.x *= SENSORS_RADS_TO_DPS;
  gyro_event.gyro.y *= SENSORS_RADS_TO_DPS;
  gyro_event.gyro.z *= SENSORS_RADS_TO_DPS;

  // Skip accel bias for now

  // Apply magnetometer calibration
  // First apply hard iron calibration
  float mx = mag_event.magnetic.x - calibration_data.mag_hard_iron[0];
  float my = mag_event.magnetic.y - calibration_data.mag_hard_iron[1];
  float mz = mag_event.magnetic.z - calibration_data.mag_hard_iron[2];

  // Then apply soft iron calibration
  mag_event.magnetic.x = mx * calibration_data.mag_soft_iron[0] +
                         my * calibration_data.mag_soft_iron[1] +
                         mz * calibration_data.mag_soft_iron[2];
  mag_event.magnetic.y = mx * calibration_data.mag_soft_iron[3] +
                         my * calibration_data.mag_soft_iron[4] +
                         mz * calibration_data.mag_soft_iron[5];
  mag_event.magnetic.z = mx * calibration_data.mag_soft_iron[6] +
                         my * calibration_data.mag_soft_iron[7] +
                         mz * calibration_data.mag_soft_iron[8];
}

void Imu::loop()
{
  // Read data from IMU and print in motioncal format
  read();

  // Apply calibration
  apply_calibration();

  float dt = (micros() - last_update) / 1000000.0f;
  filter.update(gyro_event.gyro.x, gyro_event.gyro.y, gyro_event.gyro.z,
                accel_event.acceleration.x, accel_event.acceleration.y,
                accel_event.acceleration.z, mag_event.magnetic.x,
                mag_event.magnetic.y, mag_event.magnetic.z, dt);

  last_update = micros();
}

float Imu::get_roll() { return filter.getRollRadians(); }
float Imu::get_pitch() { return filter.getPitchRadians(); }
float Imu::get_yaw() { return filter.getYawRadians(); }

void Imu::get_gravity_vector(float* grav_x, float* grav_y, float* grav_z)
{
  filter.getGravityVector(grav_x, grav_y, grav_z);
}

float Imu::get_raw_accel_x() { return accel_event.acceleration.x; }
float Imu::get_raw_accel_y() { return accel_event.acceleration.y; }
float Imu::get_raw_accel_z() { return accel_event.acceleration.z; }

float Imu::get_raw_gyro_x() { return gyro_event.gyro.x; }
float Imu::get_raw_gyro_y() { return gyro_event.gyro.y; }
float Imu::get_raw_gyro_z() { return gyro_event.gyro.z; }

float Imu::get_raw_mag_x() { return mag_event.magnetic.x; }
float Imu::get_raw_mag_y() { return mag_event.magnetic.y; }
float Imu::get_raw_mag_z() { return mag_event.magnetic.z; }

}  // namespace SensorGo