#include "imu_go.h"

SensorGo::Imu imu;
// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000);

  Serial.println("Beginning IMU calibration");

  //   Pass true to calibrate the sensor
  imu.init(true);
}

void loop()
{
  imu.loop();

  // Print roll pitch yaw
  String str = "rpy: ";
  str += String(imu.get_roll(), 5);
  str += "\t";
  str += String(imu.get_pitch(), 5);
  str += ", ";
  str += imu.get_yaw();

  freq_println(str, 10);
}