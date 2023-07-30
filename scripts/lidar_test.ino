#include "RPLidar.h"
RPLidar lidar;
#define RPLIDAR_MOTOR 3 // Definimos el pin del motor del sensor Lidar
float minDistance = 100000;
float angleAtMinDist = 0;

void saveData(float angle, float distance){
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(angle);
}

void setup(){
  Serial.begin(460800);
  Serial2.begin(115200);
  lidar.begin(Serial2);
}

void loop(){
  if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance;
      float angle = lidar.getCurrentPoint().angle;  // 0-360 deg
      if (lidar.getCurrentPoint().startBit) {
        // a new scan, display the previous data...
        saveData(angleAtMinDist, minDistance);
        minDistance = 100000;
        angleAtMinDist = 0;
      } 
      else {
        if ( distance > 0 &&  distance < minDistance) {
          minDistance = distance;
          angleAtMinDist = angle;
        }
      }
    }
    else {
      // Try to detect RPLIDAR
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 150))) {
        // Detected
        lidar.startScan();
        analogWrite(RPLIDAR_MOTOR, 200);
        delay(150);
      }
    }
}
