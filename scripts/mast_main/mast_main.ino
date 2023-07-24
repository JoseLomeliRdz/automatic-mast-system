#include <Wire.h> // Libreria para comunicacion I2C
#include <Adafruit_PWMServoDriver.h> // Libreria para controlar el driver PCA9685
#include "RPLidar.h" // Libreria para controlar el sensor Lidar

Adafruit_PWMServoDriver pwmServos = Adafruit_PWMServoDriver(); // Crear objeto pwm para controlar los servos mediante el driver PCA9685
Adafruit_PWMServoDriver pwmLidar = Adafruit_PWMServoDriver(); // Crear objeto pwm para controlar el motor del lidar mediante el driver PCA9685

#define MIN_PULSE_WIDTH 650   // Definimos el ancho de pulso minimo para el pwm
#define MAX_PULSE_WIDTH 2350  // Definimos el ancho de pulso maximo para el pwm
#define FREQUENCY 50          // Definimos la frecuencia del pwm

uint8_t servo_pan = 0; // Definimos el servo pan
uint8_t servo_tilt = 1; // Definimos el servo tilt

RPLidar lidar;
#define RPLIDAR_MOTOR 2 // Definimos el pin del motor del sensor Lidar
bool lidar_ready = false; // Variable para saber si el sensor Lidar esta listo

int time = 0; // Variable para medir el tiempo

/*Funcion MoverServo que mueve un especificado servo a un angulo especifico*/
void MoverServo(uint8_t servo, uint8_t angulo) {

  int ancho_pulso_crudo, ancho_pulso_convertido;
  
  // Obtener el ancho de pulso
  ancho_pulso_crudo = map(angulo, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  ancho_pulso_convertido = int(float(ancho_pulso_crudo) / 1000000 * FREQUENCY * 4096);

  //Mandamos la señal al servo especificado
  pwmServos.setPWM(servo, 0, ancho_pulso_convertido);
}

void LidarInitialize(){
  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 200))) {
    lidar.startScan();
    pwmLidar.setPWM(RPLIDAR_MOTOR, 0, 4096);
    lidar_ready = true;
  }
  else{
    Serial.println("Lidar not detected");
    lidar_ready = false;
  }
}

void LidarScan(uint8_t task){
  if (task == 1){
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
      Serial.write(' ');
      Serial.print(distance);
      Serial.write(' ');
      Serial.print(angle);
      Serial.write(' ');
      Serial.print(startBit);
      Serial.write(' ');
      Serial.println(quality);
    }
  } 
  else if (task == 0){
    pwmLidar.setPWM(RPLIDAR_MOTOR, 0, 0);
    lidar.stop();
  }
  
}


void setup(){
   pwmServos.setPWMFreq(FREQUENCY); // Establecer la frecuencia del servo
   pwmServos.setOscillatorFrequency(27000000); // Establecer la frecuencia del oscilador del driver PCA9685
   pwmServos.begin(); // Iniciar el driver PCA9685

   pwmLidar.setPWMFreq(1600); // Establecer la frecuencia del servo
   pwmLidar.setOscillatorFrequency(27000000); // Establecer la frecuencia del oscilador del driver PCA9685
   pwmLidar.begin(); // Iniciar el driver PCA9685
   delay(1000);
   
   lidar.begin(Serial2);

  Serial.begin(460800);
  
  while(lidar_ready == false){
    LidarInitialize();
  }
}

void loop(){
  LidarScan(1);
}