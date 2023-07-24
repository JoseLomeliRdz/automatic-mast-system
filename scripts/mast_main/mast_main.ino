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
#define RPLIDAR_MOTOR 3 // Definimos el pin del motor del sensor Lidar
float minDistance = 100000;
float angleAtMinDist = 0;

/*Funcion MoverServo que mueve un especificado servo a un angulo especifico*/
void moverServo(uint8_t servo, uint8_t angulo) {

  int ancho_pulso_crudo, ancho_pulso_convertido;
  
  // Obtener el ancho de pulso
  ancho_pulso_crudo = map(angulo, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  ancho_pulso_convertido = int(float(ancho_pulso_crudo) / 1000000 * FREQUENCY * 4096);

  //Mandamos la señal al servo especificado
  pwmServos.setPWM(servo, 0, ancho_pulso_convertido);
}

void modoManual(uint8_t angulo_pan, uint8_t angulo_tilt){
  moverServo(servo_pan,angulo_pan);
  moverServo(servo_tilt,angulo_tilt);
}

void home(){
  moverServo(servo_pan,0);
  moverServo(servo_tilt,0);
}

void ascenso(){
 // Aqui debe ir el codigo para el ascenso del mastil
}

void descenso(){
  // Aqui debe ir el codigo para el descenso del mastil
}

void modoAuto(uint8_t tilt_inf, uint8_t tilt_sup, uint8_t pan_inf, uint8_t pan_sup){

}

void saveData(float angle, float distance){
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("   angle: ");
  Serial.println(angle);
}

void LidarScan(){
    if (IS_OK(lidar.waitPoint())) {
      //perform data processing here... 
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
      Serial.println("No detecta");
      pwmLidar.setPWM(RPLIDAR_MOTOR,0,4096);
      // Try to detect RPLIDAR
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 500))) {
        // Detected
        lidar.startScan();
        pwmLidar.setPWM(RPLIDAR_MOTOR,2048,0);
        delay(500);
      }
    }
}


void interpretador(byte comando){

}

void setup(){
  pwmServos.begin(); // Iniciar el driver PCA9685
  pwmServos.setPWMFreq(FREQUENCY); // Establecer la frecuencia del servo
  pwmServos.setOscillatorFrequency(27000000); // Establecer la frecuencia del oscilador del driver PCA9685
  
  pwmLidar.begin(); // Iniciar el driver PCA9685
  pwmLidar.setPWMFreq(1600); // Establecer la frecuencia del servo
  pwmLidar.setOscillatorFrequency(27000000); // Establecer la frecuencia del oscilador del driver PCA9685
  
  Serial.begin(460800);
  Serial2.begin(115200);
  lidar.begin(Serial2);
}

void loop(){
  LidarScan();
}