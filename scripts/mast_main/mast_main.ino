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
uint8_t time = 0;
byte comando = 0x00;
/*Funcion MoverServo que mueve un especificado servo a un angulo especifico*/
void moverServo(uint8_t servo, uint8_t angulo) {

  int ancho_pulso_crudo, ancho_pulso_convertido;
  
  // Obtener el ancho de pulso
  ancho_pulso_crudo = map(angulo, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  ancho_pulso_convertido = int(float(ancho_pulso_crudo) / 1000000 * FREQUENCY * 4096);

  //Mandamos la señal al servo especificado
  pwmServos.setPWM(servo, 0, ancho_pulso_convertido);
}

void posicionManual(uint8_t angulo_pan, uint8_t angulo_tilt){
  moverServo(servo_pan,angulo_pan);
  moverServo(servo_tilt,angulo_tilt);
}

void homeTilt(){
  moverServo(servo_pan,0);
  moverServo(servo_tilt,0);
}

void ascensoMastil(){
 // Aqui debe ir el codigo para el ascenso del mastil
}

void descensoMastil(){
  // Aqui debe ir el codigo para el descenso del mastil
}

void iniciarRutina(){

}

void saveData(float angle, float distance){
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("   angle: ");
  Serial.println(angle);
}

void lidarScan(){
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
      if (IS_OK(lidar.getDeviceInfo(info, 500))) {
        // Detected
        lidar.startScan();
        pwmLidar.setPWM(RPLIDAR_MOTOR,2048,0);
        delay(500);
        Serial.print("READY");
      }
    }
}

void lidarStop(){
  pwmLidar.setPWM(RPLIDAR_MOTOR,0,4096);
  lidar.stop();
  Serial.print("STOPPED");
}

void lidarSave(){
  // Codigo para enviar los datos del lidar
}

void interpretador(byte comando){
  switch(comando){
    case 0x01:
      lidarScan();
      break;
    case 0x02:
      lidarStop();
      break;
    case 0x03:
      Serial.print(0x03);
      //lidarSave();
      break;
    case 0x04:
      Serial.print(0x04);
      //iniciarRutina();
      break;
    case 0x05:
      Serial.print(0x05);
      //posicionManual();
      break;
    case 0x06:
      Serial.print(0x06);
      //homeTilt();
      break;
    case 0x07:
      Serial.print(0x07);
      //ascensoMastil();
      break;
    case 0x08:
      Serial.print(0x08);
      //descensoMastil();
      break;
    default:
      Serial.println("0x00");
      break;
  }
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
  Serial.readBytes(&comando,1);
  interpretador(comando);
}