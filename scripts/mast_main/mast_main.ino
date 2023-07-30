#include <Wire.h> // Libreria para comunicacion I2C
#include <Adafruit_PWMServoDriver.h> // Libreria para controlar el driver PCA9685
#include "RPLidar.h" // Libreria para controlar el sensor Lidar

/* Setup para Servos */
Adafruit_PWMServoDriver pwmServos = Adafruit_PWMServoDriver(); // Crear objeto pwm para controlar los servos mediante el driver PCA9685
//Adafruit_PWMServoDriver pwmLidar = Adafruit_PWMServoDriver(); // Crear objeto pwm para controlar el motor del lidar mediante el driver PCA9685

#define MIN_PULSE_WIDTH 650   // Definimos el ancho de pulso minimo para el pwm
#define MAX_PULSE_WIDTH 2350  // Definimos el ancho de pulso maximo para el pwm
#define FREQUENCY 50          // Definimos la frecuencia del pwm

uint8_t servo_pan = 11; // Definimos el servo pan
uint8_t servo_tilt = 8; // Definimos el servo tilt

/* Setup para Lidar */
RPLidar lidar;
#define RPLIDAR_MOTOR_EN 3 // Definimos el pin del motor del sensor Lidar
#define RPLIDAR_MOTOR_IN1 4 
float minDistance = 100000;
float angleAtMinDist = 0;

/* Variables Globales */
byte comando = 0x00;
int pan = 0, tilt = 0;

/* Capturar los datos del lidar */
void saveData(float angle, float distance){
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("   angle: ");
  Serial.println(angle);
}

/* Leer el buffer del serial para capturar los angulos del posicionamiento manual o la rutina automatica*/
void capturarAngulos(){
  int separador;
  String lectura_serial;
  char array_serial[4], pan_array[2], tilt_array[2];
  if(Serial.available() > 2){
    lectura_serial = Serial.readString();
    separador = lectura_serial.indexOf(',');
    lectura_serial.substring(0,separador).toCharArray(array_serial,4);
    if(array_serial[0] == '-'){
      tilt_array[0] = array_serial[1];
      tilt_array[1] = array_serial[2];
      tilt = atoi(tilt_array);
      tilt = tilt * -1;    
    }
    else{
      tilt = atoi(array_serial);
    }
    lectura_serial.substring(separador+1,lectura_serial.length()).toCharArray(array_serial,4);
    if(array_serial[0] == '-'){
      pan_array[0] = array_serial[1];
      pan_array[1] = array_serial[2];
      pan = atoi(tilt_array);
      pan = pan * -1;    
    }
    else{
      pan = atoi(array_serial);
    }
    pan += 95;
    tilt += 95;
  }
}

/*Mover un servo a un angulo especifico*/
void moverServo(uint8_t servo, uint8_t angulo) {

  int ancho_pulso_crudo, ancho_pulso_convertido;
  
  // Obtener el ancho de pulso
  ancho_pulso_crudo = map(angulo, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  ancho_pulso_convertido = int(float(ancho_pulso_crudo) / 1000000 * FREQUENCY * 4096);

  //Mandamos la señal al servo especificado
  pwmServos.setPWM(servo, 0, ancho_pulso_convertido);
}

/*Ejecutar el movimiento de los servos a una posicion especifica (Modo manual)*/
void posicionManual(uint8_t angulo_pan, uint8_t angulo_tilt){
  moverServo(servo_pan,angulo_pan);
  moverServo(servo_tilt,angulo_tilt);
}

/*Ejecutar el movimiento de los servos a una rutina especificada (Modo automatico)*/
void iniciarRutina(){ // Falta añadir codigo

}

/*Ejecutar el movimiento de los servos a la posicion home (Tilt - 95°, Pan - 90°)*/
void homeTilt(){
  moverServo(servo_pan,90);
  moverServo(servo_tilt,95);
}

/*Control PID para el ascenso y descenso del mastil (POR IMPLEMENTAR)*/
void baseMastil(int task){
  if(task == 'a'){
    // Ascenso
  }
  else if(task == 'd'){
    // Descenso
  }
  else{
    // Detener
  }
}

/*Escaneo de una muestra del sensor Lidar*/
void lidarScan(){// Modificar inicio Motor
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
        digitalWrite(RPLIDAR_MOTOR_IN1, HIGH);
        analogWrite(RPLIDAR_MOTOR_EN,128);
        // pwmServos.setPWM(RPLIDAR_MOTOR,2048,0);
        // delay(500);
        Serial.print("READY");
      }
    }
}

/*Detener el escaneo del sensor Lidar*/
void lidarStop(){ // Modificar inicio Motor
  // pwmServos.setPWM(RPLIDAR_MOTOR,2048,0);
  lidar.stop();
  Serial.print("STOPPED");
}

/*Envio de los datos escaneados por el Lidar al Serial*/
void lidarSave(){ // Modificar guardado de datos
  // Codigo para enviar los datos del lidar
}

/*Interpretador de comandos enviados por la HMI*/
void interpretador(byte comando){
  switch(comando){
    case 0x01: // Iniciar Escaneo
      lidarScan();
      break;
    case 0x02: // Detener Escaneo
      lidarStop();
      break;
    case 0x03: // Guardar Datos
      Serial.print(0x03);
      //lidarSave();
      break;
    case 0x04: // Modo Automatico: Iniciar Rutina
      Serial.print(0x04);
      //iniciarRutina();
      break;
    case 0x05: // Modo Manual: Enviar
      capturarAngulos();
      posicionManual(pan,tilt);
      break;
    case 0x06: // Modo Manual: Home
      homeTilt();
      break;
    case 0x07: // Modo Manual: Ascenso
      Serial.print(0x07);
      //baseMastil('a');
      break;
    case 0x08: // Modo Manual: Descenso
      Serial.print(0x08);
      //baseMastil('d');
      break;
    default:
      break;
  }
}

void setup(){
  pwmServos.begin(); // Iniciar el driver PCA9685
  pwmServos.setPWMFreq(FREQUENCY); // Establecer la frecuencia del servo
  pwmServos.setOscillatorFrequency(27000000); // Establecer la frecuencia del oscilador del driver PCA9685 para los Servos
  homeTilt();
  //pwmLidar.begin(); // Iniciar el driver PCA9685
  //pwmLidar.setPWMFreq(1600); // Establecer la frecuencia del servo
  //pwmLidar.setOscillatorFrequency(27000000); // Establecer la frecuencia del oscilador del driver PCA9685 para el Lidar
  Serial.begin(460800);
  Serial2.begin(115200);
  lidar.begin(Serial2);
  pinMode(RPLIDAR_MOTOR_EN, OUTPUT);
  pinMode(RPLIDAR_MOTOR_IN1, OUTPUT);
}

void loop(){
  Serial.readBytes(&comando,1);
  interpretador(comando);
}