#include <Wire.h> // Libreria para comunicacion I2C
#include <Adafruit_PWMServoDriver.h> // Libreria para controlar el driver PCA9685
#include "RPLidar/RPLidar.h" // Libreria para controlar el sensor Lidar

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Crear objeto pwm para controlar el driver PCA9685

#define MIN_PULSE_WIDTH 650   // Definimos el ancho de pulso minimo para el pwm
#define MAX_PULSE_WIDTH 2350  // Definimos el ancho de pulso maximo para el pwm
#define FREQUENCY 50          // Definimos la frecuencia del pwm

uint8_t servo_pan = 0; // Definimos el servo pan
uint8_t servo_tilt = 1; // Definimos el servo tilt


/*
Funcion MoverServo que mueve un especificado servo a un angulo especifico
*/
void MoverServo(uint8_t servo, uint8_t angulo) {

  int ancho_pulso_crudo, ancho_pulso_convertido;
  
  // Obtener el ancho de pulso
  ancho_pulso_crudo = map(angulo, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  ancho_pulso_convertido = int(float(ancho_pulso_crudo) / 1000000 * FREQUENCY * 4096);

  //Mandamos la señal al servo especificado
  pwm.setPWM(servo, 0, ancho_pulso_convertido);
}

void Lidar(uint8_t task){


}


void setup(){
  pwm.setPWMFreq(FREQUENCY); // Establecer la frecuencia del servo
  pwm.setOscillatorFrequency(27000000); // Establecer la frecuencia del oscilador del driver PCA9685
  pwm.begin(); // Iniciar el driver PCA9685
  delay(10);

  Serial.begin(460800);

}

void loop(){

}