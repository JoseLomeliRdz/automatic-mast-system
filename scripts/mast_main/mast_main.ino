#include "RPLidar.h" // Libreria para controlar el sensor Lidar
#include <Servo.h> // Libreria para controlar los servos

/* Setup para Servos */
Servo servo_pan; // Definimos el servo pan
Servo servo_tilt; // Definimos el servo tilt

/* Setup para Lidar */
RPLidar lidar;
#define RPLIDAR_MOTOR 3 // Definimos el pin del motor del sensor Lidar
float minDistance = 100000;
float angleAtMinDist = 0;

/* Variables Globales */
byte comando = 0x00;
  // Variables para el modo manual
  uint8_t pan = 0, tilt = 0;
  // Variables para el modo automatico
  uint8_t tilt_rutina= 0, pan_inf = 0, pan_sup = 0;

/* Capturar los datos del lidar */
void saveData(uint16_t angle, uint16_t distance){
  Serial.print(distance);
  Serial.print(',');
  Serial.println(angle);
}

/* Leer el buffer del serial para capturar los angulos del posicionamiento manual o la rutina automatica*/
void capturarAngulos(){
  delay(10);
  int contador = 0;
  String lectura_serial[3];
  if(Serial.available() > 2){ // Si hay mas de 2 bytes en el buffer del serial los recibimos
    for(int i = 0; i < 3; i++){
    lectura_serial[i] = Serial.readStringUntil(',');
    if(lectura_serial[i] != ""){contador++;} // Separamos cada byte recibido en una posicion de la lista
    }
    if(contador == 3){ // Si se recibieron los 3 bytes de la rutina automatica se asignan a las respectivas variables
      tilt_rutina = 105 + lectura_serial[0].toInt(); // 105 es el angulo "cero" del tilt y 92 es el angulo "cero" del pan
      pan_inf = 92 + lectura_serial[1].toInt();
      pan_sup = 92 + lectura_serial[2].toInt();
    }
    else if(contador == 2){ // Si se recibieron los 2 bytes del posicionamiento manual se asignan a las respectivas variables
      tilt = 105 + lectura_serial[0].toInt();
      pan = 92 + lectura_serial[1].toInt();
    }
  }
}

/*Ejecutar el movimiento de los servos a una posicion especifica (Modo manual)*/
void posicionManual(){
  servo_pan.write(pan);
  servo_tilt.write(tilt);
}

/*Ejecutar el movimiento de los servos a una rutina especificada (Modo automatico)*/
void iniciarRutina(){ // Incompleta!!
  servo_tilt.write(tilt_rutina);
  servo_pan.write(pan_inf);
  while(comando == 0x04){
      lidarScan();
      if(servo_pan.read() < pan_sup){
        servo_pan.write(servo_pan.read() + 1);
      }
      else{
          comando = 0x00;
          lidarStop();
          homeTilt();
      }
  }
}

/*Ejecutar el movimiento de los servos a la posicion home (Tilt - 95°, Pan - 90°)*/
void homeTilt(){
  servo_pan.write(92);
  servo_tilt.write(105);
}

/*Control PID para el ascenso y descenso del mastil (POR IMPLEMENTAR)*/
void baseMastil(int task){ //Por desarrollar
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
void lidarScan(){
    if (IS_OK(lidar.waitPoint())) { //Si el sensor Lidar esta listo para leer un punto
      uint16_t distance = lidar.getCurrentPoint().distance; //Obtenemos el punto actual
      uint16_t angle = lidar.getCurrentPoint().angle;  //Obtenemos el angulo en el cual se tomo el punto
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

/*Detener el escaneo del sensor Lidar*/
void lidarStop(){
  analogWrite(RPLIDAR_MOTOR, 0);
  lidar.stop();
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
    case 0x03: // Modo Automatico: Iniciar Rutina
      iniciarRutina();
      break;
    case 0x04: // Modo Manual: Enviar
      posicionManual();
      break;
    case 0x05: // Modo Manual: Home
      homeTilt();
      break;
    case 0x06: // Modo Manual: Ascenso
      //baseMastil('a');
      break;
    case 0x07: // Modo Manual: Descenso
      //baseMastil('d');
      break;
    default:
      break;
  }
}

void setup(){
  servo_pan.attach(11);
  servo_tilt.attach(8);
  homeTilt();
  Serial.begin(460800);
  Serial2.begin(115200);
  lidar.begin(Serial2);
}

void loop(){
  interpretador(comando);
}

/*Interrupcion cada vez que se recibe datos del serial*/
void serialEvent(){ 
  while(Serial.available()){
    Serial.readBytes(&comando,1);
    if(Serial.available() >= 2){
      capturarAngulos();
    }
  }
}