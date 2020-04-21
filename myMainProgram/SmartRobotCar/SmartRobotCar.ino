/*************************************LIBRERIAS****************************/
#include <IRremote.h>       // Libreria control remoto infrarrojo
#include <Servo.h>          // Libreria servomotor

#include <Wire.h>
#include "rgb_lcd.h"

/********************** CODIGOS CONTROL REMOTE INFRARROJO *****************/
#define F 16736925	          // Adelante
#define B 16754775	          // Atras
#define L 16720605	          // Izquierda
#define R 16761405	          // Derecha
#define S 16712445	          // Alto

#define UNKNOWN_F 5316027     // Adelante
#define UNKNOWN_B 2747854299	// Atras
#define UNKNOWN_L 1386468383	// Izquierda
#define UNKNOWN_R 553536955   // Derecha
#define UNKNOWN_S 3622325019	// Alto

#define KEY_1 16738455        // 1
#define KEY_2 16750695        // 2
#define KEY_3 16756815        // 3
#define KEY_4 16724175        // 4
#define KEY_5 16718055        // 5
#define KEY_6 16743045        // 6
#define KEY_7 16716015        // 7
#define KEY_8 16726215        // 8
#define KEY_9 16734885        // 9
#define KEY_0 16730805        // 0
#define KEY_STAR 16728765     // *
#define KEY_HASH 16732845     // #

/******************************ASIGNACION DE PINES************************/
// Pines de motores de desplazamiento
#define ENABLE_MOTOR_IZQ_PIN 5    // Potencia motor izquierdo
#define ENABLE_MOTOR_DER_PIN 6	  // Potencia motor derecho
#define MOTOR_IZQ_ADELA_PIN 7     // Motor izquierdo adelante
#define MOTOR_IZQ_ATRAS_PIN 8     // Motor izquierdo atras
#define MOTOR_DER_ATRAS_PIN 9     // Motor derecho atras
#define MOTOR_DER_ADELA_PIN 11	  // Motor derecho adelante

// Pin del receptor infrarrojo
#define INFRARROJO_PIN 12         // Receptor infrarrojo

// Pines del sensor ultrasonico
#define ULTRASONIC_ECHO_PIN A4    // Sensor ultrasonico receptor echo
#define ULTRASONIC_TRIG_PIN A5    // sensor ultrasonico transmisor trig

// Pin del servomotor
#define SERVO_PIN 3               // Servomotor

// Pines infrarrojos seguidor de línea
#define INFRARROJO_IZQUI_PIN 2    // Sensor infrarrojo izquierdo
#define INFRARROJO_MEDIO_PIN 4    // Sensor infrarrojo medio
#define INFRARROJO_DEREC_PIN 10   // Sensor infrarrojo derecho

/************************************VARIABLES****************************/
// Variables motores
const int velocidadMaxima = 255;

// Variables control remoto infrarrojo
IRrecv infrarrojo(INFRARROJO_PIN);
decode_results resultadoDecodificado;
//unsigned long valorDecodificado;
//unsigned long milisegundosTiempo;

// Variables sensor ultrasonico
int distanciaIzquierda = 0;
int distanciaMedia = 0;
int distanciaDerecha = 0;

// Variables servomotor
Servo servo;

// Variables sensores infrarojos
int sensorIzquierdo = 0;
int sensorMedio = 0;
int sensorDerecho = 0;

// Variables LCD RGB
rgb_lcd lcd;
const int colorR = 100;
const int colorG = 0;
const int colorB = 255;

/************************************FUNCIONES****************************/
// Funciones para el movimiento del robot
void adelante(int velocidad, int tiempoMotor) { 
  digitalWrite(MOTOR_IZQ_ADELA_PIN,HIGH);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ADELA_PIN,HIGH);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad);
  delay(tiempoMotor);
}
void atras(int velocidad, int tiempoMotor) {
  digitalWrite(MOTOR_IZQ_ADELA_PIN,LOW);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ADELA_PIN,LOW);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad);
  delay(tiempoMotor);
}
void izquierda(int velocidad, int tiempoMotor) {
  digitalWrite(MOTOR_IZQ_ADELA_PIN,LOW);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ADELA_PIN,HIGH);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad); 
  delay(tiempoMotor);
}
void derecha(int velocidad, int tiempoMotor) {
  digitalWrite(MOTOR_IZQ_ADELA_PIN,HIGH);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ADELA_PIN,LOW);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad);
  delay(tiempoMotor);
}
void alto() {
  digitalWrite(ENABLE_MOTOR_IZQ_PIN, LOW);
  digitalWrite(ENABLE_MOTOR_DER_PIN, LOW);
  Serial.println("STOP!");  
}

// Funcion para medir distancia con sensor ultrasonico
int medirDistancia() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);   
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);  
  delayMicroseconds(20);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);   
  float distancia = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);  
  distancia = distancia / 58;       
  return (int)distancia;
}

// Funcion para controlar el servomotor
void controlServo(uint8_t angulo, int tiempoServo) {
  if (angulo > 175) {
    angulo = 175;
  }
  else if (angulo < 5) {
    angulo = 5;
  }
  servo.attach(SERVO_PIN);
  servo.write(angulo);
  delay(tiempoServo);
  servo.detach();
}

// Funcion escaneo ultrasonico inicial
void escaneoUltrasonico() {
  for(int i=5;i<=175;i=i+5) {
    controlServo(i,35);
  }
  for(int i=175;i>=5;i=i-5) {
    controlServo(i,35);
  }
  controlServo(90,150);
}

// Funcion del modo evasor de obstaculos
void evasorObstaculos() { 
    controlServo(90, 500);
    distanciaMedia = medirDistancia();

    if(distanciaMedia <= 40) {     
      alto();
      delay(500);                         
      controlServo(10, 1000);      
      distanciaDerecha = medirDistancia();
      
      delay(500);
      controlServo(90, 1000);                                                  
      controlServo(180, 1000); 
      distanciaIzquierda = medirDistancia();
      
      delay(500);
      controlServo(90, 1000); 
      if(distanciaDerecha > distanciaIzquierda) {
        derecha(velocidadMaxima,360);
      }
      else if(distanciaDerecha < distanciaIzquierda) {
        izquierda(velocidadMaxima,360);
      }
      else if((distanciaDerecha <= 40) || (distanciaIzquierda <= 40)) {
        atras(velocidadMaxima,180);
      }
      else {
        adelante(velocidadMaxima,0);
      }
    }  
    else {
        adelante(velocidadMaxima,0);
    }                     
}

void leerSensores(bool debug) {
  sensorIzquierdo = digitalRead(INFRARROJO_IZQUI_PIN);
  sensorMedio = digitalRead(INFRARROJO_MEDIO_PIN);
  sensorDerecho = digitalRead(INFRARROJO_DEREC_PIN);
  if(debug) {
    Serial.print("Izquierdo: ");
    Serial.println(sensorIzquierdo);
    Serial.print("Medio    : ");
    Serial.println(sensorMedio);
    Serial.print("Derecho  : ");
    Serial.println(sensorDerecho);
    Serial.println();
  }
}

// Funcion del modo seguidor de linea
void seguidorLinea() {
  leerSensores(false);
  if(!sensorMedio){
    adelante(velocidadMaxima,0);
  }
  else if(!sensorDerecho) { 
    derecha(velocidadMaxima,0);                            
  }   
  else if(!sensorIzquierdo) {
    izquierda(velocidadMaxima,0);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_IZQ_ADELA_PIN,OUTPUT);
  pinMode(MOTOR_IZQ_ATRAS_PIN,OUTPUT);
  pinMode(MOTOR_DER_ATRAS_PIN,OUTPUT);
  pinMode(MOTOR_DER_ADELA_PIN,OUTPUT);
  pinMode(ENABLE_MOTOR_IZQ_PIN,OUTPUT);
  pinMode(ENABLE_MOTOR_DER_PIN,OUTPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(INFRARROJO_IZQUI_PIN,INPUT);
  pinMode(INFRARROJO_DEREC_PIN,INPUT);
  pinMode(INFRARROJO_MEDIO_PIN,INPUT);
  infrarrojo.enableIRIn();
  //lcd.begin(16, 2);
  //lcd.setRGB(colorR, colorG, colorB);
  //lcd.print("    ROBOTINO");
  //delay(1000);
  alto();
  controlServo(90,150);
  escaneoUltrasonico();
}

void loop() {
  //lcd.setCursor(0, 1);
  //lcd.print(millis() / 1000);
  //delay(100);
  while (infrarrojo.decode(&resultadoDecodificado)) { 
    //milisegundosTiempo = millis();
    infrarrojo.decode(&resultadoDecodificado);
    Serial.println(resultadoDecodificado.value);
    switch(resultadoDecodificado.value){
      case F: 
      case UNKNOWN_F: 
        adelante(velocidadMaxima,500);
        alto(); 
        break;
      case B: 
      case UNKNOWN_B: 
        atras(velocidadMaxima,500);
        alto();
        break;
      case L: 
      case UNKNOWN_L: 
        izquierda(velocidadMaxima,500);
        alto();
        break;
      case R:
      case UNKNOWN_R: 
        derecha(velocidadMaxima,500);
        alto();
        break;
      case S: 
      case UNKNOWN_S: 
        alto(); 
        break;
      case KEY_1:
        do {
          evasorObstaculos();
        } while(!infrarrojo.decode(&resultadoDecodificado));
        break;
      case KEY_2: 
        do {
          seguidorLinea();
        } while(!infrarrojo.decode(&resultadoDecodificado)) ;
        break;  
      default: 
        break;
    }
    infrarrojo.resume();
  }
  /*
  else {
    if(millis() - milisegundosTiempo > 500){
      alto();
      milisegundosTiempo = millis();
    }
  }*/
} 
