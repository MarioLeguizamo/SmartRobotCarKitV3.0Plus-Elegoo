/*************************************LIBRERIAS****************************/
#include <IRremote.h>       // Libreria control remoto infrarrojo
#include <Servo.h>          // Libreria servomotor

/********************** CODIGOS CONTROL REMOTE INFRARROJO *****************/
#define F 16736925	          // Adelante
#define B 16754775	          // Atras
#define L 16720605	          // Izquierda
#define R 16761405	          // Derecha
#define S 16712445	          // Alto

#define UNKNOWN_F 5316027		  // Adelante
#define UNKNOWN_B 2747854299	// Atras
#define UNKNOWN_L 1386468383	// Izquierda
#define UNKNOWN_R 553536955		// Derecha
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
#define MOTOR_IZQ_ATRAS_PIN 8	    // Motor izquierdo atras
#define MOTOR_DER_ATRAS_PIN 9	    // Motor derecho atras
#define MOTOR_DER_ADELA_PIN 11	  // Motor derecho adelante

// Pin del infrarrojo
#define INFRARROJO_PIN 12         // Infrarrojo

// Pines del sensor ultrasonico
#define ULTRASONIC_ECHO_PIN A4    // Echo
#define ULTRASONIC_TRIG_PIN A5    // Trig

// Pin del servomotor
#define SERVO_PIN 3               // Servomotor

/************************************VARIABLES****************************/
// Variables motores
const int velocidadMaxima = 255;

// Variables infrarrojo
IRrecv infrarrojo(INFRARROJO_PIN);
decode_results resultadoDecodificado;
unsigned long valorDecodificado;
unsigned long milisegundosTiempo;

// Variables sensor ultrasonico
int distanciaDerecha = 0;
int distanciaIzquierda = 0;
int distanciaMedia = 0;

// Variables servomotor
Servo servo;

/************************************FUNCIONES****************************/
// Funciones para el movimiento del robot
void adelante(int velocidad){ 
  digitalWrite(MOTOR_IZQ_ADELA_PIN,HIGH);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ADELA_PIN,HIGH);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad);
  Serial.println("go forward!");
}
void atras(int velocidad){
  digitalWrite(MOTOR_IZQ_ADELA_PIN,LOW);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ADELA_PIN,LOW);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad);
  Serial.println("go back!");
}
void izquierda(int velocidad){
  digitalWrite(MOTOR_IZQ_ADELA_PIN,LOW);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ADELA_PIN,HIGH);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad); 
  Serial.println("go left!");
}
void derecha(int velocidad){
  digitalWrite(MOTOR_IZQ_ADELA_PIN,HIGH);
  digitalWrite(MOTOR_IZQ_ATRAS_PIN,LOW);
  digitalWrite(MOTOR_DER_ATRAS_PIN,HIGH);
  digitalWrite(MOTOR_DER_ADELA_PIN,LOW);
  analogWrite(ENABLE_MOTOR_IZQ_PIN,velocidad);
  analogWrite(ENABLE_MOTOR_DER_PIN,velocidad);
  Serial.println("go right!");
}
void alto(){
  digitalWrite(ENABLE_MOTOR_IZQ_PIN, LOW);
  digitalWrite(ENABLE_MOTOR_DER_PIN, LOW);
  Serial.println("STOP!");  
}

//Ultrasonic distance measurement Sub function
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

void controlServo(uint8_t angulo)
{
  if (angulo > 175)
  {
    angulo = 175;
  }
  else if (angulo < 5)
  {
    angulo = 5;
  }
  servo.attach(SERVO_PIN);
  servo.write(angulo); //sets the servo position according to the  value
  //delay(500);
  servo.detach();
}

void evasorObstaculos() { 
    servo.write(90);  //setservo position according to scaled value
    delay(500); 
    distanciaMedia = medirDistancia();

    if(distanciaMedia <= 40) {     
      alto();
      delay(500);                         
      servo.write(10);          
      delay(1000);      
      distanciaDerecha = medirDistancia();
      
      delay(500);
      servo.write(90);              
      delay(1000);                                                  
      servo.write(180);              
      delay(1000); 
      distanciaIzquierda = medirDistancia();
      
      delay(500);
      servo.write(90);              
      delay(1000);
      if(distanciaDerecha > distanciaIzquierda) {
        derecha(velocidadMaxima);
        delay(360);
      }
      else if(distanciaDerecha < distanciaIzquierda) {
        izquierda(velocidadMaxima);
        delay(360);
      }
      else if((distanciaDerecha <= 40) || (distanciaIzquierda <= 40)) {
        atras(velocidadMaxima);
        delay(180);
      }
      else {
        adelante(velocidadMaxima);
      }
    }  
    else {
        adelante(velocidadMaxima);
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
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);    
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  //servo.attach(3,700,2400);
  alto();
  infrarrojo.enableIRIn();  
}

void loop() {
  if (infrarrojo.decode(&resultadoDecodificado)){ 
    milisegundosTiempo = millis();
    valorDecodificado = resultadoDecodificado.value;
    Serial.println(valorDecodificado);
    infrarrojo.resume();
    switch(valorDecodificado){
      case F: 
      case UNKNOWN_F: 
        adelante(velocidadMaxima); 
        break;
      case B: 
      case UNKNOWN_B: 
        atras(velocidadMaxima); 
        break;
      case L: 
      case UNKNOWN_L: 
        izquierda(velocidadMaxima); 
        break;
      case R:
      case UNKNOWN_R: 
        derecha(velocidadMaxima);
        break;
      case S: 
      case UNKNOWN_S: 
        alto(); 
        break;
      //case KEY1: 
      //case 4294967295: evasorObstaculos(); break;
      default: 
        break;
    }
  }
  else{
    if(millis() - milisegundosTiempo > 500){
      alto();
      milisegundosTiempo = millis();
    }
  }
} 
