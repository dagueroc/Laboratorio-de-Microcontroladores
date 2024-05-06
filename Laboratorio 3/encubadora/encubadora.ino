#include <PID_v1.h>
#include <SPI.h>
#include <Adafruit_PCD8544.h>
#include <Adafruit_GFX.h>
#include <math.h>

// Definición de pines
int CLK           = 3;
int DIN           = 4;
int DC            = 5;
int CS            = 7;
int RST           = 6;

int BLUE          = 12;
int GREEN         = 13;
int RED           = 8;


int SWITCHP       = A5;
int SWITCHU       = A4;

int POTENCIOMETRO = A2;


int Kp = 1;
int Ki = 0.1;
int Kd = 0.1;

float simPlanta(float Q) {
    float h = 5; // Coeficiente de convección térmica para el aluminio (W/m2K)
    float Cps = 0.89; // Capacidad térmica específica del aluminio (J/g°C)
    float area = 1e-4; // Área de transferencia de calor (m2)
    float masa = 10; // Masa del bloque de aluminio (g)
    float Tamb = 25; // Temperatura ambiente (°C)
    static float T = Tamb; // Temperatura en C
    static uint32_t last_time = 0;
    uint32_t current_time = millis();
    uint32_t interval = 100; // Intervalo de tiempo entre actualizaciones (ms)

    if (current_time - last_time >= interval) {
        last_time = current_time;
        T = T + Q * interval / 1000 / masa / Cps - (T - Tamb) * area * h;
    }
    return T;
}

// Definición de variables
int Temperatura_Operacion;
double temp_ac, setPoint, ctrl_sgn, output_sgn;


// Controlador PID
PID pid(&temp_ac, &ctrl_sgn, &setPoint, Kp, Ki, Kd, DIRECT);

// Objeto PCD para conectar y escribir datos al display LCD
Adafruit_PCD8544 display = Adafruit_PCD8544(CLK,DIN,DC,CS,RST);

void setup() { 
   Serial.begin(9600);
   pinMode(CLK, OUTPUT);
   pinMode(DIN, OUTPUT); 
   pinMode(DC, OUTPUT); 
   pinMode(CS, OUTPUT);  
   pinMode(RST, OUTPUT); 
   pinMode(BLUE, OUTPUT); 
   pinMode(RED, OUTPUT);
   pinMode(GREEN, OUTPUT);
   pinMode(SWITCHU, INPUT);
   pinMode(SWITCHP, INPUT);

   Temperatura_Operacion = analogRead(POTENCIOMETRO);
   setPoint = map(Temperatura_Operacion, 0, 1023, 20, 80);
   
   output_sgn = 0;
   
  //Se limita para prevenir oscilaciones
   pid.SetOutputLimits(-100, 100);
   pid.SetMode(AUTOMATIC);

   display.begin();
} 
void loop() {

  temp_ac = simPlanta(ctrl_sgn);
  Temperatura_Operacion = analogRead(POTENCIOMETRO);
  setPoint = map(Temperatura_Operacion, 0, 1023, 20, 80);




  pid.Compute();

  

  
  display.clearDisplay();
  display.setCursor(0,0);
  // Imprime temperatura de operacion en el display
  display.print("Temp ST: "); 
  display.println(setPoint);
  
  // Imprime salida del controlador en el display
  display.print("PID sgn: "); 
  display.println(ctrl_sgn);
  
  // Imprime temperatura actual en el display
  display.print("Temp ac: ");
  display.println(temp_ac);
  
  

 if(digitalRead(SWITCHP) == HIGH){  
    display.display();
  }
   else {
    display.clearDisplay();
    display.display();
    
  }
 

  // LEDs
  if (temp_ac <= 30){
     digitalWrite(BLUE, HIGH);
  }
  else {
     digitalWrite(BLUE, LOW);
  }
  if (temp_ac >= 42){
     digitalWrite(RED, HIGH);
  }
  else {
     digitalWrite(RED, LOW);
  }
  if (temp_ac >= 30 && temp_ac <= 42){
     digitalWrite(GREEN, HIGH);

  } else{
     digitalWrite(GREEN, LOW);
  }
  // Habilitador de la comunicación con la PC
 if(digitalRead(SWITCHU) == HIGH){  
    Serial.print(temp_ac);
    Serial.print(",");
    Serial.print(setPoint);
    Serial.print(",");
    Serial.println(ctrl_sgn);
   }
   else {
    Serial.print("NULL");
    Serial.print(",");
    Serial.print("NULL");
    Serial.print(",");
    Serial.print("NULL");
    Serial.print(",");
    Serial.println("NULL");
   }
} 

//socat PTY,link=/tmp/ttyS0,raw,echo=0 PTY,link=/tmp/ttyS1,raw,echo=1