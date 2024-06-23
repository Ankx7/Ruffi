#include <QTRSensors.h>

#define CANT_SENSORS             7  //cant sensores usados
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading

//----------- Pines CNY70 ----------
QTRSensorsAnalog qtra((unsigned char[]) {
  A1, A2, A3, A4, A5, A6, A7
},
CANT_SENSORS, NUM_SAMPLES_PER_SENSOR);
unsigned int sensorValues[CANT_SENSORS];

//----------- pines driver ----------
#define AIN1 4
#define AIN2 5
#define AEN 10

#define BIN1 3
#define BIN2 2
#define BEN 11

//Pines driver
int potencia = 120; //potencia recto
int potencia_a;
int potencia_b;

//----------- pin botón ----------
#define BTN 12
int est_btn;

//----------- variables PID ----------
int P = 0; // error
int I = 0; // integral
int D = 0; //derivativo
int LAST = 0; // error anterior
float vel; // velocidad

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (BIN2, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (AIN2, OUTPUT);
  pinMode (AIN1, OUTPUT);
  pinMode (AEN, OUTPUT);
  pinMode (BEN, OUTPUT);

  pinMode(BTN, INPUT_PULLUP);

  //___________ CALIBRACIÓN ______________
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);    //indica q está calibrando
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();// reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)

  }
  digitalWrite(LED_BUILTIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(115200);
  for (int i = 0; i < CANT_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < CANT_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

  //__________________ CONFIGURACIÓN MOTORES______________________
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // se pone el high y low en los pines inversos, ya que cada motor apúnta a lados opuestos
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(AEN, 0);
  analogWrite(BEN, 0);

  //___________________________________________________________________

  while (digitalRead(BTN) == HIGH);
}
void loop() {
  unsigned int position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, 1);// 0 para pista blanca con linea negra, y 1 al revez

  // IMPRESION DE VALORES
  /*
    for(unsigned char i = 0;  i <  CANT_SENSORS ; i++ ){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    }
    Serial.println(" ");
  */
  //Serial.println(position);

  //___________________________________________________________________
  P = (position - 3000);//  error
  D = P - LAST; //  derivativa
  I = (P + LAST); // constante integrativa
  //___________________________________________________________________

  if (P < 1000 && P > -1000) {
    potencia_a = potencia;
    potencia_b = potencia;
  }  else {
    potencia_a = potencia + vel ;
    potencia_b = potencia - vel ;
    vel = (P * 0.4) + (D * 0.3) + (I * 0.01 );

    if (potencia_a > potencia) {
      potencia_a = potencia;
    }
    else if (potencia_a < 0) {
      potencia_a = 0;
    }

    if (potencia_b > potencia) {
      potencia_b = potencia;
    }
    else if (potencia_b < 0) {
      potencia_b = 0;
    }
    LAST = P;
  }

  ////// formula del contol PID//////

  //vel = (P * 0.2) + (D * 0.955) + (I * 0.01 );
  //vel = (P * 0.21) + (D * 0.955) + (I * 0.01 );
  //vel = (P * 0.22) + (D * 0.95) + (I * 0.01 );
  //vel = (P * 0.24) + (D * 0.9) + (I * 0.01 );
  // vel = (P * 0.26) + (D * 0.85) + (I * 0.01 );
  // vel = (P * 0.17) + (D * 0.06) + (I * 0.01 );


  analogWrite(AEN, potencia_b);
  analogWrite(BEN, potencia_a);
}
