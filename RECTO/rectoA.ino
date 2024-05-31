#include <QTRSensors.h>

#define NUM_SENSORS             7  //cant sensores usados
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading NDEAH
//#define EMITTER_PIN             13  // LED emisor

// pines sensores de A0 a A7
QTRSensorsAnalog qtra((unsigned char[]) {
  A1, A2, A3, A4, A5, A6, A7
},
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
unsigned int sensorValues[NUM_SENSORS];
//______________________________________________________________

//----------- conexión driver ----------

int pwm_a = 10;
int pwm_b = 11;

int ain_1 = 4;
int ain_2 = 5;

int bin_2 = 2;
int bin_1 = 3;

int potencia = 120;
int potencia_a;
int potencia_b;

//______________________________________________________________

//////defino los botones de la placa//////

#define BTN 12

int est_btn;

//___________________________________________________________________
//////defino las variables del control PID//////

int P = 0; // error
int I = 0; // integral
int D = 0; //derivativo
int LAST = 0; // error anterior
float vel; // velocidad

//velocidad para la line recta
int recto_a = potencia;
int recto_b = potencia;
//___________________________________________________________________

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode (bin_2, OUTPUT);
  pinMode (bin_1, OUTPUT);


  pinMode (ain_2, OUTPUT);
  pinMode (ain_1, OUTPUT);

  pinMode(BTN, INPUT_PULLUP);


  //___________________________________________________________________
  //////calibracion//////

  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();// reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)

  }
  digitalWrite(LED_BUILTIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  //___________________________________________________________________
  ////// configuro el pwm//////

  digitalWrite(ain_1, HIGH);
  digitalWrite(ain_2, LOW);
  // se pone el high y low en los pines inversos, ya que cada motor apúnta a lados opuestos
  digitalWrite(bin_1, LOW);
  digitalWrite(bin_2, HIGH);

  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);

  //___________________________________________________________________

  while (digitalRead(BTN) == HIGH);
}
void loop() {

  unsigned int position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, 1);// 0 para pista blanca con linea negra, y 1 al revez

  // IMPRESION DE LOS VALORES
  /*
    for(unsigned char i = 0;  i <  NUM_SENSORS ; i++ ){

    Serial.print(sensorValues[i]);
    Serial.print('\t');


    }
    Serial.println(" ");
  */

  //Serial.println(position);

  //___________________________________________________________________

  P = (position - 3000);//  error
  //___________________________________________________________________

  D = P - LAST; //  derivativa
  //_______________________________
  //______________________________________
  I = (P + LAST); // constante integrativa
  //___________________________________________________________________
  
  if (P < 1000 && P > -1000) {
    potencia_a = potencia;
    potencia_b = potencia;
  }  else {
  potencia_a = recto_a + vel ;
  potencia_b = recto_b - vel ;
  vel = (P * 0.33) + (D * 0.84) + (I * 0.01 );

  /*if (potencia_a > potencia) {
    potencia_a = potencia;
  } else if (potencia_a < 0) {
    if (recto_a < potencia) {
      recto_a = recto_a + 5;
      recto_b = recto_b + 5;
    }
    potencia_a = 0;
  }

  if (potencia_b > potencia) {
    potencia_b = potencia;
  }  else if (potencia_b < 0) {
    potencia_b = 0;
  }*/
  
  LAST = P;
  }

  ////// formula del contol PID//////
  
  //vel = (P * 0.2) + (D * 0.955) + (I * 0.01 );
  //vel = (P * 0.21) + (D * 0.955) + (I * 0.01 );
  //vel = (P * 0.22) + (D * 0.95) + (I * 0.01 );
  //vel = (P * 0.24) + (D * 0.9) + (I * 0.01 );
  // vel = (P * 0.26) + (D * 0.85) + (I * 0.01 );

  // vel = (P * 0.17) + (D * 0.06) + (I * 0.01 );


  analogWrite(pwm_a, potencia_b);
  analogWrite(pwm_b, potencia_a);

}
