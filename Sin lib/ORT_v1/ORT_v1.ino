//+-----------------------------------------------+
//|                      ORT V1                   |
//+-----------------------------------------------+
//----------------- SENSORES ------------------
#define CANT_SENS 8
uint8_t qtr[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
int max_val[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int min_val[] = { 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000 };
int prom_val[CANT_SENS] = {};
int samples; //cant de muestras para la calibración

// --------- BOTON y LED ---------
#define SW 12
#define LED_DELAY 1000
bool LEDstate = 0;
unsigned int last_LED = 0;

//----------------- MOTORES ------------------
#define MIA 4
#define MIB 5
#define MIE 10

#define MDA 2
#define MDB 3
#define MDE 11

int recto = 140;
int pot = 120;
int pwmI, pwmD;

// --------------- PID ----------------
int P = 0;
int I = 0;
int D = 0;
int LAST = 0;  // error anterior
float PID;

void setup() {
  for (int i = 0; i < CANT_SENS; i++) {
    pinMode(qtr[i], INPUT);
  }

  pinMode(SW, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MIA, OUTPUT);
  pinMode(MIB, OUTPUT);
  pinMode(MIE, OUTPUT);
  pinMode(MDA, OUTPUT);
  pinMode(MDB, OUTPUT);
  pinMode(MDE, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  while (digitalRead(SW) == HIGH) {}

  //________ Calibración_________
  samples = 0;
  while (samples <= 5) {            //Cada sensor tiene 5 muestras
    if (millis() - last_LED >= LED_DELAY) {  //cambia de estado cada 1000 ms
      digitalWrite(LED_BUILTIN, !LEDstate);
      LEDstate = !LEDstate;
    }
    for (int i = 0; i < CANT_SENS; i++) {
      if (analogRead(qtr[i]) < min_val[i]) {
        min_val[i] = analogRead(qtr[i]);
      }
      if (analogRead(qtr[i]) > max_val[i]) {
        max_val[i] = analogRead(qtr[i]);
      }
      prom_val[i] = (min_val[i] + max_val[i])/2;
    }
    samples++;
  }

  Serial.println("\nMIN");
  for (int i = 0; i < CANT_SENS; i++) {
    Serial.println("\tS" + String(i) + ": " + String(min_val[i]));
  }
  Serial.println("\nMAX");
  for (int i = 0; i < CANT_SENS; i++) {
    Serial.println("\tS" + String(i) + ": " + String(max_val[i]));
  }
  Serial.println("\nPromedio");
  for (int i = 0; i < CANT_SENS; i++) {
    Serial.println("\tS" + String(i) + ": " + String(prom_val[i]));
  }
  Serial.println(" ");
  
  digitalWrite(LED_BUILTIN, LOW);

  while (digitalRead(SW) == HIGH) {}
  Serial.println("Start");
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
switch
}
