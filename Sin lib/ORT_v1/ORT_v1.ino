//+-----------------------------------------------+
//|        ORT V1 - RUFFINA qtr-7                 |
//+-----------------------------------------------+
//----------------- SENSORES ------------------
#define CANT_SENS 7
int qtr[] = { A7, A6, A5, A4, A3, A2, A1 };
int max_val[] = { 0, 0, 0, 0, 0, 0, 0 };
int min_val[] = { 2000, 2000, 2000, 2000, 2000, 2000, 2000 };
int prom_val[CANT_SENS] = {};
int samples;  //cant de muestras para la calibración

// --------- BOTON y LEd ---------
#define SW 12
#define LED_DELAY 500
bool LEDstate = 0;
unsigned int last_LED = 0;

// --------------- PD ----------------
float p = 0, d = 0, lastP = 0, elapsedTime, pd, error, timeNow = 0, lastTime = 0;
bool qtr_detect[CANT_SENS];
uint8_t count_sens = 0;
float kp = 0.32, kd = 0.01;
//float kp = 0.40, kd = 0.03;
//float kp = 0.30, kd = 0.07;
//float kp = 0.30, kd = 0.08;
//float kp = 0.35, kd = 0.08;

// --------- MOTORES ---------
#define AIN1 4
#define AIN2 5
#define AEN 10

#define BIN1 3
#define BIN2 2
#define BEN 11

uint8_t pwm;
uint8_t pwmA;
uint8_t pwmB;
uint8_t pot = 100;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AEN, OUTPUT);
  pinMode(BEN, OUTPUT);
  pinMode(SW, INPUT_PULLUP);

  Serial.begin(9600);
  for (int i = 0; i < CANT_SENS; i++) {
    pinMode(qtr[i], INPUT);
  }

  pinMode(SW, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  while (digitalRead(SW) == HIGH) {}

  //________ Calibración_________

  for (samples = 0; samples <= 3000; samples++) {  //Cada sensor tiene 5 muestras
    for (int i = 0; i < CANT_SENS; i++) {
      if (analogRead(qtr[i]) < min_val[i]) {
        min_val[i] = analogRead(qtr[i]);
      }
      if (analogRead(qtr[i]) > max_val[i]) {
        max_val[i] = analogRead(qtr[i]);
      }
      prom_val[i] = (min_val[i] + max_val[i]) / 2;
      prom_val[i] -= (prom_val[i] * 0.2);

      if (millis() - last_LED >= LED_DELAY) {  //cambia de estado cada 1000 ms
        digitalWrite(LED_BUILTIN, !LEDstate);
        LEDstate = !LEDstate;
        last_LED = millis();
      }
    }
    //Serial.println(LEDstate);
  }

  Serial.println("\nMIN");
  for (int i = 0; i < CANT_SENS; i++) {
    Serial.print("\t\tS" + String(i) + ": " + String(min_val[i]));
  }
  Serial.println("\nMAX");
  for (int i = 0; i < CANT_SENS; i++) {
    Serial.print("\t\tS" + String(i) + ": " + String(max_val[i]));
  }
  Serial.println("\nPromedio");
  for (int i = 0; i < CANT_SENS; i++) {
    Serial.print("\t\tS" + String(i) + ": " + String(prom_val[i]));
  }
  Serial.println("\n");
  digitalWrite(LED_BUILTIN, LOW);  //calibraicón terminada

  //__________________ CONFIGURACIÓN MOTORES______________________
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // se pone el high y low en los pines inversos, ya que cada motor apúnta a lados opuestos
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(AEN, 0);
  analogWrite(BEN, 0);
  while (digitalRead(SW) == HIGH) {}
  while (digitalRead(SW) == LOW) {}
  Serial.println("Start");
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  calculate_pd();
  int pwm = map(pd, -3000,3000,-255,255);
  if (pwm > pot) {
    pwm = pot;
  }
  if (pwm < -pot) {
    pwm = -pot;
  }
  pwmA = pot + pwm;
  pwmB = pot - pwm;
  if (pwmA < 0) {
    pwmA = 0;
  } else if (pwmA > 255) {
    pwmA = 255;
  }
  if (pwmB < 0) {
    pwmB = 0;
  } else if (pwmB > 255) {
    pwmB = 255;
  }

  /*
  for (int i = 0; i < CANT_SENS; i++) {
    Serial.print("S" + String(i) + "_val: " + String(qtr_detect[i]) + "      ");
  }
*/
  //Serial.print("\tpwm: " + String(pwm));
  Serial.print("MotA: " + String(pwmA) + "  MotB: " + String(pwmB) + "  ERROR: " + String(error) + "  pwm : " + String(pwm) + "  d : " + String(d * kd));
  //Serial.print("  count_sens: " + String(count_sens) + "   ERROR: " + String(error) + "   pd: " + String(pd));
  Serial.println(" ");

  analogWrite(AEN, pwmA);
  analogWrite(BEN, pwmB);
  lastP = p;
}

void pdError() {
  count_sens = 0;
  for (int i = 0; i < CANT_SENS; i++) {
    if (analogRead(qtr[i]) <= prom_val[i]) {  //si detecta blanco
      qtr_detect[i] = 1;
      count_sens++;
    } else {  //si detecta negro
      qtr_detect[i] = 0;
    }
  }
  //Serial.print("    count: " + String(count_sens) + "\n");
  if (count_sens == 0) {  //xq sino da indeterminación 0/0
    error = 0;
  } else {
    error = (-3000 * qtr_detect[0]) + (-2000 * qtr_detect[1]) + (-1000 * qtr_detect[2]) + (0 * qtr_detect[3]) + (1000 * qtr_detect[4]) + (2000 * qtr_detect[5]) + (3000 * qtr_detect[6]);  //modificar suma según cant sens
    error /= count_sens;                                                                                                                                                             //reinicio
  }
}

void calculate_pd() {
  pdError();
  lastTime = timeNow;
  timeNow = millis();
  elapsedTime = (timeNow - lastTime) / 1000;
  p = error;
  d = (p - lastP) / elapsedTime;
  //------------------- valores P y D -------------
  pd = (p * kp) + (d * kd);
  //-----------------------------------------------
}
