/*
  Código de PRUEBA de cableado — sistema de semáforos inteligentes
  ESP32 DevKit V1

  Objetivo: verificar que los 4 LEDs RGB, los 4 sensores IR y el buzzer
  están bien conectados, ANTES de programar la lógica real del semáforo.
  No es el código final (ese usará una máquina de estados no bloqueante).

  Qué hace:
  - Enciende cada LED RGB en rojo, luego verde, luego azul, uno a la vez.
  - Lee los 4 sensores IR continuamente y muestra su estado por Serial.
  - Hace sonar el buzzer brevemente al terminar cada ciclo de LEDs.
*/

// --- Pines de los 4 LEDs RGB (cátodo común) ---
const int LED1_R = 14, LED1_G = 16, LED1_B = 17;
const int LED2_R = 18, LED2_G = 19, LED2_B = 21;
const int LED3_R = 22, LED3_G = 23, LED3_B = 25;
const int LED4_R = 26, LED4_G = 27, LED4_B = 32;

// Agrupamos los 4 LEDs en arreglos para recorrerlos fácil con un for
const int ledsR[4] = {LED1_R, LED2_R, LED3_R, LED4_R};
const int ledsG[4] = {LED1_G, LED2_G, LED3_G, LED4_G};
const int ledsB[4] = {LED1_B, LED2_B, LED3_B, LED4_B};

// --- Pines de los 4 sensores IR (solo entrada) ---
const int SENSOR1 = 34;
const int SENSOR2 = 35;
const int SENSOR3 = 36;
const int SENSOR4 = 39;
const int sensores[4] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4};

// --- Pin del buzzer (módulo con SIG) ---
const int BUZZER = 4;

void setup() {
  Serial.begin(115200);

  // Configurar los 12 pines de LEDs como salida
  for (int i = 0; i < 4; i++) {
    pinMode(ledsR[i], OUTPUT);
    pinMode(ledsG[i], OUTPUT);
    pinMode(ledsB[i], OUTPUT);
  }

  // Configurar los 4 pines de sensores como entrada
  for (int i = 0; i < 4; i++) {
    pinMode(sensores[i], INPUT);
  }

  // Buzzer como salida
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  Serial.println("=== Prueba de cableado iniciada ===");
}

// Apaga los 3 colores de un LED específico (por índice 0-3)
void apagarLed(int i) {
  digitalWrite(ledsR[i], LOW);
  digitalWrite(ledsG[i], LOW);
  digitalWrite(ledsB[i], LOW);
}

// Enciende solo un color de un LED específico
void encenderColor(int i, int pinColor) {
  apagarLed(i);
  digitalWrite(pinColor, HIGH);
}

void probarSensores() {
  Serial.print("Sensores -> ");
  for (int i = 0; i < 4; i++) {
    int estado = digitalRead(sensores[i]);
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(estado == HIGH ? "detectado" : "libre");
    Serial.print("  ");
  }
  Serial.println();
}

void beepBuzzer() {
  digitalWrite(BUZZER, HIGH);
  delay(150);
  digitalWrite(BUZZER, LOW);
}

void loop() {
  // Recorre los 4 LEDs, uno a la vez, probando sus 3 colores
  for (int i = 0; i < 4; i++) {
    Serial.print("LED ");
    Serial.print(i + 1);
    Serial.println(" -> ROJO");
    encenderColor(i, ledsR[i]);
    probarSensores();
    delay(800);

    Serial.print("LED ");
    Serial.print(i + 1);
    Serial.println(" -> VERDE");
    encenderColor(i, ledsG[i]);
    probarSensores();
    delay(800);

    Serial.print("LED ");
    Serial.print(i + 1);
    Serial.println(" -> AZUL");
    encenderColor(i, ledsB[i]);
    probarSensores();
    delay(800);

    apagarLed(i);
  }

  Serial.println("--- Ciclo completo, beep ---");
  beepBuzzer();
  delay(500);
}
