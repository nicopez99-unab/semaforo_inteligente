/*
  Sistema de semáforos inteligentes — cruce de peatones en calle de doble sentido
  ESP32 DevKit V1

  4 LEDs RGB cátodo común:
    - 2 vehiculares (uno por sentido de la calle) -> siempre cambian juntos
    - 2 peatonales (uno por lado del cruce)        -> siempre cambian juntos

  4 sensores IR FC-51:
    - 2 vehiculares -> detectan autos pasando en cada sentido
    - 2 peatonales  -> detectan a alguien esperando para cruzar

  1 buzzer -> suena durante la fase peatonal (aviso sonoro de cruce)

  Lógica "inteligente": en verde vehicular, si algún sensor peatonal detecta
  espera, el cambio a peatonal ocurre en cuanto se cumple el tiempo mínimo Y
  no hay autos pasando — pero nunca se hace esperar al peatón más de
  VEH_GREEN_MAX, aunque sigan pasando autos.

  Integración con la ESP32-CAM: al entrar en fase peatonal, este ESP32 le
  manda una petición HTTP a la cámara para que tome una foto. Requiere que
  la ESP32-CAM esté en el MISMO proyecto de Velxio (lienzo multi-placa),
  corriendo el sketch "esp32cam_captura.ino" — ambas se conectan solas a
  la red virtual "Velxio-GUEST" del simulador. Solo falta poner la IP que
  le haya tocado a la cámara en CAM_IP más abajo (la ves en su monitor
  serial). Si algún día pasas esto a hardware real, ahí sí cambia
  SSID_WIFI por el nombre de tu propia red y agrega la contraseña.
*/

#include <WiFi.h>
#include <HTTPClient.h>

// ---------- Red WiFi (la virtual de Velxio, ya lista) ----------
const char* SSID_WIFI = "Velxio-GUEST"; // red abierta del simulador, sin contraseña

// ---------- IP de la ESP32-CAM (la ves en su monitor serial al conectar) ----------
const char* CAM_IP = "192.168.1.100"; // <-- cámbiala por la real

// ---------- Pines: LEDs vehiculares ----------
const int V1_R = 14, V1_G = 16, V1_B = 17;
const int V2_R = 18, V2_G = 19, V2_B = 21;

// ---------- Pines: LEDs peatonales ----------
const int P1_R = 22, P1_G = 23, P1_B = 25;
const int P2_R = 26, P2_G = 27, P2_B = 32;

// ---------- Pines: sensores IR (solo entrada) ----------
const int S_VEH1 = 34;
const int S_VEH2 = 35;
const int S_PED1 = 36;
const int S_PED2 = 39;

// Nivel de la señal cuando el sensor DETECTA algo:
const int SENSOR_ACTIVO = HIGH;

// ---------- Pin del buzzer ----------
const int BUZZER = 4;

// ---------- Colores posibles para un LED RGB ----------
enum Color { OFF, RED, YELLOW, GREEN };

// ---------- Estados de la máquina ----------
enum EstadoSemaforo { VEH_GREEN, VEH_YELLOW, PED_WALK, PED_FLASH };
EstadoSemaforo estado = VEH_GREEN;
unsigned long estadoInicio = 0;

// ---------- Tiempos (ms) — ajustables ----------
const unsigned long VEH_GREEN_MIN     = 5000;
const unsigned long VEH_GREEN_MAX     = 15000;
const unsigned long VEH_YELLOW_TIME   = 2000;
const unsigned long PED_WALK_TIME     = 6000;
const unsigned long PED_FLASH_TIME    = 3000;
const unsigned long PED_FLASH_INTERVAL = 400;
const unsigned long BEEP_INTERVAL_WALK  = 300;
const unsigned long BEEP_INTERVAL_FLASH = 150;

bool pedRequest = false; // se activa si un sensor peatonal detectó espera

void setup() {
  Serial.begin(115200);

  int pinesLed[] = {V1_R, V1_G, V1_B, V2_R, V2_G, V2_B,
                     P1_R, P1_G, P1_B, P2_R, P2_G, P2_B};
  for (int i = 0; i < 12; i++) pinMode(pinesLed[i], OUTPUT);

  pinMode(S_VEH1, INPUT);
  pinMode(S_VEH2, INPUT);
  pinMode(S_PED1, INPUT);
  pinMode(S_PED2, INPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  estadoInicio = millis();
  Serial.println("=== Sistema de semáforos iniciado ===");

  WiFi.begin(SSID_WIFI);
  Serial.print("Conectando a WiFi");
  unsigned long inicioConexion = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicioConexion < 10000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi conectado. IP propia: " + WiFi.localIP().toString());
  } else {
    Serial.println();
    Serial.println("No se pudo conectar a WiFi — el semáforo seguirá funcionando sin la cámara.");
  }
}

// Le pide a la ESP32-CAM que tome una foto (no bloquea el semáforo si falla)
void tomarFotoRemota() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Sin WiFi: no se pudo avisar a la cámara");
    return;
  }
  HTTPClient http;
  String url = "http://" + String(CAM_IP) + "/capturar";
  http.begin(url);
  int codigo = http.GET();
  if (codigo > 0) {
    Serial.println("Cámara respondió: " + http.getString());
  } else {
    Serial.println("No se pudo contactar a la cámara");
  }
  http.end();
}

// Aplica un color a un LED RGB específico (solo usamos R y G; B queda libre)
void setLed(int rPin, int gPin, int bPin, Color c) {
  digitalWrite(rPin, (c == RED || c == YELLOW) ? HIGH : LOW);
  digitalWrite(gPin, (c == GREEN || c == YELLOW) ? HIGH : LOW);
  digitalWrite(bPin, LOW);
}

void setVehiculares(Color c) {
  setLed(V1_R, V1_G, V1_B, c);
  setLed(V2_R, V2_G, V2_B, c);
}

void setPeatonales(Color c) {
  setLed(P1_R, P1_G, P1_B, c);
  setLed(P2_R, P2_G, P2_B, c);
}

// Cambia de estado y reinicia el cronómetro de esa fase
void cambiarEstado(EstadoSemaforo nuevo) {
  estado = nuevo;
  estadoInicio = millis();
}

// Buzzer parpadeante no bloqueante (se llama cada loop, decide solo cuándo alternar)
void beepNoBloqueante(unsigned long intervalo) {
  static unsigned long ultimoCambio = 0;
  static bool sonando = false;
  if (millis() - ultimoCambio >= intervalo) {
    sonando = !sonando;
    digitalWrite(BUZZER, sonando ? HIGH : LOW);
    ultimoCambio = millis();
  }
}

// Parpadeo del peatonal en la fase de aviso final (no bloqueante)
void parpadeoPeatonalNoBloqueante() {
  static unsigned long ultimoCambio = 0;
  static bool encendido = true;
  if (millis() - ultimoCambio >= PED_FLASH_INTERVAL) {
    encendido = !encendido;
    setPeatonales(encendido ? GREEN : OFF);
    ultimoCambio = millis();
  }
}

void loop() {
  unsigned long ahora = millis();
  unsigned long transcurrido = ahora - estadoInicio;

  bool vehOcupado = (digitalRead(S_VEH1) == SENSOR_ACTIVO) || (digitalRead(S_VEH2) == SENSOR_ACTIVO);
  bool pedEsperando = (digitalRead(S_PED1) == SENSOR_ACTIVO) || (digitalRead(S_PED2) == SENSOR_ACTIVO);
  if (pedEsperando) pedRequest = true; // queda "guardado" aunque el sensor deje de detectar

  switch (estado) {

    case VEH_GREEN:
      setVehiculares(GREEN);
      setPeatonales(RED);
      digitalWrite(BUZZER, LOW);

      if (transcurrido >= VEH_GREEN_MIN) {
        bool puedeCambiar = pedRequest && !vehOcupado;
        bool tiempoMaximo = transcurrido >= VEH_GREEN_MAX;
        if (puedeCambiar || tiempoMaximo) {
          Serial.println("-> Cambiando a amarillo vehicular");
          cambiarEstado(VEH_YELLOW);
        }
      }
      break;

    case VEH_YELLOW:
      setVehiculares(YELLOW);
      setPeatonales(RED);
      digitalWrite(BUZZER, LOW);

      if (transcurrido >= VEH_YELLOW_TIME) {
        Serial.println("-> Cambiando a peatonal verde");
        cambiarEstado(PED_WALK);
        tomarFotoRemota();
      }
      break;

    case PED_WALK:
      setVehiculares(RED);
      setPeatonales(GREEN);
      beepNoBloqueante(BEEP_INTERVAL_WALK);

      if (transcurrido >= PED_WALK_TIME) {
        Serial.println("-> Cambiando a aviso final peatonal");
        cambiarEstado(PED_FLASH);
      }
      break;

    case PED_FLASH:
      setVehiculares(RED);
      parpadeoPeatonalNoBloqueante();
      beepNoBloqueante(BEEP_INTERVAL_FLASH);

      if (transcurrido >= PED_FLASH_TIME) {
        Serial.println("-> Volviendo a vehicular verde");
        pedRequest = false;
        digitalWrite(BUZZER, LOW);
        cambiarEstado(VEH_GREEN);
      }
      break;
  }
}
