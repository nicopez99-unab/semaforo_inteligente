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

  CAMBIO (versión 5): integración con el programa de la cámara (yolo_semaforo.py):
    - Este ESP32 ahora tiene un servidor web (puerto 80), igual que esp32_actuador.ino:
        http://<IP_DEL_ESP32>/estado            -> {"peatones":"ROJO","autos":"VERDE",...}
        http://<IP_DEL_ESP32>/alerta?estado=on  -> enciende el LED integrado (GPIO 2) (off lo apaga)
    - La cámara lee /estado para saber de qué color está cada semáforo.
    - Como la cámara ahora graba video continuo, ya no se le piden fotos
      (PEDIR_FOTO_A_CAMARA = false). Ponlo en true si sigues usando la ESP32-CAM.

  Integración con la ESP32-CAM (versión anterior): al entrar en fase peatonal, este ESP32 le
  manda una petición HTTP a la cámara para que tome una foto. Requiere que
  la ESP32-CAM esté en el MISMO proyecto de Velxio (lienzo multi-placa),
  corriendo el sketch "esp32cam_captura.ino" — ambas se conectan solas a
  la red virtual "Velxio-GUEST" del simulador. Solo falta poner la IP que
  le haya tocado a la cámara en CAM_IP más abajo (la ves en su monitor
  serial). Si algún día pasas esto a hardware real, ahí sí cambia
  SSID_WIFI por el nombre de tu propia red y agrega la contraseña.

  Uso extra de los sensores vehiculares:
  1) Detección de infracción: si un sensor vehicular detecta algo mientras los
     autos tienen rojo (pasado un margen de gracia), suena una alarma, se pide
     una foto a la cámara y se cuenta la infracción. Hay un tiempo de espera
     entre infracciones para no disparar varias fotos por el mismo auto.
  2) Amarillo extendido: el amarillo no termina mientras un sensor siga
     detectando un auto en el cruce, hasta un tope (VEH_YELLOW_MAX).
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>   // CAMBIO: servidor web para que la cámara (Python) consulte el semáforo

// ---------- Red WiFi (la virtual de Velxio, ya lista) ----------
const char* SSID_WIFI = "A55 de Nicolás"; // red abierta del simulador, sin contraseña
const char* PASS_WIFI = "12345678";             // CAMBIO: contraseña de tu red real ("" = red abierta)

// ---------- IP de la ESP32-CAM (la ves en su monitor serial al conectar) ----------
const char* CAM_IP = "10.218.102.158"; // <-- cámbiala por la real
// CAMBIO: false = no se piden fotos (la cámara Python ya graba video continuo).
// Además así el semáforo no se congela 1,5 s cada vez que la ESP32-CAM no responde.
const bool PEDIR_FOTO_A_CAMARA = false;

// ---------- CAMBIO: servidor web para la cámara (Python) ----------
WebServer servidor(80);
const int PIN_ALERTA = 2;   // LED integrado de muchas placas ESP32 DevKit (se enciende con alerta de la cámara)
void manejarEstado();       // declaradas aquí porque setup() las usa antes de definirlas
void manejarAlerta();

// ---------- Pines: LEDs vehiculares ----------
const int V1_R = 14, V1_G = 16, V1_B = 17;
const int V2_R = 18, V2_G = 19, V2_B = 21;

// ---------- Pines: LEDs peatonales ----------
const int P1_R = 22, P1_G = 23, P1_B = 25;
const int P2_R = 26, P2_G = 27, P2_B = 32;

// ---------- Pines: sensores IR (solo entrada) ----------
const int S_VEH1 = 36;
const int S_VEH2 = 39;
const int S_PED1 = 34;
const int S_PED2 = 35;

// Nivel de la señal cuando el sensor DETECTA algo:
//   Velxio (simulador)  -> HIGH
//   FC-51 físico real   -> LOW  (su salida baja al detectar un obstáculo)
const int SENSOR_ACTIVO = LOW;

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
const unsigned long PED_WALK_TIME     = 10000;
const unsigned long PED_FLASH_TIME    = 3000;
const unsigned long PED_FLASH_INTERVAL = 400;
const unsigned long BEEP_INTERVAL_WALK  = 400;
const unsigned long BEEP_INTERVAL_FLASH = 200;

// ---------- Buzzer ----------
// true  -> buzzer pasivo (necesita una onda de frecuencia para sonar; con un nivel
//          fijo HIGH/LOW solo hace "clicks")
// false -> buzzer activo (suena solo con HIGH)
const bool BUZZER_PASIVO = true;
const int TONO_WALK  = 1500;  // Hz, fase peatonal verde
const int TONO_FLASH = 2000;  // Hz, aviso final (más agudo = más urgente)
const int TONO_ALARMA = 2500; // Hz, alarma de infracción

// ---------- Sensores vehiculares: amarillo extendido e infracciones ----------
const unsigned long VEH_YELLOW_MAX    = 4000; // tope del amarillo si sigue habiendo un auto
const unsigned long INFRACCION_GRACIA = 1000; // ms tras ponerse en rojo en que se ignora un auto que termina de pasar
const unsigned long INFRACCION_ESPERA = 5000; // ms mínimos entre una infracción y la siguiente
const unsigned long ALARMA_DURACION   = 800;  // ms que suena la alarma

unsigned long rojoInicio = 0;        // momento en que los autos pasaron a rojo
unsigned long ultimaInfraccion = 0;
bool hayInfraccionPrevia = false;
unsigned long alarmaHasta = 0;       // la alarma suena mientras millis() < alarmaHasta
int totalInfracciones = 0;

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

  buzzerInit();
  pinMode(PIN_ALERTA, OUTPUT);       // CAMBIO
  digitalWrite(PIN_ALERTA, LOW);     // CAMBIO

  estadoInicio = millis();
  Serial.println("=== Sistema de semáforos iniciado ===");

  WiFi.begin(SSID_WIFI, PASS_WIFI);   // CAMBIO: se agrega la contraseña (vacía en Velxio)
  WiFi.setSleep(false);
  Serial.print("Conectando a WiFi");
  unsigned long inicioConexion = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicioConexion < 10000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi conectado. IP propia: " + WiFi.localIP().toString());
    Serial.println("Copia esta IP en ESP32_IP de yolo_semaforo.py");   // CAMBIO
  } else {
    Serial.println();
    Serial.println("No se pudo conectar a WiFi — el semáforo seguirá funcionando sin la cámara.");
  }

  // CAMBIO: rutas del servidor web (si se conecta más tarde al WiFi, igual responde)
  servidor.on("/estado", manejarEstado);
  servidor.on("/alerta", manejarAlerta);
  servidor.begin();
}

// Le pide a la ESP32-CAM que tome una foto (no bloquea el semáforo si falla)
void tomarFotoRemota() {
  if (!PEDIR_FOTO_A_CAMARA) return;   // CAMBIO: desactivado por defecto (ver arriba)
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Sin WiFi: no se pudo avisar a la cámara");
    return;
  }
  HTTPClient http;
  String url = "http://" + String(CAM_IP) + "/capturar";
  http.begin(url);
  http.setConnectTimeout(1500); // si la cámara no responde, no congelar el semáforo
  http.setTimeout(1500);
  int codigo = http.GET();
  if (codigo > 0) {
    Serial.println("Cámara respondió: " + http.getString());
  } else {
    Serial.println("No se pudo contactar a la cámara");
  }
  http.end();
}

// ---------- CAMBIO: respuestas del servidor web para la cámara (Python) ----------
// Color del semáforo PEATONAL según el estado de la máquina
const char* colorPeatones() {
  switch (estado) {
    case PED_WALK:  return "VERDE";
    case PED_FLASH: return "VERDE";   // verde parpadeando: para la cámara sigue siendo verde
    default:        return "ROJO";    // VEH_GREEN y VEH_YELLOW
  }
}

// Color del semáforo VEHICULAR según el estado de la máquina
const char* colorAutos() {
  switch (estado) {
    case VEH_GREEN:  return "VERDE";
    case VEH_YELLOW: return "AMARILLO";
    default:         return "ROJO";   // PED_WALK y PED_FLASH
  }
}

const char* nombreFase() {
  switch (estado) {
    case VEH_GREEN:  return "VEH_GREEN";
    case VEH_YELLOW: return "VEH_YELLOW";
    case PED_WALK:   return "PED_WALK";
    default:         return "PED_FLASH";
  }
}

// GET /estado  ->  {"peatones":"ROJO","autos":"VERDE","fase":"VEH_GREEN","infracciones":0}
void manejarEstado() {
  String json = String("{\"peatones\":\"") + colorPeatones() +
                "\",\"autos\":\"" + colorAutos() +
                "\",\"fase\":\"" + nombreFase() +
                "\",\"infracciones\":" + String(totalInfracciones) + "}";
  servidor.send(200, "application/json", json);
}

// GET /alerta?estado=on (u off)  ->  la cámara avisa que la imagen se puso roja
void manejarAlerta() {
  String valor = servidor.arg("estado");
  digitalWrite(PIN_ALERTA, valor == "on" ? HIGH : LOW);
  servidor.send(200, "text/plain", "ALERTA " + valor);
  Serial.println("Cámara: alerta " + valor);
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

// En el core ESP32 3.x el tono se maneja por pin; en el 2.x, por canal LEDC
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define BUZZER_LEDC BUZZER
#else
  #define BUZZER_LEDC 0
#endif

void silencio() {
  if (BUZZER_PASIVO) ledcWriteTone(BUZZER_LEDC, 0);
  else digitalWrite(BUZZER, LOW);
}

void sonar(int frecuencia) {
  if (BUZZER_PASIVO) ledcWriteTone(BUZZER_LEDC, frecuencia);
  else digitalWrite(BUZZER, HIGH);
}

void buzzerInit() {
  if (BUZZER_PASIVO) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(BUZZER, 2000, 8);
#else
    ledcSetup(0, 2000, 8);
    ledcAttachPin(BUZZER, 0);
#endif
  } else {
    pinMode(BUZZER, OUTPUT);
  }
  silencio();
}

// Pitido intermitente no bloqueante a una frecuencia dada
// (se llama en cada loop y decide solo cuándo alternar)
void beepNoBloqueante(unsigned long intervalo, int frecuencia) {
  static unsigned long ultimoCambio = 0;
  static bool sonando = false;
  if (millis() - ultimoCambio >= intervalo) {
    sonando = !sonando;
    if (sonando) sonar(frecuencia);
    else silencio();
    ultimoCambio = millis();
  }
}

bool alarmaActiva() {
  return millis() < alarmaHasta;
}

// Enciende la alarma mientras dure y la apaga una sola vez al terminar
void actualizarAlarma() {
  static bool sonandoAlarma = false;
  if (alarmaActiva()) {
    if (!sonandoAlarma) {
      sonar(TONO_ALARMA);
      sonandoAlarma = true;
    }
  } else if (sonandoAlarma) {

    silencio();
    sonandoAlarma = false;
  }
}

// Un auto fue detectado con el semáforo vehicular en rojo
void registrarInfraccion() {
  totalInfracciones++;
  ultimaInfraccion = millis();
  hayInfraccionPrevia = true;
  alarmaHasta = millis() + ALARMA_DURACION;
  actualizarAlarma(); // que suene antes de pedir la foto
  Serial.println("¡INFRACCIÓN! Vehículo detectado con rojo. Total: " + String(totalInfracciones));
  tomarFotoRemota();
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
  servidor.handleClient();   // CAMBIO: atiende las consultas de la cámara (no bloquea)

  unsigned long ahora = millis();
  unsigned long transcurrido = ahora - estadoInicio;

  bool vehOcupado = (digitalRead(S_VEH1) == SENSOR_ACTIVO) || (digitalRead(S_VEH2) == SENSOR_ACTIVO);
  bool pedEsperando = (digitalRead(S_PED1) == SENSOR_ACTIVO) || (digitalRead(S_PED2) == SENSOR_ACTIVO);
  if (pedEsperando) pedRequest = true; // queda "guardado" aunque el sensor deje de detectar

  // Infracción: auto detectado con rojo vehicular, pasado el margen de gracia
  bool rojoVehicular = (estado == PED_WALK || estado == PED_FLASH);
  if (rojoVehicular && vehOcupado && (ahora - rojoInicio >= INFRACCION_GRACIA) &&
      (!hayInfraccionPrevia || ahora - ultimaInfraccion >= INFRACCION_ESPERA)) {
    registrarInfraccion();
  }
  actualizarAlarma();

  switch (estado) {

    case VEH_GREEN:
      setVehiculares(GREEN);
      setPeatonales(RED);
      silencio();

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
      silencio();

      // El amarillo dura al menos VEH_YELLOW_TIME, y se extiende mientras un
      // sensor vehicular siga detectando un auto, hasta VEH_YELLOW_MAX
      if (transcurrido >= VEH_YELLOW_TIME && (!vehOcupado || transcurrido >= VEH_YELLOW_MAX)) {
        Serial.println("-> Cambiando a peatonal verde");
        cambiarEstado(PED_WALK);
        rojoInicio = millis();
        tomarFotoRemota();
      }
      break;

    case PED_WALK:
      setVehiculares(RED);
      setPeatonales(GREEN);
      if (!alarmaActiva()) beepNoBloqueante(BEEP_INTERVAL_WALK, TONO_WALK);

      if (transcurrido >= PED_WALK_TIME) {
        Serial.println("-> Cambiando a aviso final peatonal");
        cambiarEstado(PED_FLASH);
      }
      break;

    case PED_FLASH:
      setVehiculares(RED);
      parpadeoPeatonalNoBloqueante();
      if (!alarmaActiva()) beepNoBloqueante(BEEP_INTERVAL_FLASH, TONO_FLASH);

      if (transcurrido >= PED_FLASH_TIME) {
        Serial.println("-> Volviendo a vehicular verde");
        pedRequest = false;
        alarmaHasta = 0; // si la alarma seguía sonando, se corta al volver el verde
        silencio();
        cambiarEstado(VEH_GREEN);
      }
      break;
  }
}
