"""
mock_esp32.py - Simula el ESP32 del semáforo para probar yolo_semaforo.py sin hardware.

Uso:  1) python mock_esp32.py                      (queda escuchando en el puerto 8080)
      2) python yolo_semaforo.py 0 127.0.0.1:8080  (en otra terminal; 0 = webcam)

Responde lo mismo que el ESP32 real:
  GET /estado                -> {"peatones":"ROJO","autos":"VERDE","fase":"VEH_GREEN","infracciones":0}
  GET /alerta?estado=on|off  -> imprime la alerta que manda yolo_semaforo.py

Las fases se repiten en ciclo, con los mismos colores que la máquina de estados del ESP32.
"""
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

PUERTO = 8080

# (fase, segundos, color peatones, color autos)
FASES = [
    ("VEH_GREEN", 10, "ROJO", "VERDE"),
    ("VEH_YELLOW", 2, "ROJO", "AMARILLO"),
    ("PED_WALK", 10, "VERDE", "ROJO"),
    ("PED_FLASH", 3, "VERDE", "ROJO"),
]
INICIO = time.time()
CICLO = sum(f[1] for f in FASES)
ultima_fase = None


def fase_actual():
    t = (time.time() - INICIO) % CICLO
    for nombre, duracion, peatones, autos in FASES:
        if t < duracion:
            return nombre, peatones, autos
        t -= duracion


class Manejador(BaseHTTPRequestHandler):
    def do_GET(self):
        global ultima_fase
        url = urlparse(self.path)
        if url.path == "/estado":
            fase, peatones, autos = fase_actual()
            if fase != ultima_fase:
                print(time.strftime("%H:%M:%S"), f"Fase: {fase}  (peatones {peatones}, autos {autos})")
                ultima_fase = fase
            cuerpo = json.dumps({"peatones": peatones, "autos": autos, "fase": fase, "infracciones": 0})
            tipo = "application/json"
        elif url.path == "/alerta":
            estado = parse_qs(url.query).get("estado", ["?"])[0]
            print(time.strftime("%H:%M:%S"), "ALERTA:", estado)
            cuerpo, tipo = "ALERTA " + estado, "text/plain"
        else:
            self.send_error(404)
            return
        datos = cuerpo.encode()
        self.send_response(200)
        self.send_header("Content-Type", tipo)
        self.send_header("Content-Length", str(len(datos)))
        self.end_headers()
        self.wfile.write(datos)

    def log_message(self, *args):
        pass  # sin una línea por cada consulta


print(f"ESP32 simulado en el puerto {PUERTO}. Usa: python yolo_semaforo.py 0 127.0.0.1:{PUERTO}")
HTTPServer(("0.0.0.0", PUERTO), Manejador).serve_forever()
