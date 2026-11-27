"""
yolo_semaforo.py - YOLO en tiempo real + semáforo ESP32 + grabación automática.
(Une en un solo archivo 05_yolo_camara.py y camara.py)

Uso:  python yolo_semaforo.py                          (webcam por defecto)
      python yolo_semaforo.py http://IP:81/stream      (ESP32-CAM o celular)
      python yolo_semaforo.py 0 192.168.1.50           (2º argumento opcional: IP del ESP32 del semáforo)

Ventanas que se abren:
  - "YOLO en tiempo real"  -> la cámara. Se cierra con la X (QUIT) o con Alt + F4.
  - "Bloc de notas"        -> comandos y estadísticas (solo lectura).
                              Cerrar ESTA ventana termina el programa completo.
Teclas (con cualquiera de las dos ventanas activa):
  - Control + G   -> guarda los últimos 5 minutos de grabación.
  - Control + F5  -> refresca la cámara, o la vuelve a abrir si estaba cerrada.

Si existe la carpeta yolo26n_ncnn_model (ver 06_exportar_ncnn.py) la usa.

Requisitos: pip install ultralytics opencv-python pillow
            (en Raspberry Pi además: sudo apt install python3-tk)
"""
import sys, os, time, json, glob, shutil, platform, threading, queue
import urllib.request
from datetime import datetime
import tkinter as tk

import cv2
import numpy as np
from PIL import Image, ImageTk
from ultralytics import YOLO


# ═════════════════════════════════════════════════════════════════════════════
#  CONFIGURACIÓN GENERAL
# ═════════════════════════════════════════════════════════════════════════════
TAMANO = 640                    # 320 = más rápido · 640 = más preciso (y detecta mejor objetos PEQUEÑOS o lejanos)

# ─────────────────────────────────────────────────────────────────────────────
#  [PUNTO 4] CLASES A DETECTAR Y AJUSTE DE TAMAÑO
# ─────────────────────────────────────────────────────────────────────────────
CLASES_PERSONA = {"person"}
CLASES_AUTO = {"car"}            # puedes agregar "truck", "bus", "motorcycle" si también quieres contarlos como autos

# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║ >>> AQUÍ SE AJUSTA EL TAMAÑO CON QUE LA CÁMARA REPRESENTA/ACEPTA OBJETOS <<<║
# ║ Medidas en PÍXELES de la caja que dibuja YOLO alrededor del objeto.       ║
# ║ - Si los autos se ven más PEQUEÑOS (están lejos), baja "min_ancho" y      ║
# ║   "min_alto" de TAMANO_AUTO (y sube TAMANO de arriba a 640).               ║
# ║ - Si aparecen detecciones falsas muy chicas, sube los mínimos.            ║
# ║ - "max_..." sirve para ignorar cajas gigantes (por ejemplo, algo pegado   ║
# ║   al lente). 9999 = sin límite.                                           ║
# ╚═══════════════════════════════════════════════════════════════════════════╝
TAMANO_PERSONA = {"min_ancho": 10, "min_alto": 25, "max_ancho": 9999, "max_alto": 9999}
TAMANO_AUTO    = {"min_ancho": 20, "min_alto": 15, "max_ancho": 9999, "max_alto": 9999}
CONF_PERSONA = 0.25               # confianza mínima para aceptar una persona (0 a 1)
CONF_AUTO    = 0.25               # confianza mínima para aceptar un auto (más baja = detecta autos pequeños, pero más errores)
GROSOR_CAJA  = None              # grosor de las cajas dibujadas (None = automático, o un número como 1, 2, 3)

# ─────────────────────────────────────────────────────────────────────────────
#  [PUNTO 5 y 7] ZONA (COORDENADAS) QUE HACE QUE LA PANTALLA SE PONGA ROJA
# ─────────────────────────────────────────────────────────────────────────────
# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║ >>> AQUÍ SE MODIFICAN LAS COORDENADAS DE LA ZONA <<<                      ║
# ║ Rectángulo en PÍXELES de la imagen: esquina (X1, Y1) hasta (X2, Y2).      ║
# ║ (0, 0) es la esquina SUPERIOR IZQUIERDA; X crece a la derecha, Y hacia    ║
# ║ abajo. Ejemplo tuyo: x=1000 y=2000 hasta x=1100 y=2200                    ║
# ║   -> ZONA_X1, ZONA_Y1, ZONA_X2, ZONA_Y2 = 1000, 2000, 1100, 2200          ║
# ║ OJO: las coordenadas deben caber en la resolución de tu cámara (al abrir  ║
# ║ se imprime la resolución en la terminal; una webcam típica es 640x480).   ║
# ╚═══════════════════════════════════════════════════════════════════════════╝
ZONA_X1, ZONA_Y1, ZONA_X2, ZONA_Y2 = 200, 150, 440, 320

# Qué punto del objeto debe estar dentro de la zona:
#   "abajo"  -> el centro de la base de la caja (los pies de la persona / las ruedas del auto). Recomendado.
#   "centro" -> el centro de la caja.
#   "caja"   -> basta con que cualquier parte de la caja toque la zona.
PUNTO_DETECCION = "abajo"
INTENSIDAD_ROJO = 0.30           # qué tan roja se pone la imagen (0.0 nada · 1.0 rojo total)
TOLERANCIA_SALIDA_SEG = 0.5      # segundos sin el objeto en la zona antes de volver a la normalidad
                                 # (evita parpadeos si YOLO pierde el objeto un instante; 0 = inmediato)
MOSTRAR_ZONA = True              # dibuja el rectángulo de la zona en la imagen

# ─────────────────────────────────────────────────────────────────────────────
#  SEMÁFORO (ESP32)
# ─────────────────────────────────────────────────────────────────────────────
ESP32_IP = None        # <-- IP que el ESP32 del semáforo muestra en su monitor serial
                                 #     None = trabajar sin ESP32 (usa los valores manuales de abajo)
SEMAFORO_MANUAL_PEATONES = "ROJO"    # solo se usan si ESP32_IP = None (para hacer pruebas)
SEMAFORO_MANUAL_AUTOS = "VERDE"      # valores posibles: "ROJO", "AMARILLO", "VERDE"
PERIODO_CONSULTA_SEG = 0.3       # cada cuánto se le pregunta el estado al ESP32
AVISAR_ALERTA_AL_ESP32 = True    # enciende el LED integrado (GPIO 2) del ESP32 mientras hay alerta

# ─────────────────────────────────────────────────────────────────────────────
#  [PUNTO 9] GRABACIÓN
# ─────────────────────────────────────────────────────────────────────────────
CARPETA_TEMPORAL = "grabacion_temporal"     # grabación continua (se borra cada 30 min)
CARPETA_GUARDADAS = "grabaciones_guardadas" # grabaciones guardadas (Ctrl+G y alertas)
MANUAL_SEG = 5 * 60              # Ctrl + G guarda los últimos 5 minutos
BLOQUE_SEG = 30 * 60             # cada 30 minutos se corta y se borra la grabación
ANTES_ALERTA_SEG = 30            # una alerta guarda desde 30 s ANTES de ponerse roja...
DESPUES_ALERTA_SEG = 10          # ...hasta 10 s DESPUÉS de volver a la normalidad
SEGMENTO_SEG = 10                # la grabación continua se guarda en trozos de 10 s (para poder cortarla)
CONSERVAR_AL_CORTAR_SEG = 300    # CAMBIO: al cortar cada 30 min se conservan los últimos 5 min para que
                                 # Ctrl+G y los 30 s previos de una alerta funcionen justo después del corte.
                                 # Pon 0 si quieres que se borre absolutamente todo.
GRABAR_ANOTADO = True            # True = graba la imagen con cajas/zona/rojo · False = graba la imagen original
ESCALA_VENTANA = 1.0             # tamaño de la ventana de la cámara (0.5 = mitad, 2 = doble)


# ═════════════════════════════════════════════════════════════════════════════
#  CÁMARA  (contenido de camara.py)
# ═════════════════════════════════════════════════════════════════════════════
# Fuentes aceptadas (el parámetro "fuente" o el argumento en la terminal):
#     (nada)                      -> webcam por defecto (o la cámara de la Raspberry Pi si existe)
#     0, 1, 2 ...                 -> otra webcam USB / integrada
#     http://IP:81/stream         -> ESP32-CAM (ejemplo CameraWebServer)
#     http://IP:8080/video        -> celular usado como cámara (app "IP Webcam" u otra similar)
#     video.mp4                   -> un archivo de video (si no tienes cámara)
#     picam                       -> Camera Module de la Raspberry Pi

def _picamera(ancho, alto):
    from picamera2 import Picamera2
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(
        main={"format": "RGB888", "size": (ancho, alto)}))
    cam.start()
    print("Cámara: Raspberry Pi Camera Module")
    # Con RGB888, Picamera2 entrega los bytes en orden BGR: listo para OpenCV
    leer = lambda: cam.capture_array()
    leer.cerrar = lambda: (cam.stop(), cam.close())     # CAMBIO: permite cerrar la cámara (para Ctrl+F5 / X)
    return leer


def abrir_camara(fuente=None, ancho=640, alto=480):
    if fuente == "picam":
        return _picamera(ancho, alto)

    if fuente is None:
        try:                                    # en una Raspberry Pi con cámara CSI
            return _picamera(ancho, alto)
        except Exception:
            fuente = 0                          # en un PC: webcam por defecto

    if isinstance(fuente, str) and fuente.isdigit():
        fuente = int(fuente)

    if isinstance(fuente, int) and platform.system() == "Windows":
        cap = cv2.VideoCapture(fuente, cv2.CAP_DSHOW)   # en Windows abre más rápido
    else:
        cap = cv2.VideoCapture(fuente)

    if isinstance(fuente, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         # menos retraso con cámaras por WiFi

    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir la cámara: {fuente}")
    print("Cámara:", fuente)

    def leer():
        ok, frame = cap.read()
        return frame if ok else None
    leer.cerrar = cap.release                   # CAMBIO: permite cerrar la cámara (para Ctrl+F5 / X)
    return leer


def fuente_desde_argumentos(argv, posicion=1):
    """Devuelve la fuente escrita en la terminal o None si no se indicó."""
    return argv[posicion] if len(argv) > posicion else None


# ═════════════════════════════════════════════════════════════════════════════
#  GRABADOR  (punto 2 y 9)
# ═════════════════════════════════════════════════════════════════════════════
def crear_writer(ruta_sin_ext, fps, tamano):
    """Crea un VideoWriter. Prueba MP4 y, si no se puede, AVI (MJPG)."""
    for codec, ext in (("mp4v", ".mp4"), ("MJPG", ".avi")):
        ruta = ruta_sin_ext + ext
        w = cv2.VideoWriter(ruta, cv2.VideoWriter_fourcc(*codec), fps, tamano)
        if w.isOpened():
            return w, ruta
        w.release()
    return None, None


def marca_tiempo(t=None):
    return datetime.fromtimestamp(t or time.time()).strftime("%Y%m%d_%H%M%S")


class Grabador:
    """
    Graba continuamente en trozos de SEGMENTO_SEG segundos dentro de CARPETA_TEMPORAL.
    Para guardar un tramo (Ctrl+G o alerta) junta los trozos necesarios en un solo video.
    Cada BLOQUE_SEG borra la grabación temporal, salvo que haya algo pendiente de guardar.
    """

    def __init__(self):
        os.makedirs(CARPETA_GUARDADAS, exist_ok=True)
        shutil.rmtree(CARPETA_TEMPORAL, ignore_errors=True)   # restos de una ejecución anterior
        os.makedirs(CARPETA_TEMPORAL, exist_ok=True)
        self.segmentos = []          # trozos cerrados: {"ruta", "t_ini", "t_fin", "tiempos"}
        self.actual = None           # trozo que se está escribiendo
        self.trabajos = []           # tramos pendientes de guardar: {"nombre", "t_ini", "t_fin"}
        self.hilos = []              # hilos que están armando videos guardados
        self.resultados = queue.Queue()
        self.guardadas = 0           # "Grabaciones guardadas" (manuales + alertas)
        self.inicio_bloque = time.time()
        self.fps_estimado = 10.0
        self._aviso_pospuesto = False

    # ---------- grabación continua ----------
    def escribir(self, frame, t):
        h, w = frame.shape[:2]
        if self.actual and (t - self.actual["t_ini"] >= SEGMENTO_SEG or self.actual["tamano"] != (w, h)):
            self.cerrar_segmento()
        if self.actual is None:
            base = os.path.join(CARPETA_TEMPORAL, f"seg_{marca_tiempo(t)}_{int(t * 1000) % 1000:03d}")
            writer, ruta = crear_writer(base, max(1.0, self.fps_estimado), (w, h))
            if writer is None:
                print("No se pudo crear el archivo de video temporal")
                return
            self.actual = {"writer": writer, "ruta": ruta, "t_ini": t, "tamano": (w, h), "tiempos": []}
        self.actual["writer"].write(frame)
        self.actual["tiempos"].append(t)

    def cerrar_segmento(self):
        a = self.actual
        if a is None:
            return
        a["writer"].release()
        self.actual = None
        if a["tiempos"]:
            self.segmentos.append({"ruta": a["ruta"], "t_ini": a["tiempos"][0],
                                   "t_fin": a["tiempos"][-1], "tiempos": a["tiempos"]})
        else:
            self._borrar_archivo(a["ruta"])

    # ---------- pedidos de guardado ----------
    def pedir_guardado(self, nombre, t_ini, t_fin=None):
        """t_fin = None significa 'todavía no se sabe' (alerta en curso)."""
        trabajo = {"nombre": nombre, "t_ini": t_ini, "t_fin": t_fin}
        self.trabajos.append(trabajo)
        return trabajo

    def ocupado(self):
        return bool(self.trabajos) or any(h.is_alive() for h in self.hilos)

    def revisar(self, ahora):
        """Se llama periódicamente: termina guardados y hace el corte de 30 min."""
        while not self.resultados.empty():
            ok, info = self.resultados.get()
            if ok:
                self.guardadas += 1
                print("Grabación guardada:", info)
            else:
                print("No se pudo guardar la grabación:", info)
        self.hilos = [h for h in self.hilos if h.is_alive()]

        for tr in list(self.trabajos):
            if tr["t_fin"] is not None and tr["t_fin"] <= ahora:
                self.trabajos.remove(tr)
                self._lanzar(tr)

        if ahora - self.inicio_bloque >= BLOQUE_SEG:
            if self.ocupado():
                # Pantalla roja / guardado en curso: NO se puede borrar la grabación todavía
                if not self._aviso_pospuesto:
                    print("Corte de 30 min pospuesto: hay una grabación pendiente de guardar")
                    self._aviso_pospuesto = True
            else:
                self._cortar_bloque(ahora)

    def _cortar_bloque(self, ahora):
        self.cerrar_segmento()
        limite = ahora - CONSERVAR_AL_CORTAR_SEG
        borrar = [s for s in self.segmentos if s["t_fin"] < limite]
        for s in borrar:
            self._borrar_archivo(s["ruta"])
        self.segmentos = [s for s in self.segmentos if s["t_fin"] >= limite]
        self.inicio_bloque = ahora
        self._aviso_pospuesto = False
        print(f"Corte de 30 min: grabación temporal borrada ({len(borrar)} trozos). Se sigue grabando.")

    def _lanzar(self, tr):
        self.cerrar_segmento()                                  # para incluir lo último grabado
        segs = [s for s in self.segmentos if s["t_fin"] >= tr["t_ini"] and s["t_ini"] <= tr["t_fin"]]
        if not segs:
            print("No hay grabación para guardar en ese intervalo")
            return
        destino = os.path.join(CARPETA_GUARDADAS, tr["nombre"])
        h = threading.Thread(target=self._unir, args=(segs, tr["t_ini"], tr["t_fin"], destino))
        h.start()
        self.hilos.append(h)

    def _unir(self, segs, t_ini, t_fin, destino):
        """(Hilo aparte) junta los trozos en un solo video recortado a [t_ini, t_fin]."""
        try:
            tiempos = [t for s in segs for t in s["tiempos"] if t_ini <= t <= t_fin]
            if len(tiempos) < 2:
                self.resultados.put((False, "muy pocos cuadros"))
                return
            fps = (len(tiempos) - 1) / max(0.001, tiempos[-1] - tiempos[0])  # FPS reales -> duración correcta
            fps = min(60.0, max(1.0, fps))
            writer, ruta, tamano, n = None, None, None, 0
            for s in segs:
                cap = cv2.VideoCapture(s["ruta"])
                for t in s["tiempos"]:
                    ok, f = cap.read()
                    if not ok or t > t_fin:
                        break
                    if t < t_ini:
                        continue
                    if writer is None:
                        tamano = (f.shape[1], f.shape[0])
                        writer, ruta = crear_writer(destino, fps, tamano)
                        if writer is None:
                            cap.release()
                            self.resultados.put((False, "no se pudo crear " + destino))
                            return
                    if (f.shape[1], f.shape[0]) != tamano:
                        f = cv2.resize(f, tamano)
                    writer.write(f)
                    n += 1
                cap.release()
            if writer is not None:
                writer.release()
            self.resultados.put((n > 0, f"{ruta} ({n} cuadros, {n / fps:.0f} s)"))
        except Exception as e:
            self.resultados.put((False, str(e)))

    # ---------- salida ----------
    def finalizar(self):
        """Guarda lo pendiente (con lo que haya) y borra la grabación temporal."""
        ahora = time.time()
        self.cerrar_segmento()
        for tr in list(self.trabajos):
            tr["t_fin"] = ahora if tr["t_fin"] is None else min(tr["t_fin"], ahora)
        self.revisar(ahora)
        if self.hilos:
            print("Guardando grabaciones pendientes, espera...")
        for h in self.hilos:
            h.join()
        self.revisar(time.time())
        shutil.rmtree(CARPETA_TEMPORAL, ignore_errors=True)

    @staticmethod
    def _borrar_archivo(ruta):
        try:
            os.remove(ruta)
        except OSError:
            pass


# ═════════════════════════════════════════════════════════════════════════════
#  COMUNICACIÓN CON EL ESP32 DEL SEMÁFORO (punto 3)
# ═════════════════════════════════════════════════════════════════════════════
_sin_proxy = urllib.request.build_opener(urllib.request.ProxyHandler({}))   # red local: sin proxy
semaforo = {"peatones": "DESCONOCIDO", "autos": "DESCONOCIDO", "conectado": False}
detener = threading.Event()


def hilo_semaforo(ip):
    """Pregunta al ESP32 http://<IP>/estado y guarda el color de cada semáforo."""
    url = f"http://{ip}/estado"
    avisado = None
    while not detener.is_set():
        try:
            with _sin_proxy.open(url, timeout=1.5) as resp:
                datos = json.loads(resp.read().decode())
            semaforo["peatones"] = str(datos.get("peatones", "DESCONOCIDO")).upper()
            semaforo["autos"] = str(datos.get("autos", "DESCONOCIDO")).upper()
            semaforo["conectado"] = True
        except Exception:
            semaforo.update(peatones="DESCONOCIDO", autos="DESCONOCIDO", conectado=False)
        if avisado != semaforo["conectado"]:
            avisado = semaforo["conectado"]
            print("Semáforo ESP32:", "conectado" if avisado else f"sin respuesta en {url}")
        detener.wait(PERIODO_CONSULTA_SEG)


def avisar_esp32(encendida):
    """Le pide al ESP32 que encienda/apague su LED de alerta (no bloquea la cámara)."""
    if not (ESP32_IP and AVISAR_ALERTA_AL_ESP32):
        return
    url = f"http://{ESP32_IP}/alerta?estado={'on' if encendida else 'off'}"

    def enviar():
        try:
            _sin_proxy.open(url, timeout=1.5).close()
        except Exception:
            pass
    threading.Thread(target=enviar, daemon=True).start()


# ═════════════════════════════════════════════════════════════════════════════
#  DETECCIÓN
# ═════════════════════════════════════════════════════════════════════════════
ruta = "yolo26n_ncnn_model" if os.path.isdir("yolo26n_ncnn_model") else "yolo26n.pt"
modelo = YOLO(ruta)
print("Modelo:", ruta)

IDS_CLASES = [k for k, v in modelo.names.items() if v in CLASES_PERSONA | CLASES_AUTO]
CONF_MINIMA = min(CONF_PERSONA, CONF_AUTO)
usar_seguimiento = True          # usa IDs de seguimiento para contar cada persona/auto UNA sola vez


def en_zona(x1, y1, x2, y2):
    if PUNTO_DETECCION == "caja":
        return x1 < ZONA_X2 and x2 > ZONA_X1 and y1 < ZONA_Y2 and y2 > ZONA_Y1
    px = (x1 + x2) / 2
    py = y2 if PUNTO_DETECCION == "abajo" else (y1 + y2) / 2
    return ZONA_X1 <= px <= ZONA_X2 and ZONA_Y1 <= py <= ZONA_Y2


def detectar(frame):
    """Devuelve (resultado_filtrado, personas, autos). Cada objeto = (x1, y1, x2, y2, id)."""
    global usar_seguimiento
    # CAMBIO: antes era modelo(frame, imgsz=TAMANO, conf=0.5, verbose=False)[0].
    # Ahora se limita a personas y autos, y se usa .track() para no contar el mismo objeto dos veces.
    r = None
    if usar_seguimiento:
        try:
            r = modelo.track(frame, persist=True, imgsz=TAMANO, conf=CONF_MINIMA,
                             classes=IDS_CLASES, verbose=False)[0]
        except Exception as e:
            print("Seguimiento no disponible, se usa conteo simple:", e)
            usar_seguimiento = False
    if r is None:
        r = modelo(frame, imgsz=TAMANO, conf=CONF_MINIMA, classes=IDS_CLASES, verbose=False)[0]

    conservar, personas, autos = [], [], []
    for i, c in enumerate(r.boxes):
        nombre = modelo.names[int(c.cls)]
        if nombre in CLASES_PERSONA:
            lim, conf_min, lista = TAMANO_PERSONA, CONF_PERSONA, personas
        elif nombre in CLASES_AUTO:
            lim, conf_min, lista = TAMANO_AUTO, CONF_AUTO, autos
        else:
            continue
        x1, y1, x2, y2 = (float(v) for v in c.xyxy[0])
        ancho, alto = x2 - x1, y2 - y1
        if float(c.conf) < conf_min:
            continue
        if not (lim["min_ancho"] <= ancho <= lim["max_ancho"] and lim["min_alto"] <= alto <= lim["max_alto"]):
            continue                                    # descartado por TAMAÑO (ver AJUSTE DE TAMAÑO arriba)
        id_obj = int(c.id) if c.id is not None else None
        conservar.append(i)
        lista.append((x1, y1, x2, y2, id_obj))
    return r[conservar], personas, autos


def tenir_rojo(img):
    capa = np.empty_like(img)
    capa[:] = (0, 0, 255)                               # rojo en BGR
    return cv2.addWeighted(img, 1 - INTENSIDAD_ROJO, capa, INTENSIDAD_ROJO, 0)


# ═════════════════════════════════════════════════════════════════════════════
#  ESTADO DEL PROGRAMA
# ═════════════════════════════════════════════════════════════════════════════
grabador = Grabador()
leer = None                      # función de la cámara (None = cámara cerrada)
ventana_cam = None
etiqueta_cam = None
imagen_tk = None
fuente = fuente_desde_argumentos(sys.argv)
if len(sys.argv) > 2:
    ESP32_IP = sys.argv[2]

contador_autos = 0               # autos identificados (cada uno contado una vez)
contador_personas = 0            # personas identificadas (cada una contada una vez)
ids_autos, ids_personas = set(), set()
previos = {"autos": 0, "personas": 0}
alertas = 0                      # veces que la pantalla se puso roja
alerta_activa = False
ultima_condicion = 0.0
trabajo_alerta = None
fps = 0.0
t_anterior = None
aviso_zona = False
generacion = 0                   # cambia en cada apertura de la cámara (evita dos bucles a la vez)


def contar(personas, autos):
    """Actualiza los contadores acumulados de personas y autos."""
    global contador_autos, contador_personas
    if usar_seguimiento:
        for *_, i in personas:
            if i is not None and i not in ids_personas:
                ids_personas.add(i)
                contador_personas += 1
        for *_, i in autos:
            if i is not None and i not in ids_autos:
                ids_autos.add(i)
                contador_autos += 1
    else:   # sin seguimiento: se suma cuando aparecen más objetos que en el cuadro anterior
        contador_personas += max(0, len(personas) - previos["personas"])
        contador_autos += max(0, len(autos) - previos["autos"])
    previos["personas"], previos["autos"] = len(personas), len(autos)


def leer_semaforo():
    if ESP32_IP:
        return semaforo["peatones"], semaforo["autos"]
    return SEMAFORO_MANUAL_PEATONES, SEMAFORO_MANUAL_AUTOS


# ═════════════════════════════════════════════════════════════════════════════
#  BUCLE DE LA CÁMARA
# ═════════════════════════════════════════════════════════════════════════════
def ciclo_camara(gen):
    global fps, t_anterior, alerta_activa, ultima_condicion, alertas, trabajo_alerta, imagen_tk, aviso_zona
    if leer is None or gen != generacion:              # cámara cerrada o reabierta con Control + F5
        return
    frame = leer()
    if frame is None:
        print("Se perdió la imagen de la cámara. Pulsa Control + F5 para reabrirla.")
        cerrar_camara()
        return
    ahora = time.time()
    # CAMBIO: los FPS ahora son los reales del ciclo completo (antes solo medían el tiempo de YOLO)
    if t_anterior is not None:
        instantaneo = 1 / max(1e-6, ahora - t_anterior)
        fps = instantaneo if fps == 0 else fps * 0.9 + instantaneo * 0.1
        grabador.fps_estimado = fps
    t_anterior = ahora

    if not aviso_zona and (ZONA_X2 > frame.shape[1] or ZONA_Y2 > frame.shape[0]):
        print(f"AVISO: la zona ({ZONA_X1},{ZONA_Y1})-({ZONA_X2},{ZONA_Y2}) se sale de la imagen "
              f"({frame.shape[1]}x{frame.shape[0]}). Ajusta ZONA_X1..ZONA_Y2.")
        aviso_zona = True

    r, personas, autos = detectar(frame)
    contar(personas, autos)

    persona_en_zona = any(en_zona(*p[:4]) for p in personas)
    auto_en_zona = any(en_zona(*a[:4]) for a in autos)

    # ╔═══════════════════════════════════════════════════════════════════════╗
    # ║ >>> AQUÍ ESTÁN LAS CONDICIONES DEL SEMÁFORO (punto 7) <<<             ║
    # ║ semaforo_peatones y semaforo_autos son las VARIABLES; sus valores     ║
    # ║ posibles son "ROJO", "AMARILLO" y "VERDE" (los manda el ESP32).       ║
    # ╚═══════════════════════════════════════════════════════════════════════╝
    semaforo_peatones, semaforo_autos = leer_semaforo()

    # CONDICIÓN 1 -> LA DE LA PERSONA:
    #   Peatones en ROJO y Autos en VERDE o AMARILLO  =>  la imagen se pone roja
    #   SOLO si una PERSONA está dentro de las coordenadas de la zona.
    if semaforo_peatones == "ROJO" and semaforo_autos in ("VERDE", "AMARILLO"):
        condicion = persona_en_zona

    # CONDICIÓN 2 -> LA DEL AUTO:
    #   Peatones en VERDE (también mientras parpadea) y Autos en ROJO  =>  la imagen se pone roja
    #   SOLO si un AUTO está dentro de las coordenadas de la zona.
    elif semaforo_peatones == "VERDE" and semaforo_autos == "ROJO":
        condicion = auto_en_zona

    # Cualquier otro caso (por ejemplo, ESP32 desconectado): la imagen se ve normal
    else:
        condicion = False

    if condicion:
        ultima_condicion = ahora
    nueva = condicion or (alerta_activa and ahora - ultima_condicion < TOLERANCIA_SALIDA_SEG)

    if nueva and not alerta_activa:                     # la pantalla SE PONE ROJA
        alertas += 1
        avisar_esp32(True)
        if trabajo_alerta is not None and any(t is trabajo_alerta for t in grabador.trabajos):
            trabajo_alerta["t_fin"] = None              # sigue dentro de los 10 s de la anterior: mismo video
        else:
            trabajo_alerta = grabador.pedir_guardado(f"alerta_{marca_tiempo(ahora)}",
                                                     ahora - ANTES_ALERTA_SEG)
    elif alerta_activa and not nueva:                   # la pantalla VUELVE A LA NORMALIDAD
        avisar_esp32(False)
        if trabajo_alerta is not None:
            trabajo_alerta["t_fin"] = ahora + DESPUES_ALERTA_SEG
    alerta_activa = nueva

    # ---------- dibujo ----------
    vista = r.plot(line_width=GROSOR_CAJA)
    if alerta_activa:
        vista = tenir_rojo(vista)
    if MOSTRAR_ZONA:
        color = (0, 0, 255) if alerta_activa else (0, 255, 255)
        cv2.rectangle(vista, (int(ZONA_X1), int(ZONA_Y1)), (int(ZONA_X2), int(ZONA_Y2)), color, 2)
    cv2.putText(vista, f"person: {len(personas)}  car: {len(autos)}   FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(vista, f"Peatones: {semaforo_peatones}  Autos: {semaforo_autos}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if alerta_activa:
        cv2.putText(vista, "ALERTA", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    if int(ahora * 2) % 2 == 0:                         # punto "REC" parpadeante
        cv2.circle(vista, (vista.shape[1] - 20, 20), 8, (0, 0, 255), -1)

    # ---------- grabación continua (punto 2) ----------
    grabador.escribir(vista if GRABAR_ANOTADO else frame, ahora)

    # ---------- mostrar ----------
    # CAMBIO: la imagen se muestra en una ventana Tkinter en vez de cv2.imshow, porque OpenCV
    # no detecta bien Control + G / Control + F5 ni deja seguir escuchando teclas con la ventana cerrada.
    if ventana_cam is not None:
        if ESCALA_VENTANA != 1.0:
            vista = cv2.resize(vista, None, fx=ESCALA_VENTANA, fy=ESCALA_VENTANA)
        imagen_tk = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(vista, cv2.COLOR_BGR2RGB)))
        etiqueta_cam.configure(image=imagen_tk)
    raiz.after(1, ciclo_camara, gen)


# ═════════════════════════════════════════════════════════════════════════════
#  VENTANAS Y TECLAS (punto 8 y 10)
# ═════════════════════════════════════════════════════════════════════════════
def abrir_camara_ui():
    global leer, ventana_cam, etiqueta_cam, t_anterior, aviso_zona, generacion
    try:
        leer = abrir_camara(fuente)
    except Exception as e:
        print("Error:", e, "-> pulsa Control + F5 para reintentar")
        leer = None
        return
    ventana_cam = tk.Toplevel(raiz)
    ventana_cam.title("YOLO en tiempo real")
    # La X (QUIT) y Alt + F4 envían WM_DELETE_WINDOW: se cierra la cámara, el programa sigue
    ventana_cam.protocol("WM_DELETE_WINDOW", cerrar_camara)
    etiqueta_cam = tk.Label(ventana_cam, bg="black")
    etiqueta_cam.pack()
    t_anterior, aviso_zona = None, False
    generacion += 1
    raiz.after(1, ciclo_camara, generacion)


def cerrar_camara():
    global leer, ventana_cam, alerta_activa
    if leer is not None:
        try:
            leer.cerrar()
        except Exception:
            pass
        leer = None
    if alerta_activa:                                   # si estaba en rojo, se termina la alerta
        alerta_activa = False
        avisar_esp32(False)
        if trabajo_alerta is not None:
            trabajo_alerta["t_fin"] = time.time() + DESPUES_ALERTA_SEG
    grabador.cerrar_segmento()
    if ventana_cam is not None:
        ventana_cam.destroy()
        ventana_cam = None
    print("Cámara cerrada. Pulsa Control + F5 para abrirla de nuevo.")


def tecla_ctrl_g(_evento=None):
    ahora = time.time()
    grabador.pedir_guardado(f"manual_{marca_tiempo(ahora)}", ahora - MANUAL_SEG, ahora)
    grabador.revisar(ahora)
    print("Control + G: guardando los últimos 5 minutos...")
    return "break"


def tecla_ctrl_f5(_evento=None):
    print("Control + F5: refrescando la cámara...")
    if leer is not None or ventana_cam is not None:
        cerrar_camara()
    abrir_camara_ui()
    return "break"


def texto_notas():
    fps_mostrado = fps if leer is not None else 0.0
    return ("Comandos:\n"
            "Pulsa 'G + Control' para guardar los ultimos 5 minutos de la camara\n"
            "Pulsa 'Alt + F4' para cerrar la camara\n"
            "Pulsa 'Control + F5' para refrescar o abrir de nuevo la camara\n"
            "Estadisticas:\n"
            f"Contador de autos: {contador_autos}\n"
            f"Contador de personas: {contador_personas}\n"
            f"Alertas de incumplimiento: {alertas}\n"
            f"Grabaciones guardadas: {grabador.guardadas}\n"
            f"FPS: {fps_mostrado:.1f}")


def tarea_periodica():
    """Cada 0,5 s: guardados pendientes, corte de 30 min y bloc de notas."""
    grabador.revisar(time.time())
    nuevo = texto_notas()
    if notas.get("1.0", "end-1c") != nuevo:
        notas.configure(state="normal")                 # se habilita solo un instante para escribir...
        notas.delete("1.0", "end")
        notas.insert("1.0", nuevo)
        notas.configure(state="disabled")               # ...y queda bloqueado: el usuario no puede editar
    estado_cam = "abierta" if leer is not None else "cerrada (Control + F5 para abrir)"
    raiz.title(f"Bloc de notas - Cámara {estado_cam}")
    raiz.after(500, tarea_periodica)


def salir():
    detener.set()
    if leer is not None or ventana_cam is not None:
        cerrar_camara()
    avisar_esp32(False)
    grabador.finalizar()
    raiz.destroy()


# Bloc de notas (ventana principal, solo lectura)
raiz = tk.Tk()
raiz.title("Bloc de notas")
notas = tk.Text(raiz, width=70, height=10, font=("Consolas", 11), wrap="none",
                bg="white", fg="black", cursor="arrow")
notas.pack(fill="both", expand=True)
notas.configure(state="disabled")
raiz.protocol("WM_DELETE_WINDOW", salir)               # cerrar el bloc de notas = terminar el programa

# Teclas válidas en ambas ventanas
for secuencia in ("<Control-g>", "<Control-G>"):
    raiz.bind_all(secuencia, tecla_ctrl_g)
raiz.bind_all("<Control-F5>", tecla_ctrl_f5)
# CAMBIO: la tecla q ya NO cierra la cámara (se cierra con la X o Alt + F4)

if ESP32_IP:
    threading.Thread(target=hilo_semaforo, args=(ESP32_IP,), daemon=True).start()
else:
    print("Sin ESP32: se usan los valores manuales del semáforo")

abrir_camara_ui()                                      # la grabación empieza sola al abrir la cámara
tarea_periodica()
try:
    raiz.mainloop()
except KeyboardInterrupt:
    salir()
