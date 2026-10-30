"""
Núcleo sin hardware del plugin sk-py-bno08x (rama majareta del fork DujekLabs).

Todo lo que no toca el bus I2C vive aquí para poder probarlo sin el sensor:
matemáticas (cuaternión -> actitud del barco con el montaje, rotación al marco Tierra, rateOfTurn),
lectura de la configuración, construcción del delta de Signal K y el bucle con
vigilancia y reconexión. plugin.py solo conecta esto con la librería de Adafruit,
la red (declinación NOAA) y la salida estándar.

Marcos de referencia (ver README, apartado "Convención de ejes"):
- Sensor: marco Android del BNO08x (X, Y, Z del chip, Z sale de la cara del chip).
- Tierra: el del cuaternión del BNO08x, ENU (X Este, Y Norte magnético, Z arriba).
  Con el game rotation vector el "Norte" es arbitrario, pero Z sigue siendo la vertical.
- Barco: el de Signal K / aeronáutico, x proa, y estribor, z abajo. El paso sensor -> barco
  es EXPLÍCITO: qué eje del chip va a proa y cuál a estribor (devEjeProa, devEjeEstribor;
  por defecto +Y y +X), y de ahí la actitud (roll + estribor abajo, pitch + proa arriba,
  yaw + a estribor).

MIT License, Copyright (c) 2026 dpazz (código original) y DujekLabs (cambios majareta).
"""

import datetime
import logging
import math
import struct
import time
from collections import namedtuple

logger = logging.getLogger("sk-py-bno08x")

# Identificadores de informe del SH-2 (los mismos valores que adafruit_bno08x)
INF_GIROSCOPIO = 0x02
INF_MAGNETOMETRO = 0x03
INF_ACEL_LINEAL = 0x04
INF_VECTOR_ROTACION = 0x05
INF_VECTOR_ROTACION_JUEGO = 0x08

INFORMES_SEGUIDOS = (
    INF_GIROSCOPIO,
    INF_MAGNETOMETRO,
    INF_ACEL_LINEAL,
    INF_VECTOR_ROTACION,
    INF_VECTOR_ROTACION_JUEGO,
)

NOMBRE_INFORME = {
    INF_GIROSCOPIO: "gyroscope",
    INF_MAGNETOMETRO: "magnetometer",
    INF_ACEL_LINEAL: "linearAcceleration",
    INF_VECTOR_ROTACION: "rotationVector",
    INF_VECTOR_ROTACION_JUEGO: "gameRotationVector",
}

REPORT_ACCURACY_STATUS = [
    "Accuracy Unreliable",
    "Low Accuracy",
    "Medium Accuracy",
    "High Accuracy",
]

FUENTE_ROTACION = "rotationVector"
FUENTE_JUEGO = "gameRotationVector"

DOS_PI = 2 * math.pi

# Una muestra de un informe: valores, precisión 0-3 (bits de estado del informe),
# instante de llegada (reloj monotónico) y extra (precisión del rumbo en rad para
# el rotation vector, None en los demás).
Muestra = namedtuple("Muestra", "valores precision t extra")


class ImuSinDatos(Exception):
    """El sensor no entrega muestras nuevas: colgado, reseteado o bus caído."""


# ---------------------------------------------------------------------------
# Lectura de los informes crudos del SH-2
# ---------------------------------------------------------------------------

def analizar_extras(report_id, report_bytes):
    """Devuelve (precision 0-3, precision_rumbo_rad o None) de un informe de sensor.

    La librería de Adafruit solo guarda la precisión del magnetómetro y descarta la
    estimación de precisión del rumbo del rotation vector (bytes 12-13, Q12 en rad)."""
    precision = report_bytes[2] & 0b11
    precision_rumbo = None
    if report_id == INF_VECTOR_ROTACION and len(report_bytes) >= 14:
        precision_rumbo = struct.unpack_from("<h", report_bytes, 12)[0] * 2 ** -12
    return precision, precision_rumbo


def crear_clase_bno(base, reloj=time.monotonic, max_paquetes=40):
    """Subclase de BNO08X_I2C (u otra base compatible) que guarda, por informe, la
    última muestra con su precisión y el instante real en que llegó."""

    class BnoMajareta(base):
        def __init__(self, *args, **kwargs):
            self.muestras = {}
            super().__init__(*args, **kwargs)

        def _process_report(self, report_id, report_bytes):
            super()._process_report(report_id, report_bytes)
            if report_id in INFORMES_SEGUIDOS:
                precision, extra = analizar_extras(report_id, report_bytes)
                self.muestras[report_id] = Muestra(
                    self._readings.get(report_id), precision, reloj(), extra)

        def actualizar(self):
            """Lee lo pendiente en el bus (con tope, para que un chip que siempre dice
            'tengo datos' no bloquee el bucle) y devuelve las últimas muestras."""
            self._process_available_packets(max_packets=max_paquetes)
            return dict(self.muestras)

        def iniciar_calibracion(self):
            self.begin_calibration()

        def guardar_calibracion(self):
            self.save_calibration_data()

    return BnoMajareta


# ---------------------------------------------------------------------------
# Matemáticas
# ---------------------------------------------------------------------------

# EL MONTAJE (auditoría Codex 30-oct, H7): qué eje del chip va a proa y cuál a estribor. Antes la actitud salía de la
# fórmula original, que toma el rumbo del +Y del chip pero el balance alrededor del +X: con el +Y a proa (el montaje del
# README), balance y cabeceo salían cambiados (escorar 20° a estribor daba pitch -20°). Ahora la transformación es explícita.
EJES_SENSOR = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0), "+y": (0.0, 1.0, 0.0),
               "-y": (0.0, -1.0, 0.0), "+z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0)}
MONTAJE_POR_DEFECTO = ("+y", "+x")  # el del README: +Y del chip a proa, +X a estribor, la cara (+Z) arriba


def _cruz(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def ejes_montaje(proa, estribor):
    """('+y', '+x') -> los dos vectores del marco del sensor. ValueError si un eje no
    existe o si proa y estribor no son perpendiculares (un montaje imposible)."""
    p, e = EJES_SENSOR.get(str(proa).lower()), EJES_SENSOR.get(str(estribor).lower())
    if p is None or e is None:
        raise ValueError("eje del sensor desconocido: %s / %s (vale +x, -x, +y, -y, +z, -z)" % (proa, estribor))
    if abs(sum(a * b for a, b in zip(p, e))) > 1e-9:
        raise ValueError("proa (%s) y estribor (%s) tienen que ser perpendiculares" % (proa, estribor))
    return p, e


def actitud_barco(q_ijkr, montaje=None):
    """Cuaternión (i, j, k, real) de la librería -> (roll, pitch, yaw) del BARCO en rad,
    convención de Signal K (Z-Y-X, la misma que tierra_a_barco): roll + estribor abajo,
    pitch + proa arriba, yaw = rumbo de la proa en sentido horario desde el Norte del
    cuaternión. Se giran al marco Tierra (ENU) los ejes del chip que van a proa y a
    estribor: yaw = atan2(proa.E, proa.N), pitch = asin(proa.arriba), roll =
    atan2(-estribor.arriba, cubierta.arriba), con cubierta = estribor x proa."""
    p, e = montaje if montaje is not None else ejes_montaje(*MONTAJE_POR_DEFECTO)
    f, s = rotar_a_tierra(q_ijkr, p), rotar_a_tierra(q_ijkr, e)
    arriba = _cruz(s, f)
    pitch = math.asin(max(-1.0, min(1.0, f[2])))
    roll = math.atan2(-s[2], arriba[2])
    yaw = envolver(math.atan2(f[0], f[1]))
    return roll, pitch, yaw


def envolver(angulo):
    """Ángulo a [0, 2*pi)."""
    a = math.fmod(angulo, DOS_PI)
    if a < 0:
        a += DOS_PI
    if a >= DOS_PI:  # fmod de un negativo diminuto puede dar exactamente 2*pi
        a = 0.0
    return a


def rotar_a_tierra(q_ijkr, v):
    """Rota el vector v (marco del sensor) con el cuaternión (i, j, k, real) de la
    librería: devuelve v en el marco Tierra del cuaternión (ENU)."""
    i, j, k, r = q_ijkr
    n = math.sqrt(i * i + j * j + k * k + r * r)
    if n == 0:
        raise ValueError("cuaternión nulo")
    i, j, k, r = i / n, j / n, k / n, r / n
    vx, vy, vz = v
    # v' = v + 2r (u x v) + 2 u x (u x v), con u = (i, j, k)
    cx = j * vz - k * vy
    cy = k * vx - i * vz
    cz = i * vy - j * vx
    ccx = j * cz - k * cy
    ccy = k * cx - i * cz
    ccz = i * cy - j * cx
    return (vx + 2 * (r * cx + ccx),
            vy + 2 * (r * cy + ccy),
            vz + 2 * (r * cz + ccz))


def tasa_de_giro(q_ijkr, gyro):
    """navigation.rateOfTurn (rad/s, + a estribor): componente vertical (Tierra) de la
    velocidad angular. El giróscopo da + para giros antihorarios (regla de la mano
    derecha); con Z arriba, antihorario visto desde arriba es caer a babor, de ahí el
    signo menos. No depende de cómo esté montado el sensor."""
    return -rotar_a_tierra(q_ijkr, gyro)[2]


def tierra_a_barco(a_enu, roll, pitch, yaw):
    """Vector del marco Tierra (ENU) al marco del barco de Signal K (x proa, y estribor,
    z abajo), usando la actitud publicada (convención aeronáutica Z-Y-X)."""
    n, e, d = a_enu[1], a_enu[0], -a_enu[2]
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    x1 = cy * n + sy * e
    y1 = -sy * n + cy * e
    z1 = d
    x2 = cp * x1 - sp * z1
    z2 = sp * x1 + cp * z1
    y3 = cr * y1 + sr * z2
    z3 = -sr * y1 + cr * z2
    return x2, y3, z3


# ---------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------

def _num(opciones, clave, defecto):
    valor = opciones.get(clave, defecto)
    try:
        return float(valor)
    except (TypeError, ValueError):
        return float(defecto)


def _frecuencia(opciones, clave, defecto, maximo=50.0):
    f = _num(opciones, clave, defecto)
    if f <= 0:
        f = defecto
    return min(f, maximo)


class Config:
    """Opciones de un dispositivo de imuDevices (schema.json), con valores por defecto."""

    def __init__(self, opciones):
        o = opciones or {}
        self.direccion = o.get("devName", 75)
        self.retardo = int(_num(o, "devDelayReports", 0))
        self.calib_inicio = bool(o.get("devCalibRequired", False))
        self.decl_needed = bool(o.get("devDeclRequired", False))
        self.decl_interval = _num(o, "devDeclInterval", 5)
        self.decl_estimate = _num(o, "devDeclEstimate", 3)
        self.hdgOffset = _num(o, "devHdgOffset", 0)
        self.hdgDeviation = _num(o, "devHdgDeviation", 0)
        self.rollOffset = _num(o, "devRollOffset", 0)
        self.pitchOffset = _num(o, "devPitchOffset", 0)
        # el montaje: qué eje del chip va a proa y cuál a estribor (H7); uno imposible no arranca
        self.montaje = ejes_montaje(o.get("devEjeProa", MONTAJE_POR_DEFECTO[0]), o.get("devEjeEstribor", MONTAJE_POR_DEFECTO[1]))
        # --- nuevas (rama majareta) ---
        fuente = o.get("devHeadingSource", FUENTE_ROTACION)
        self.fuente_rumbo = fuente if fuente in (FUENTE_ROTACION, FUENTE_JUEGO) else FUENTE_ROTACION
        self.f_actitud = _frecuencia(o, "devAttitudeRate", 10)
        self.f_rumbo = _frecuencia(o, "devHeadingRate", 10)
        self.f_acel = _frecuencia(o, "devAccelRate", 10)
        self.f_estado = _frecuencia(o, "devStatusRate", 1, maximo=5)
        self.rate_of_turn = bool(o.get("devRateOfTurn", True))
        self.acel_lineal = bool(o.get("devLinearAcceleration", True))
        self.calib_dinamica = bool(o.get("devDynamicCalibration", True))
        self.autoguardar = bool(o.get("devAutoSaveCalibration", False))
        self.autoguardar_min = int(min(3, max(1, _num(o, "devAutoSaveMinStatus", 3))))
        self.autoguardar_seg = max(1.0, _num(o, "devAutoSaveSeconds", 30))
        self.autoguardar_intervalo = max(1.0, _num(o, "devAutoSaveIntervalMin", 60)) * 60
        self.vigilancia_seg = max(0.5, _num(o, "devWatchdogSeconds", 3))

    @property
    def informe_cuaternion(self):
        return INF_VECTOR_ROTACION if self.fuente_rumbo == FUENTE_ROTACION else INF_VECTOR_ROTACION_JUEGO

    def informes(self):
        """{informe: intervalo en microsegundos} que hay que activar en el chip. Cada
        informe va al doble de la frecuencia a la que se publica, para que la muestra
        que se publica nunca tenga más de medio periodo de antigüedad."""
        def us(hz):
            return int(1e6 / min(100.0, 2 * hz))
        inf = {
            self.informe_cuaternion: us(max(self.f_actitud, self.f_rumbo, self.f_acel)),
            INF_MAGNETOMETRO: us(2.5),  # solo para su estado de calibración (0-3)
        }
        if self.rate_of_turn:
            inf[INF_GIROSCOPIO] = us(self.f_actitud)
        if self.acel_lineal:
            inf[INF_ACEL_LINEAL] = us(self.f_acel)
        return inf

    def edad_maxima(self, informe):
        """Antigüedad máxima (s) para publicar una muestra de ese informe."""
        hz = 1e6 / self.informes().get(informe, 200000)
        return max(0.3, 3.0 / hz)


# ---------------------------------------------------------------------------
# Calibración de arranque (opción antigua devCalibRequired)
# ---------------------------------------------------------------------------

def esperar_calibracion(leer_estado, reloj=time.monotonic, dormir=time.sleep,
                        minimo=2, estable_seg=5.0, limite_seg=50.0, paso=0.1):
    """Espera a que el estado de calibración sea >= minimo durante estable_seg.
    Devuelve (ok, ultimo_estado). Antes se guardaba la calibración también al agotar
    el tiempo; ahora solo se guarda si ok."""
    inicio = reloj()
    bueno_desde = None
    estado = 0
    while True:
        dormir(paso)
        estado = leer_estado()
        ahora = reloj()
        if estado >= minimo:
            if bueno_desde is None:
                bueno_desde = ahora
            if ahora - bueno_desde > estable_seg:
                return True, estado
        else:
            bueno_desde = None
        if ahora - inicio > limite_seg:
            return False, estado


# ---------------------------------------------------------------------------
# Declinación fija (la de red vive en plugin.py)
# ---------------------------------------------------------------------------

class DeclinacionFija:
    """Proveedor de declinación mínimo: valor fijo, una única 'novedad' al principio."""

    def __init__(self, valor_rad, publicar=False):
        self.valor = valor_rad
        self._pendiente = publicar

    def tomar_nueva(self):
        if self._pendiente:
            self._pendiente = False
            return self.valor
        return None


# ---------------------------------------------------------------------------
# Bucle
# ---------------------------------------------------------------------------

RUTAS_META = [
    ("sensors.imu.acceleration",
     {"description": "Aceleración lineal sin gravedad en el marco del barco {x proa, y estribor, z abajo}, m/s2",
      "units": "m/s2"}),
    ("sensors.imu.verticalAcceleration",
     {"description": "Aceleración vertical sin gravedad en el marco Tierra, + arriba", "units": "m/s2"}),
    ("sensors.imu.headingAccuracy",
     {"description": "Precisión estimada del rumbo del rotation vector del BNO08x", "units": "rad"}),
    ("sensors.imu.state", {"description": "Estado del IMU: ok / reconectando"}),
]


class BucleImu:
    """Lee el sensor, construye un delta por ciclo y gestiona fallos y reconexión.

    conectar(cfg) -> objeto con actualizar() -> {informe: Muestra},
                     iniciar_calibracion(), guardar_calibracion()
    emitir(delta)  -> escribe un delta de Signal K
    estado(texto, es_error) -> estado del plugin en la interfaz de Signal K
    """

    MAX_FALLOS = 3
    ESPERA_MIN = 1.0
    ESPERA_MAX = 30.0

    def __init__(self, cfg, conectar, emitir, src, estado=None, declinacion=None,
                 reloj=time.monotonic, dormir=time.sleep, utc=None):
        self.cfg = cfg
        self.conectar = conectar
        self.emitir = emitir
        self._src = src  # texto o función (el src depende de la dirección encontrada)
        self.estado = estado or (lambda texto, error=False: None)
        self.declinacion = declinacion or DeclinacionFija(cfg.decl_estimate * math.pi / 180, cfg.decl_needed)
        self.reloj = reloj
        self.dormir = dormir
        self.utc = utc or (lambda: datetime.datetime.now(datetime.timezone.utc))
        self.hw = None
        self.fallos = 0
        self.espera = self.ESPERA_MIN
        self.ultimo_estado = None
        self.periodos = {
            "actitud": 1.0 / cfg.f_actitud,
            "rumbo": 1.0 / cfg.f_rumbo,
            "acel": 1.0 / cfg.f_acel,
            "estado": 1.0 / cfg.f_estado,
        }
        self.tick = min(self.periodos.values())
        self._reiniciar_contadores()
        self.cuenta_legacy = 100
        self.bueno_desde = None
        self.ultimo_guardado = None

    def _reiniciar_contadores(self):
        self.proximo = {g: 0.0 for g in self.periodos}
        self.saltos = {g: 0 for g in self.periodos}
        self.publicado = {}  # (grupo, informe) -> t de la última muestra publicada
        self.ultimo_dato = self.reloj()
        self.vistos = {}  # informe -> t de la última muestra vista

    # -- utilidades ---------------------------------------------------------

    @property
    def src(self):
        return self._src() if callable(self._src) else self._src

    def _marca(self, t_muestra, ahora):
        """Instante UTC real de la muestra (no el de envío)."""
        ts = self.utc() - datetime.timedelta(seconds=max(0.0, ahora - t_muestra))
        return ts.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (ts.microsecond // 1000)

    def _update(self, ts, valores):
        return {"source": {"label": "IMU sensor", "src": self.src},
                "timestamp": ts, "values": valores}

    def _toca(self, grupo, ahora):
        if ahora + 1e-9 < self.proximo[grupo]:
            return False
        periodo = self.periodos[grupo]
        siguiente = self.proximo[grupo] + periodo  # en rejilla, aunque el ciclo llegue tarde
        if siguiente <= ahora:  # muy atrasado (o primer ciclo): se recoloca
            siguiente = ahora + periodo
        self.proximo[grupo] = siguiente
        if self.saltos[grupo] > 0:  # devDelayReports: publicar 1 de cada (N + 1)
            self.saltos[grupo] -= 1
            return False
        self.saltos[grupo] = self.cfg.retardo
        return True

    def _fresca(self, muestras, informe, grupo, ahora):
        """Devuelve la muestra si es nueva para este grupo y no está caducada."""
        m = muestras.get(informe)
        if m is None or m.valores is None:
            return None
        if ahora - m.t > self.cfg.edad_maxima(informe):
            return None
        if self.publicado.get((grupo, informe), -1.0) >= m.t:
            return None
        return m

    def _poner_estado(self, texto, error=False):
        if texto != self.ultimo_estado:
            self.ultimo_estado = texto
            self.estado(texto, error)

    # -- un ciclo -----------------------------------------------------------

    def ciclo(self, ahora=None):
        """Lee el sensor y devuelve el delta de este ciclo (o None si no hay nada nuevo)."""
        muestras = self.hw.actualizar()
        if ahora is None:
            ahora = self.reloj()

        # Vigilancia: ¿llega algo nuevo?
        nuevo = False
        for inf, m in muestras.items():
            if m.t > self.vistos.get(inf, -1.0):
                self.vistos[inf] = m.t
                nuevo = True
        if nuevo:
            self.ultimo_dato = ahora
        elif ahora - self.ultimo_dato > self.cfg.vigilancia_seg:
            raise ImuSinDatos("sin muestras nuevas del BNO08x desde hace %.1f s" % (ahora - self.ultimo_dato))

        cfg = self.cfg
        updates = []
        qi = cfg.informe_cuaternion
        q_ultima = muestras.get(qi)
        q_valida = (q_ultima is not None and q_ultima.valores is not None
                    and ahora - q_ultima.t <= cfg.edad_maxima(qi))
        actitud = None
        if q_valida:
            roll, pitch, yaw = actitud_barco(q_ultima.valores, cfg.montaje)
            roll += cfg.rollOffset * math.pi / 180
            pitch += cfg.pitchOffset * math.pi / 180
            yaw = envolver(yaw + cfg.hdgOffset * math.pi / 180)
            actitud = (roll, pitch, yaw)

        # Actitud y rateOfTurn
        if self._toca("actitud", ahora):
            m = self._fresca(muestras, qi, "actitud", ahora)
            if m is not None and actitud is not None:
                self.publicado[("actitud", qi)] = m.t
                roll, pitch, yaw = actitud
                updates.append(self._update(self._marca(m.t, ahora), [
                    {"path": "navigation.attitude", "value": {"pitch": pitch, "roll": roll, "yaw": yaw}}]))
                self.cuenta_legacy -= 1
            if cfg.rate_of_turn and q_valida:
                g = self._fresca(muestras, INF_GIROSCOPIO, "actitud", ahora)
                if g is not None:
                    self.publicado[("actitud", INF_GIROSCOPIO)] = g.t
                    updates.append(self._update(self._marca(g.t, ahora), [
                        {"path": "navigation.rateOfTurn", "value": tasa_de_giro(q_ultima.valores, g.valores)}]))

        # Rumbos
        if self._toca("rumbo", ahora):
            m = self._fresca(muestras, qi, "rumbo", ahora)
            if m is not None and actitud is not None:
                self.publicado[("rumbo", qi)] = m.t
                yaw = actitud[2]
                magnetico = envolver(yaw + cfg.hdgDeviation * math.pi / 180)
                valores = [
                    {"path": "navigation.headingCompass", "value": yaw},
                    {"path": "navigation.headingMagnetic", "value": magnetico},
                ]
                if cfg.decl_needed:
                    valores.append({"path": "navigation.headingTrue",
                                    "value": envolver(magnetico + self.declinacion.valor)})
                updates.append(self._update(self._marca(m.t, ahora), valores))
        if cfg.decl_needed:
            nueva = self.declinacion.tomar_nueva()
            if nueva is not None:
                updates.append(self._update(self._marca(ahora, ahora), [
                    {"path": "navigation.magneticVariation", "value": nueva}]))

        # Aceleración lineal
        if cfg.acel_lineal and self._toca("acel", ahora) and actitud is not None:
            a = self._fresca(muestras, INF_ACEL_LINEAL, "acel", ahora)
            if a is not None:
                self.publicado[("acel", INF_ACEL_LINEAL)] = a.t
                tierra = rotar_a_tierra(q_ultima.valores, a.valores)
                x, y, z = tierra_a_barco(tierra, *actitud)
                updates.append(self._update(self._marca(a.t, ahora), [
                    {"path": "sensors.imu.acceleration", "value": {"x": x, "y": y, "z": z}},
                    {"path": "sensors.imu.verticalAcceleration", "value": tierra[2]}]))

        # Estado: precisión y calibración
        valores_estado = []
        if self._toca("estado", ahora):
            for inf in sorted(muestras):
                m = muestras[inf]
                if ahora - m.t <= cfg.edad_maxima(inf):
                    valores_estado.append({"path": "sensors.imu.calibration." + NOMBRE_INFORME[inf],
                                           "value": m.precision})
            rv = muestras.get(INF_VECTOR_ROTACION)
            if (cfg.fuente_rumbo == FUENTE_ROTACION and rv is not None and rv.extra is not None
                    and ahora - rv.t <= cfg.edad_maxima(INF_VECTOR_ROTACION)):
                valores_estado.append({"path": "sensors.imu.headingAccuracy", "value": rv.extra})
        valores_estado += self._autoguardar(muestras, ahora)
        if cfg.calib_inicio and self.cuenta_legacy <= 0:
            # Igual que antes: con devCalibRequired, cada 100 deltas de actitud
            self.cuenta_legacy = 100
            mag = muestras.get(INF_MAGNETOMETRO)
            if mag is not None:
                valores_estado += [
                    {"path": "sensors.magnetometer.calibration_status", "value": mag.precision},
                    {"path": "sensors.magnetometer.calibration_quality",
                     "value": REPORT_ACCURACY_STATUS[mag.precision]}]
        if valores_estado:
            updates.append(self._update(self._marca(ahora, ahora), valores_estado))

        return {"updates": updates} if updates else None

    def _autoguardar(self, muestras, ahora):
        """Guarda la calibración dinámica (DCD) cuando el magnetómetro y la fuente de
        orientación llevan devAutoSaveSeconds con estado >= devAutoSaveMinStatus."""
        cfg = self.cfg
        if not cfg.autoguardar:
            return []
        clave = [muestras.get(INF_MAGNETOMETRO), muestras.get(cfg.informe_cuaternion)]
        if all(m is not None and m.precision >= cfg.autoguardar_min for m in clave):
            if self.bueno_desde is None:
                self.bueno_desde = ahora
        else:
            self.bueno_desde = None
            return []
        if ahora - self.bueno_desde < cfg.autoguardar_seg:
            return []
        if self.ultimo_guardado is not None and ahora - self.ultimo_guardado < cfg.autoguardar_intervalo:
            return []
        self.ultimo_guardado = ahora  # también si falla: no reintentar en cada ciclo
        try:
            self.hw.guardar_calibracion()
        except Exception as e:  # no es un fallo del bus que obligue a reconectar
            logger.warning("no se pudo guardar la calibración: %s", e)
            return []
        logger.info("calibración del BNO08x guardada (estado >= %d)", cfg.autoguardar_min)
        return [{"path": "sensors.imu.calibration.lastSaved", "value": self._marca(ahora, ahora)}]

    # -- conexión y bucle -----------------------------------------------------

    def _emitir_meta(self):
        ahora = self.reloj()
        self.emitir({"updates": [{"source": {"label": "IMU sensor", "src": self.src},
                                  "timestamp": self._marca(ahora, ahora),
                                  "meta": [{"path": p, "value": v} for p, v in RUTAS_META]}]})

    def _publicar_estado_imu(self, texto):
        ahora = self.reloj()
        self.emitir({"updates": [self._update(self._marca(ahora, ahora), [
            {"path": "sensors.imu.state", "value": texto}])]})

    def reconectar(self):
        """Intenta conectar una vez. True si lo consigue; si no, espera (con espera
        creciente hasta ESPERA_MAX) y devuelve False."""
        try:
            self.hw = self.conectar(self.cfg)
        except Exception as e:
            self.hw = None
            self._poner_estado("IMU no disponible (%s); reintento en %.0f s" % (e, self.espera), True)
            self.dormir(self.espera)
            self.espera = min(self.espera * 2, self.ESPERA_MAX)
            return False
        self.espera = self.ESPERA_MIN
        self.fallos = 0
        self._reiniciar_contadores()
        self._poner_estado("IMU ok (%s, %s)" % (self.src, self.cfg.fuente_rumbo))
        self._emitir_meta()
        self._publicar_estado_imu("ok")
        return True

    def perder(self, motivo):
        """El sensor ha dejado de responder: se descarta y se avisa."""
        self.hw = None
        logger.error("IMU perdido: %s", motivo)
        self._poner_estado("IMU perdido (%s); reconectando" % motivo, True)
        try:
            self._publicar_estado_imu("reconectando")
        except Exception:
            pass

    def paso(self):
        """Una vuelta del bucle principal (conexión, lectura o espera)."""
        if self.hw is None:
            self.reconectar()
            return
        inicio = self.reloj()
        try:
            delta = self.ciclo(inicio)
            self.fallos = 0
        except ImuSinDatos as e:
            self.perder(str(e))
            return
        except Exception as e:  # OSError del bus, RuntimeError/PacketError de la librería
            self.fallos += 1
            logger.warning("fallo leyendo el BNO08x (%d/%d): %r", self.fallos, self.MAX_FALLOS, e)
            if self.fallos >= self.MAX_FALLOS:
                self.perder(repr(e))
            else:
                self.dormir(self.tick)
            return
        if delta is not None:
            self.emitir(delta)
        restante = self.tick - (self.reloj() - inicio)
        if restante > 0:
            self.dormir(restante)

    def ejecutar(self, max_pasos=None):
        n = 0
        while max_pasos is None or n < max_pasos:
            self.paso()
            n += 1
