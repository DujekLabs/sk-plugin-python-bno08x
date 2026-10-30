"""
MIT License

Copyright (c) 2026 dpazz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""


# Rama majareta (DujekLabs): la lógica que no toca el hardware está en imu_nucleo.py
# (probada en test/); este fichero solo conecta el núcleo con el bus I2C, la red
# (declinación NOAA) y la salida hacia index.js.

import sys, json, logging, os, time, threading

from math import pi

import imu_nucleo
from imu_nucleo import Config, BucleImu, crear_clase_bno, esperar_calibracion

_BNO08X_DEFAULT_ADDRESS = 0x4A
_BNO08X_ALTERNATIVE_ADDRESS = 0x4B
_SK_LOCAL = 'http://localhost:3000/signalk/v1/api/vessels/self/navigation/'
_DEBUG_LOG_MAX = 1000000  # debug.log se reescribe al pasar de 1 MB


class CustomAdapter(logging.LoggerAdapter):
    """
    From Python cookbook:
        This example adapter expects the passed in dict-like object to have a
        'pluginid' key, whose value in brackets is prepended to the log message.
    """
    def process(self, msg, kwargs):
        return '[%s] %s' % (self.extra['pluginid'], msg), kwargs


basic_logger = logging.getLogger(__name__)
logger = CustomAdapter(basic_logger, {'pluginid': "sk-py-bno08x"})


class LogLimitado:
    """Destino de sys.stdout: la librería de Adafruit imprime paquetes de depuración
    con print(); si llegaran a la salida real romperían el flujo de deltas que lee
    index.js. Van a debug.log, que se reescribe al pasar de 1 MB."""

    def __init__(self, ruta):
        self.ruta = ruta
        self.f = open(ruta, 'a')

    def write(self, texto):
        try:
            if self.f.tell() > _DEBUG_LOG_MAX:
                self.f.close()
                self.f = open(self.ruta, 'w')
            return self.f.write(texto)
        except Exception:
            return 0

    def flush(self):
        try:
            self.f.flush()
        except Exception:
            pass


class SalidaSignalK:
    """Escribe una línea JSON por mensaje en la salida estándar real (la que lee
    index.js). Los mensajes con 'skPluginStatus' son estado del plugin, no deltas."""

    def __init__(self, f):
        self.f = f
        self.cerrojo = threading.Lock()

    def escribir(self, obj):
        linea = json.dumps(obj) + '\n'
        with self.cerrojo:
            self.f.write(linea)
            self.f.flush()

    def delta(self, delta):
        self.escribir(delta)

    def estado(self, texto, error=False):
        (logger.error if error else logger.info)(texto)
        self.escribir({'skPluginStatus': texto, 'error': bool(error)})


# ---------------------------------------------------------------------------
# Declinación magnética (NOAA o Signal K), en un hilo para no bloquear el bucle
# ---------------------------------------------------------------------------

class Declinacion:
    """Mismo criterio que antes (NOAA con la posición de Signal K; si no, el último
    valor de Signal K; si no, el estimado del schema), pero en segundo plano: antes las
    consultas de red (hasta decenas de segundos) paraban la publicación de la actitud."""

    def __init__(self, cfg, my_source):
        self.valor = cfg.decl_estimate * pi / 180  # estimación del schema en grados
        self.my_source = my_source
        self.intervalo = max(60.0, cfg.decl_interval * 3600)
        self._nueva = None
        self._cerrojo = threading.Lock()

    def arrancar(self):
        threading.Thread(target=self._hilo, name='declinacion', daemon=True).start()

    def tomar_nueva(self):
        with self._cerrojo:
            v, self._nueva = self._nueva, None
            return v

    def _hilo(self):
        while True:
            try:
                v = self.getDeclination()
                if v is not None:
                    with self._cerrojo:
                        self.valor = v
                        self._nueva = v
                    logger.info("navigation.magneticVariation updated")
            except Exception as e:
                logger.warning("declination update failed: %r" % (e,))
            time.sleep(self.intervalo)

    @staticmethod
    def internet_on():
        from urllib.request import urlopen
        try:
            urlopen('https://www.google.com/', timeout=10)
            return True
        except Exception:
            return False

    @staticmethod
    def SignalkPositionAvailable():
        """Return the Signal K response for navigation.position, or None if there is no position."""
        import requests
        try:
            resp = requests.get(_SK_LOCAL + 'position/value', verify=False, timeout=10)
            if resp.status_code != 200 or resp.text == 'null':
                return None
            return resp
        except Exception:
            return None

    def getSignalkVariation(self):
        import requests, ujson
        try:
            # use the last value stored in signalk
            resp = requests.get(_SK_LOCAL + 'magneticVariation/$source', verify=False, timeout=10)
            data = ujson.loads(resp.content)
            if data != self.my_source:
                try:
                    resp = requests.get(_SK_LOCAL + 'magneticVariation/values', verify=False, timeout=10)
                    data = ujson.loads(resp.content)
                    return data[self.my_source]['value']
                except Exception:
                    return self.valor
            resp = requests.get(_SK_LOCAL + 'magneticVariation/value', verify=False, timeout=10)
            return ujson.loads(resp.content)
        except Exception:
            return self.valor  # anyway return the last available value

    def getDeclination(self):
        import requests, ujson
        resp = self.SignalkPositionAvailable()
        if resp is None:
            return self.getSignalkVariation()
        try:
            data = ujson.loads(resp.content)
            lat = "{:.4f}".format(data['latitude'])
            lon = "{:.4f}".format(data['longitude'])
            if not self.internet_on():
                return self.getSignalkVariation()
            NOAA_DeclCalcAPI = ("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination?lat1="
                                + lat + "&lon1=" + lon + "&key=zNEw7&resultFormat=json")
            resp = requests.get(NOAA_DeclCalcAPI, verify=True, timeout=15)
            data = ujson.loads(resp.content)
            for key in data['result']:
                return key['declination'] * pi / 180  # NoAA conventionally responds in degrees
        except Exception:
            pass
        return self.getSignalkVariation()


# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------

def scan_for_bno(i2c, preferred=None, quiet=False):
    while not i2c.try_lock():
        pass
    try:
        devices = i2c.scan()
    finally:
        # Unlock the bus (also when the scan fails or no device is found)
        i2c.unlock()
    if not quiet:
        logger.info("I2C devices found: " + str([hex(i) for i in devices]))
    if preferred in (_BNO08X_DEFAULT_ADDRESS, _BNO08X_ALTERNATIVE_ADDRESS) and preferred in devices:
        return preferred  # the configured address wins when it is on the bus
    if _BNO08X_ALTERNATIVE_ADDRESS in devices:
        return _BNO08X_ALTERNATIVE_ADDRESS
    if _BNO08X_DEFAULT_ADDRESS in devices:
        return _BNO08X_DEFAULT_ADDRESS
    raise ValueError("NO VALID BNO08X ADDRESS FOUND IN THE I2C BUS")


class ConectorBno:
    """Abre el bus, encuentra el BNO08x, lo inicializa y activa los informes.
    Se llama en el arranque y en cada reconexión."""

    def __init__(self):
        import board, busio
        import adafruit_bno08x
        from adafruit_bno08x.i2c import BNO08X_I2C
        # los identificadores del núcleo deben coincidir con los de la librería
        for nombre, valor in (('BNO_REPORT_GYROSCOPE', imu_nucleo.INF_GIROSCOPIO),
                              ('BNO_REPORT_MAGNETOMETER', imu_nucleo.INF_MAGNETOMETRO),
                              ('BNO_REPORT_LINEAR_ACCELERATION', imu_nucleo.INF_ACEL_LINEAL),
                              ('BNO_REPORT_ROTATION_VECTOR', imu_nucleo.INF_VECTOR_ROTACION),
                              ('BNO_REPORT_GAME_ROTATION_VECTOR', imu_nucleo.INF_VECTOR_ROTACION_JUEGO)):
            if getattr(adafruit_bno08x, nombre) != valor:
                raise RuntimeError("unexpected adafruit_bno08x constant " + nombre)
        self.board, self.busio = board, busio
        self.clase = crear_clase_bno(BNO08X_I2C)
        self.i2c = None
        self.primera = True
        self.direccion = None

    def __call__(self, cfg):
        if self.i2c is None:
            self.i2c = self.busio.I2C(self.board.SCL, self.board.SDA)
        try:
            addr = scan_for_bno(self.i2c, cfg.direccion, quiet=not self.primera)
        except Exception:
            self._cerrar_bus()  # el próximo intento vuelve a abrir el bus desde cero
            raise
        if addr != cfg.direccion and self.primera:
            logger.critical("THE CONFIGURED ADDRESS VALUE '"
                            + (hex(cfg.direccion) if isinstance(cfg.direccion, int) else str(cfg.direccion))
                            + "' IS DIFFERENT FROM THE ONE FOUND --> '" + hex(addr) + "'")
        self.direccion = addr
        try:
            bno = self.clase(self.i2c, reset=None, address=addr, debug=False)
            if cfg.calib_dinamica or (cfg.calib_inicio and self.primera):
                bno.iniciar_calibracion()  # no persiste tras un reset del chip
            for informe, intervalo_us in cfg.informes().items():
                bno.enable_feature(informe, intervalo_us)
            if cfg.calib_inicio and self.primera:
                self._calibrar_arranque(bno)
        except Exception:
            self._cerrar_bus()
            raise
        self.primera = False
        return bno

    def _cerrar_bus(self):
        try:
            if self.i2c is not None:
                self.i2c.deinit()
        except Exception:
            pass
        self.i2c = None

    def _calibrar_arranque(self, bno):
        """devCalibRequired: espera hasta 50 s a un estado >= 2 estable 5 s, como antes,
        pero solo guarda la calibración si se ha conseguido."""
        logger.info("startup calibration: move the sensor (up to 50 s)")
        ok, estado = esperar_calibracion(lambda: bno.calibration_status)
        if ok:
            bno.guardar_calibracion()
            logger.info("calibration done and saved, status = %d (%s)"
                        % (estado, imu_nucleo.REPORT_ACCURACY_STATUS[estado]))
        else:
            logger.critical("Calibration timeout !!! status = %d, calibration NOT saved" % estado)


def main():
    salida_real = sys.stdout
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    logging.getLogger("adafruit_bno08x").setLevel(logging.WARNING)
    logging.getLogger("requests").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)

    config = json.loads(sys.stdin.readline())
    dispositivos = config.get("imuDevices") or [{}]
    if len(dispositivos) > 1:
        logger.warning("only the first entry of imuDevices is used")
    cfg = Config(dispositivos[0])

    salida = SalidaSignalK(salida_real)
    sys.stdout = LogLimitado(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'debug.log'))

    try:
        conector = ConectorBno()
    except Exception as e:
        salida.estado("cannot load the BNO08x libraries: %r" % (e,), True)
        raise

    # El src depende de la dirección encontrada; hasta conectar se usa la configurada
    def src():
        addr = conector.direccion if conector.direccion is not None else cfg.direccion
        return 'I2C_at[' + (hex(addr) if isinstance(addr, int) else str(addr)) + ']'

    declinacion = None
    if cfg.decl_needed:
        declinacion = Declinacion(cfg, 'sk-py-bno08x.' + src())
        declinacion.arrancar()

    bucle = BucleImu(cfg, conector, salida.delta, src, estado=salida.estado, declinacion=declinacion)
    salida.estado("starting (%s, heading from %s)" % (src(), cfg.fuente_rumbo))
    while True:
        bucle.paso()
        if declinacion is not None:
            declinacion.my_source = 'sk-py-bno08x.' + src()


if __name__ == "__main__":
    main()
