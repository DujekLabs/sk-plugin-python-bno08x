"""Prueba de humo de plugin.py entero (main) con módulos de hardware simulados.

Se sustituyen board, busio y adafruit_bno08x por imitaciones que generan informes
SH-2 crudos (bytes en coma fija, como el chip), para comprobar el cableado: escaneo
del bus, activación de informes, lectura, salida de deltas y estado hacia index.js.
"""

import io
import json
import math
import os
import struct
import sys
import types
import unittest

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, RAIZ)

GR = math.pi / 180


def _q14(x):
    return int(round(x * 2 ** 14))


def _q8(x):
    return int(round(x * 2 ** 8))


def _q9(x):
    return int(round(x * 2 ** 9))


class FalsoBNO08X_I2C:
    """Imita lo justo de adafruit_bno08x.BNO08X_I2C (los internos que usa el plugin)."""

    instancias = []

    def __init__(self, i2c, reset=None, address=0x4A, debug=False):
        self._readings = {}
        self.address = address
        self.activados = {}
        self.calibrando = False
        self.guardado = False
        FalsoBNO08X_I2C.instancias.append(self)

    def enable_feature(self, feature_id, report_interval=50000):
        self.activados[feature_id] = report_interval
        self._readings[feature_id] = (0.0, 0.0, 0.0)

    def begin_calibration(self):
        self.calibrando = True

    def save_calibration_data(self):
        self.guardado = True

    @property
    def calibration_status(self):
        return 3

    def _process_report(self, report_id, report_bytes):
        cuenta, escala = {0x02: (3, 2 ** -9), 0x03: (3, 2 ** -4), 0x04: (3, 2 ** -8),
                          0x05: (4, 2 ** -14), 0x08: (4, 2 ** -14)}[report_id]
        self._readings[report_id] = tuple(
            struct.unpack_from("<h", report_bytes, 4 + 2 * i)[0] * escala for i in range(cuenta))

    def _process_available_packets(self, max_packets=None):
        # barco escorado 20° a estribor con el montaje del README (+Y del chip a proa): giro de +20° sobre +Y (H7)
        s, c = math.sin(10 * GR), math.cos(10 * GR)
        rv = bytearray(14)
        rv[0], rv[2] = 0x05, 3
        struct.pack_into("<hhhhh", rv, 4, 0, _q14(s), 0, _q14(c), int(0.02 * 4096))
        gyro = bytearray(10)
        gyro[0], gyro[2] = 0x02, 3
        struct.pack_into("<hhh", gyro, 4, 0, _q9(0.0), _q9(0.05))
        lin = bytearray(10)
        lin[0], lin[2] = 0x04, 2
        struct.pack_into("<hhh", lin, 4, _q8(0.0), _q8(0.0), _q8(0.5))
        mag = bytearray(10)
        mag[0], mag[2] = 0x03, 3
        for rep in (rv, gyro, lin, mag):
            if rep[0] in self.activados:
                self._process_report(rep[0], rep)


class FalsoI2C:
    def __init__(self, scl, sda):
        pass

    def try_lock(self):
        return True

    def unlock(self):
        pass

    def scan(self):
        return [0x4B]

    def deinit(self):
        pass


def instalar_modulos_falsos():
    board = types.ModuleType("board")
    board.SCL, board.SDA = 3, 2
    busio = types.ModuleType("busio")
    busio.I2C = FalsoI2C
    ada = types.ModuleType("adafruit_bno08x")
    ada.BNO_REPORT_GYROSCOPE = 0x02
    ada.BNO_REPORT_MAGNETOMETER = 0x03
    ada.BNO_REPORT_LINEAR_ACCELERATION = 0x04
    ada.BNO_REPORT_ROTATION_VECTOR = 0x05
    ada.BNO_REPORT_GAME_ROTATION_VECTOR = 0x08
    ada_i2c = types.ModuleType("adafruit_bno08x.i2c")
    ada_i2c.BNO08X_I2C = FalsoBNO08X_I2C
    ada.i2c = ada_i2c
    sys.modules.update({"board": board, "busio": busio, "adafruit_bno08x": ada,
                        "adafruit_bno08x.i2c": ada_i2c})


class Basta(Exception):
    pass


class TestPluginEntero(unittest.TestCase):

    def test_main_con_hardware_simulado(self):
        instalar_modulos_falsos()
        import plugin

        opciones = {"imuDevices": [{"devName": 74, "devRefresh": 3, "devAccelRate": 20}]}
        salida = io.StringIO()
        viejos = sys.stdin, sys.stdout
        clase_original = plugin.BucleImu

        class BucleCorto(clase_original):
            pasos = 0

            def paso(self):
                BucleCorto.pasos += 1
                if BucleCorto.pasos > 8:
                    raise Basta()
                super().paso()

        plugin.BucleImu = BucleCorto
        sys.stdin, sys.stdout = io.StringIO(json.dumps(opciones) + "\n"), salida
        try:
            with self.assertRaises(Basta):
                plugin.main()
        finally:
            log = sys.stdout
            sys.stdin, sys.stdout = viejos
            plugin.BucleImu = clase_original
            if hasattr(log, "f"):
                log.f.close()

        lineas = [json.loads(x) for x in salida.getvalue().splitlines() if x]
        estados = [x for x in lineas if "skPluginStatus" in x]
        deltas = [x for x in lineas if "updates" in x]
        self.assertTrue(any("IMU ok (I2C_at[0x4b]" in e["skPluginStatus"] for e in estados))
        bno = FalsoBNO08X_I2C.instancias[-1]
        self.assertEqual(bno.address, 0x4B)  # la que hay en el bus, no la configurada
        self.assertTrue(bno.calibrando)  # calibración dinámica por defecto
        self.assertEqual(bno.activados, {0x05: 25000, 0x03: 200000, 0x02: 50000, 0x04: 25000})
        rutas = {}
        for d in deltas:
            for u in d["updates"]:
                for v in u.get("values", []):
                    rutas[v["path"]] = v["value"]
                    self.assertEqual(u["source"]["src"], "I2C_at[0x4b]")
        self.assertAlmostEqual(rutas["sensors.imu.verticalAcceleration"], 0.5 * math.cos(20 * GR), places=2)
        self.assertAlmostEqual(rutas["navigation.rateOfTurn"], -0.05 * math.cos(20 * GR), places=2)
        self.assertAlmostEqual(rutas["navigation.attitude"]["roll"], 20 * GR, places=3)  # + = a estribor
        self.assertAlmostEqual(rutas["navigation.attitude"]["pitch"], 0.0, places=3)
        self.assertAlmostEqual(rutas["sensors.imu.headingAccuracy"], 0.02, places=3)
        self.assertEqual(rutas["sensors.imu.calibration.linearAcceleration"], 2)
        self.assertEqual(rutas["sensors.imu.state"], "ok")


if __name__ == "__main__":
    unittest.main()
