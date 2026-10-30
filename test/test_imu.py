"""Pruebas del núcleo sin hardware (imu_nucleo.py) con un BNO08x de mentira.

Ejecutar desde la raíz del repositorio:  python3 -m unittest discover -s test -v
Solo usa la biblioteca estándar: no hace falta la librería de Adafruit ni el sensor.
"""

import datetime
import math
import os
import random
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import imu_nucleo as n  # noqa: E402
from imu_nucleo import Muestra  # noqa: E402

GR = math.pi / 180


# ---------------------------------------------------------------------------
# Utilidades: cuaterniones de prueba en el formato de la librería (i, j, k, real)
# ---------------------------------------------------------------------------

def q_eje(eje, angulo):
    s = math.sin(angulo / 2)
    x, y, z = eje
    return (x * s, y * s, z * s, math.cos(angulo / 2))


def q_mult(a, b):
    """Producto a*b con cuaterniones (i, j, k, real)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def a_sensor(q, v_tierra):
    """Lo que mediría el sensor (marco del sensor) si en el marco Tierra hay v_tierra."""
    return n.rotar_a_tierra(q_conj(q), v_tierra)


def q_barco(roll, pitch, yaw, proa=(0, 1, 0), estribor=(1, 0, 0)):
    """El cuaternión de un barco con esa actitud física (rad; roll + estribor abajo, pitch +
    proa arriba, yaw + a estribor), para un chip montado con esos ejes a proa y a estribor y
    la cara arriba: rumbo alrededor de la vertical y, sobre los ejes del propio chip, cabeceo
    alrededor de estribor y balance alrededor de proa (Z-Y-X)."""
    return q_mult(q_eje((0, 0, 1), -yaw), q_mult(q_eje(estribor, pitch), q_eje(proa, roll)))


class Reloj:
    def __init__(self, t=1000.0):
        self.t = t
        self.siestas = []

    def __call__(self):
        return self.t

    def dormir(self, s):
        self.siestas.append(s)
        self.t += s


UTC0 = datetime.datetime(2026, 10, 30, 12, 0, 0, tzinfo=datetime.timezone.utc)


class BnoFalso:
    """BNO08x de mentira: devuelve las muestras que le pongan, con su instante."""

    def __init__(self, reloj):
        self.reloj = reloj
        self.muestras = {}
        self.fallar = 0
        self.guardados = 0
        self.calibraciones = 0

    def poner(self, informe, valores, precision=3, extra=None, t=None):
        self.muestras[informe] = Muestra(valores, precision, self.reloj() if t is None else t, extra)

    def actualizar(self):
        if self.fallar:
            self.fallar -= 1
            raise OSError(121, "Remote I/O error")
        return dict(self.muestras)

    def iniciar_calibracion(self):
        self.calibraciones += 1

    def guardar_calibracion(self):
        self.guardados += 1


def bucle(opciones=None, reloj=None, hw=None):
    reloj = reloj or Reloj()
    hw = hw or BnoFalso(reloj)
    emitidos, estados = [], []
    cfg = n.Config(opciones or {})
    b = n.BucleImu(cfg, lambda c: hw, emitidos.append, "I2C_at[0x4b]",
                   estado=lambda t, e=False: estados.append((t, e)),
                   reloj=reloj, dormir=reloj.dormir,
                   utc=lambda: UTC0 + datetime.timedelta(seconds=reloj.t - 1000.0))
    return b, hw, reloj, emitidos, estados


def valores(delta):
    """{ruta: valor} de todas las updates de un delta."""
    out = {}
    for u in (delta or {}).get("updates", []):
        for v in u.get("values", []):
            out[v["path"]] = v["value"]
    return out


# ---------------------------------------------------------------------------
# Matemáticas
# ---------------------------------------------------------------------------

class TestActitud(unittest.TestCase):
    """H7 (auditoría Codex, 30-oct): la actitud con el montaje explícito, probada con
    movimientos físicos conocidos (antes se comparaba con la fórmula vieja, que con el +Y a
    proa cambiaba balance y cabeceo)."""

    def test_identidad(self):
        for a in n.actitud_barco((0.0, 0.0, 0.0, 1.0)):
            self.assertAlmostEqual(a, 0.0)

    def test_el_caso_de_la_auditoria(self):
        # 20° alrededor de +Y (la proa): la proa sigue al Norte y estribor baja = balance a estribor
        roll, pitch, yaw = n.actitud_barco(q_eje((0, 1, 0), 20 * GR))
        self.assertAlmostEqual(roll, 20 * GR)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_movimientos_fisicos_conocidos(self):
        casos = [((20, 0, 0), "escora 20° a estribor"), ((-15, 0, 0), "escora 15° a babor"),
                 ((0, 10, 0), "proa arriba 10°"), ((0, -8, 0), "proa abajo 8°"),
                 ((0, 0, 30), "cae 30° a estribor"), ((0, 0, 270), "rumbo al Oeste"),
                 ((25, -5, 123), "ciñendo escorado, con la ola, rumbo 123°")]
        for (r, p, y), nombre in casos:
            a = n.actitud_barco(q_barco(r * GR, p * GR, y * GR))
            for x, esperado in zip(a, (r * GR, p * GR, n.envolver(y * GR))):
                self.assertAlmostEqual(x, esperado, msg=nombre)

    def test_cualquier_actitud_vuelve(self):
        rnd = random.Random(7)
        for _ in range(2000):
            r, p, y = rnd.uniform(-60, 60) * GR, rnd.uniform(-60, 60) * GR, rnd.uniform(0, 359.9) * GR
            for x, esperado in zip(n.actitud_barco(q_barco(r, p, y)), (r, p, y)):
                self.assertAlmostEqual(x, esperado, places=9)

    def test_otro_montaje(self):
        # el chip con +X a proa: estribor es -Y (con la cara arriba). Sin ningún offset, lo mismo
        m = n.ejes_montaje("+x", "-y")
        roll, pitch, yaw = n.actitud_barco(q_barco(20 * GR, 5 * GR, 40 * GR, proa=(1, 0, 0), estribor=(0, -1, 0)), m)
        self.assertAlmostEqual(roll, 20 * GR)
        self.assertAlmostEqual(pitch, 5 * GR)
        # el rumbo cuenta desde la proa del barco: con el cuaternión identidad el +X del chip mira al Este
        self.assertAlmostEqual(n.actitud_barco((0.0, 0.0, 0.0, 1.0), m)[2], 90 * GR)

    def test_montaje_imposible(self):
        with self.assertRaises(ValueError):
            n.ejes_montaje("+y", "-y")
        with self.assertRaises(ValueError):
            n.ejes_montaje("+w", "+x")
        with self.assertRaises(ValueError):
            n.Config({"devEjeProa": "+y", "devEjeEstribor": "+y"})

    def test_coherente_con_el_marco_del_barco(self):
        # la actitud y tierra_a_barco son la MISMA convención: el estribor del chip, girado a Tierra y
        # pasado al barco con la actitud, es (0, 1, 0)
        q = q_barco(18 * GR, -7 * GR, 211 * GR)
        roll, pitch, yaw = n.actitud_barco(q)
        for a, b in zip(n.tierra_a_barco(n.rotar_a_tierra(q, (1, 0, 0)), roll, pitch, yaw), (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(a, b)

    def test_cuaternion_sin_normalizar(self):
        q = q_barco(10 * GR, 4 * GR, 30 * GR)
        for x, y in zip(n.actitud_barco(q), n.actitud_barco(tuple(3 * c for c in q))):
            self.assertAlmostEqual(x, y)

    def test_envolver(self):
        self.assertAlmostEqual(n.envolver(-0.1), 2 * math.pi - 0.1)
        self.assertAlmostEqual(n.envolver(2 * math.pi + 0.2), 0.2)
        self.assertEqual(n.envolver(-1e-18), 0.0)
        self.assertTrue(0 <= n.envolver(-2 * math.pi) < 2 * math.pi)


class TestAceleracionTierra(unittest.TestCase):

    def test_escorado_20_vertical_pura(self):
        """Barco escorado 20°: una aceleración vertical pura en la Tierra sale igual."""
        for eje in ((1, 0, 0), (0, 1, 0), (-1, 0, 0)):
            q = q_eje(eje, 20 * GR)
            medida = a_sensor(q, (0.0, 0.0, 1.5))
            self.assertNotAlmostEqual(medida[2], 1.5)  # el sensor, escorado, no la ve vertical
            e, nn, u = n.rotar_a_tierra(q, medida)
            self.assertAlmostEqual(u, 1.5)
            self.assertAlmostEqual(e, 0.0)
            self.assertAlmostEqual(nn, 0.0)

    def test_escora_cabeceo_y_rumbo(self):
        q = q_mult(q_eje((0, 0, 1), -73 * GR), q_mult(q_eje((0, 1, 0), 5 * GR), q_eje((1, 0, 0), 20 * GR)))
        medida = a_sensor(q, (0.3, -0.2, -2.0))
        for a, b in zip(n.rotar_a_tierra(q, medida), (0.3, -0.2, -2.0)):
            self.assertAlmostEqual(a, b)

    def test_marco_barco_escorado_a_estribor(self):
        # roll + = escora a estribor (Signal K). Aceleración hacia arriba de 1 m/s2.
        x, y, z = n.tierra_a_barco((0.0, 0.0, 1.0), 20 * GR, 0.0, 1.234)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, -math.sin(20 * GR))  # estribor apunta algo hacia abajo
        self.assertAlmostEqual(z, -math.cos(20 * GR))  # z del barco es hacia abajo

    def test_marco_barco_rumbo_y_cabeceo(self):
        # Rumbo 90° (proa al Este): una aceleración hacia el Este es x (proa) +
        x, y, z = n.tierra_a_barco((2.0, 0.0, 0.0), 0.0, 0.0, 90 * GR)
        self.assertAlmostEqual((x, y, z)[0], 2.0)
        self.assertAlmostEqual(y, 0.0)
        # Rumbo 0, aceleración al Este = estribor
        self.assertAlmostEqual(n.tierra_a_barco((2.0, 0.0, 0.0), 0.0, 0.0, 0.0)[1], 2.0)
        # Proa arriba 10°: una aceleración hacia arriba tiene componente x + (proa)
        x, y, z = n.tierra_a_barco((0.0, 0.0, 1.0), 0.0, 10 * GR, 0.0)
        self.assertAlmostEqual(x, math.sin(10 * GR))
        self.assertAlmostEqual(z, -math.cos(10 * GR))

    def test_cuaternion_nulo(self):
        with self.assertRaises(ValueError):
            n.rotar_a_tierra((0, 0, 0, 0), (1, 2, 3))


class TestRateOfTurn(unittest.TestCase):

    def test_signo(self):
        identidad = (0.0, 0.0, 0.0, 1.0)
        # giro antihorario visto desde arriba (+ alrededor de Z arriba) = caer a babor
        self.assertAlmostEqual(n.tasa_de_giro(identidad, (0.0, 0.0, 0.1)), -0.1)
        self.assertAlmostEqual(n.tasa_de_giro(identidad, (0.0, 0.0, -0.1)), 0.1)

    def test_escorado_y_coherente_con_el_rumbo(self):
        """Escorado 20°, girando a estribor 0.2 rad/s: rateOfTurn = +0.2 y coincide con
        la derivada del rumbo que publica el plugin."""
        q0 = q_mult(q_eje((0, 0, 1), 40 * GR), q_eje((1, 0, 0), 20 * GR))
        w_tierra = (0.0, 0.0, -0.2)  # horario visto desde arriba = a estribor
        gyro = a_sensor(q0, w_tierra)
        self.assertAlmostEqual(n.tasa_de_giro(q0, gyro), 0.2)
        dt = 1e-4
        q1 = q_mult(q_eje((0, 0, 1), -0.2 * dt), q0)
        y0 = n.actitud_barco(q0)[2]
        y1 = n.actitud_barco(q1)[2]
        self.assertAlmostEqual((y1 - y0) / dt, 0.2, places=4)


class TestInformesCrudos(unittest.TestCase):

    def test_precision_rumbo_del_rotation_vector(self):
        rep = bytearray(14)
        rep[0] = n.INF_VECTOR_ROTACION
        rep[2] = 0b10 | 0b1100  # precisión 2 (los bits altos no cuentan)
        struct.pack_into("<h", rep, 12, int(round(0.05 * 4096)))
        precision, rumbo = n.analizar_extras(n.INF_VECTOR_ROTACION, rep)
        self.assertEqual(precision, 2)
        self.assertAlmostEqual(rumbo, 0.05, places=3)
        rep[0] = n.INF_GIROSCOPIO
        self.assertEqual(n.analizar_extras(n.INF_GIROSCOPIO, rep[:10]), (2, None))

    def test_subclase_guarda_muestras_con_instante(self):
        class Base:
            def __init__(self, *a, **k):
                self._readings = {}
                self.paquetes = []

            def _process_report(self, report_id, report_bytes):
                self._readings[report_id] = (1.0, 2.0, 3.0)

            def _process_available_packets(self, max_packets=None):
                self.paquetes.append(max_packets)
                rep = bytearray(10)
                rep[0] = n.INF_GIROSCOPIO
                rep[2] = 3
                self._process_report(n.INF_GIROSCOPIO, rep)
                self._process_report(0x06, bytearray(10))  # gravedad: no se sigue

        reloj = Reloj(55.0)
        bno = n.crear_clase_bno(Base, reloj=reloj)()
        m = bno.actualizar()
        self.assertEqual(list(m), [n.INF_GIROSCOPIO])
        self.assertEqual(m[n.INF_GIROSCOPIO], Muestra((1.0, 2.0, 3.0), 3, 55.0, None))
        self.assertEqual(bno.paquetes, [40])  # con tope de paquetes por ciclo


# ---------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------

class TestConfig(unittest.TestCase):

    def test_por_defecto(self):
        c = n.Config({"devName": 75, "devRefresh": 3})
        self.assertEqual(c.fuente_rumbo, n.FUENTE_ROTACION)
        self.assertEqual((c.f_actitud, c.f_rumbo, c.f_acel), (10, 10, 10))
        self.assertTrue(c.rate_of_turn and c.acel_lineal and c.calib_dinamica)
        self.assertFalse(c.autoguardar)
        inf = c.informes()
        self.assertEqual(inf[n.INF_VECTOR_ROTACION], 50000)  # 20 Hz
        self.assertEqual(inf[n.INF_GIROSCOPIO], 50000)
        self.assertEqual(inf[n.INF_ACEL_LINEAL], 50000)
        self.assertNotIn(n.INF_VECTOR_ROTACION_JUEGO, inf)

    def test_juego_y_valores_raros(self):
        c = n.Config({"devHeadingSource": "gameRotationVector", "devAccelRate": 20,
                      "devAttitudeRate": -1, "devRateOfTurn": False, "devLinearAcceleration": False,
                      "devHeadingRate": "x"})
        self.assertEqual(c.informe_cuaternion, n.INF_VECTOR_ROTACION_JUEGO)
        self.assertEqual(c.f_actitud, 10)
        self.assertEqual(c.f_rumbo, 10)
        inf = c.informes()
        self.assertEqual(inf[n.INF_VECTOR_ROTACION_JUEGO], 25000)  # 2 x 20 Hz
        self.assertEqual(set(inf), {n.INF_VECTOR_ROTACION_JUEGO, n.INF_MAGNETOMETRO})
        self.assertEqual(n.Config({"devHeadingSource": "otra"}).fuente_rumbo, n.FUENTE_ROTACION)


# ---------------------------------------------------------------------------
# Bucle: delta por ciclo, timestamps, frecuencias, compatibilidad
# ---------------------------------------------------------------------------

Q_ESCORADO = q_mult(q_eje((0, 0, 1), -40 * GR), q_eje((1, 0, 0), 20 * GR))


def cargar(hw, q=Q_ESCORADO, t=None):
    hw.poner(n.INF_VECTOR_ROTACION, q, precision=3, extra=0.035, t=t)
    hw.poner(n.INF_GIROSCOPIO, a_sensor(q, (0.0, 0.0, -0.1)), t=t)
    hw.poner(n.INF_ACEL_LINEAL, a_sensor(q, (0.0, 0.0, 0.8)), t=t)
    hw.poner(n.INF_MAGNETOMETRO, (20.0, 5.0, -40.0), precision=2, t=t)


class TestDelta(unittest.TestCase):

    def test_un_delta_por_ciclo_con_todo(self):
        b, hw, reloj, emitidos, _ = bucle({"devDeclRequired": True, "devDeclEstimate": 2,
                                           "devHdgDeviation": 1})
        b.paso()  # conecta
        self.assertEqual(emitidos[-1]["updates"][0]["values"][0]["value"], "ok")
        self.assertIn("meta", emitidos[-2]["updates"][0])
        emitidos.clear()
        cargar(hw, t=reloj() - 0.02)  # muestras de hace 20 ms
        b.paso()
        self.assertEqual(len(emitidos), 1)  # todo en un único delta
        d = emitidos[0]
        v = valores(d)
        att = v["navigation.attitude"]
        r, p, y = n.actitud_barco(Q_ESCORADO)
        self.assertEqual(att, {"pitch": p, "roll": r, "yaw": y})
        self.assertAlmostEqual(v["navigation.headingCompass"], y)
        self.assertAlmostEqual(v["navigation.headingMagnetic"], n.envolver(y + 1 * GR))
        self.assertAlmostEqual(v["navigation.headingTrue"], n.envolver(y + 3 * GR))
        self.assertAlmostEqual(v["navigation.magneticVariation"], 2 * GR)
        self.assertAlmostEqual(v["navigation.rateOfTurn"], 0.1)
        self.assertAlmostEqual(v["sensors.imu.verticalAcceleration"], 0.8)
        acc = v["sensors.imu.acceleration"]
        self.assertEqual(set(acc), {"x", "y", "z"})
        self.assertAlmostEqual(math.sqrt(acc["x"] ** 2 + acc["y"] ** 2 + acc["z"] ** 2), 0.8)
        self.assertAlmostEqual(v["sensors.imu.headingAccuracy"], 0.035)
        self.assertEqual(v["sensors.imu.calibration.rotationVector"], 3)
        self.assertEqual(v["sensors.imu.calibration.magnetometer"], 2)
        # timestamps reales: los de la muestra, no el de envío
        marcas = {tuple(x["path"] for x in u["values"]): u["timestamp"] for u in d["updates"]}
        self.assertEqual(marcas[("navigation.attitude",)], "2026-10-30T11:59:59.980Z")
        self.assertEqual(marcas[("navigation.rateOfTurn",)], "2026-10-30T11:59:59.980Z")
        self.assertEqual(marcas[("navigation.magneticVariation",)], "2026-10-30T12:00:00.000Z")
        for u in d["updates"]:
            self.assertEqual(u["source"], {"label": "IMU sensor", "src": "I2C_at[0x4b]"})

    def test_no_repite_valores_viejos(self):
        b, hw, reloj, emitidos, _ = bucle({"devStatusRate": 0.01})
        b.paso()
        cargar(hw)
        reloj.t += 0.01
        b.paso()
        emitidos.clear()
        b.paso()  # mismas muestras: nada nuevo que publicar
        self.assertEqual(emitidos, [])
        # muestra caducada (1 s): tampoco se publica aunque sea "nueva" para el grupo
        cargar(hw, t=reloj() - 1.0)
        b.paso()
        self.assertEqual(emitidos, [])

    def test_frecuencias(self):
        b, hw, reloj, emitidos, _ = bucle({"devAttitudeRate": 10, "devHeadingRate": 5, "devAccelRate": 20})
        b.paso()
        emitidos.clear()
        inicio = reloj()
        while reloj() - inicio < 2.0 - 1e-6:
            cargar(hw)  # el sensor siempre tiene muestra nueva
            b.paso()
        cuenta = {}
        for d in emitidos:
            for ruta in valores(d):
                cuenta[ruta] = cuenta.get(ruta, 0) + 1
        self.assertEqual(cuenta["sensors.imu.verticalAcceleration"], 40)
        self.assertEqual(cuenta["navigation.attitude"], 20)
        self.assertEqual(cuenta["navigation.rateOfTurn"], 20)
        self.assertEqual(cuenta["navigation.headingCompass"], 10)
        self.assertEqual(cuenta["sensors.imu.calibration.gyroscope"], 2)
        self.assertEqual(len(emitidos), 40)  # un delta por ciclo, nunca más

    def test_retardo_devDelayReports(self):
        b, hw, reloj, emitidos, _ = bucle({"devDelayReports": 1, "devAccelRate": 10})
        b.paso()
        emitidos.clear()
        for _ in range(10):
            cargar(hw)
            b.paso()
        n_att = sum("navigation.attitude" in valores(d) for d in emitidos)
        self.assertEqual(n_att, 5)  # 1 de cada 2

    def test_game_rotation_vector_y_compatibilidad(self):
        b, hw, reloj, emitidos, _ = bucle({"devHeadingSource": "gameRotationVector",
                                           "devCalibRequired": True, "devHdgOffset": 10})
        b.paso()
        emitidos.clear()
        q = q_eje((0, 0, 1), 30 * GR)
        legacy = None
        for _ in range(101):
            hw.poner(n.INF_VECTOR_ROTACION_JUEGO, q)
            hw.poner(n.INF_MAGNETOMETRO, (1, 2, 3), precision=1)
            b.paso()
            v = valores(emitidos[-1])
            if "sensors.magnetometer.calibration_status" in v:
                legacy = v
        v = valores(emitidos[0])
        self.assertAlmostEqual(v["navigation.headingCompass"], 340 * GR)  # 330 + 10
        self.assertNotIn("sensors.imu.headingAccuracy", v)
        self.assertNotIn("navigation.headingTrue", v)
        self.assertEqual(legacy["sensors.magnetometer.calibration_status"], 1)
        self.assertEqual(legacy["sensors.magnetometer.calibration_quality"], "Low Accuracy")


# ---------------------------------------------------------------------------
# Robustez: reintento, vigilancia, reconexión
# ---------------------------------------------------------------------------

class TestRobustez(unittest.TestCase):

    def test_reintento_al_conectar_con_espera_creciente(self):
        reloj = Reloj()
        hw = BnoFalso(reloj)
        intentos = []

        def conectar(cfg):
            intentos.append(reloj())
            if len(intentos) <= 7:
                raise ValueError("NO VALID BNO08X ADDRESS FOUND IN THE I2C BUS")
            return hw

        estados, emitidos = [], []
        b = n.BucleImu(n.Config({}), conectar, emitidos.append, "s",
                       estado=lambda t, e=False: estados.append((t, e)),
                       reloj=reloj, dormir=reloj.dormir)
        for _ in range(8):
            b.paso()
        self.assertEqual(reloj.siestas, [1, 2, 4, 8, 16, 30, 30])
        self.assertIs(b.hw, hw)
        self.assertTrue(estados[0][1])  # primero, error
        # 7 fallos, 6 avisos: el mismo texto ("reintento en 30 s") no se repite
        self.assertEqual(len([e for e in estados if e[1]]), 6)
        self.assertEqual(estados[-1], ("IMU ok (s, rotationVector)", False))

    def test_fallos_de_bus_y_reconexion(self):
        b, hw, reloj, emitidos, estados = bucle()
        b.paso()
        cargar(hw)
        hw.fallar = 2
        b.paso()
        b.paso()
        self.assertIs(b.hw, hw)  # dos fallos seguidos: aún no se da por perdido
        cargar(hw)
        reloj.t += 0.01
        b.paso()
        self.assertEqual(b.fallos, 0)  # una lectura buena resetea la cuenta
        hw.fallar = 3
        for _ in range(3):
            b.paso()
        self.assertIsNone(b.hw)
        self.assertTrue(estados[-1][1])
        self.assertIn("reconectando", estados[-1][0])
        self.assertEqual(valores(emitidos[-1])["sensors.imu.state"], "reconectando")
        b.paso()  # reconecta
        self.assertIs(b.hw, hw)
        self.assertFalse(estados[-1][1])

    def test_vigilancia_chip_colgado(self):
        b, hw, reloj, emitidos, estados = bucle({"devWatchdogSeconds": 3})
        b.paso()
        cargar(hw)
        b.paso()
        emitidos.clear()
        # el chip deja de mandar: la librería seguiría devolviendo el último valor
        while b.hw is not None and reloj() < 1010:
            b.paso()
        self.assertIsNone(b.hw)
        self.assertGreater(reloj() - 1000, 3.0)
        self.assertLess(reloj() - 1000, 3.5)
        publicadas = set()
        for d in emitidos:
            publicadas |= set(valores(d))
        self.assertNotIn("navigation.attitude", publicadas)  # nada viejo como bueno
        self.assertIn("sin muestras nuevas", estados[-1][0])

    def test_ejecutar_no_muere(self):
        reloj = Reloj()

        def conectar(cfg):
            raise OSError("bus caído")

        b = n.BucleImu(n.Config({}), conectar, lambda d: None, "s", reloj=reloj, dormir=reloj.dormir)
        b.ejecutar(max_pasos=20)
        self.assertIsNone(b.hw)
        self.assertEqual(reloj.siestas[-1], 30)


# ---------------------------------------------------------------------------
# Calibración
# ---------------------------------------------------------------------------

class TestCalibracion(unittest.TestCase):

    def test_autoguardado(self):
        b, hw, reloj, emitidos, _ = bucle({"devAutoSaveCalibration": True, "devAutoSaveSeconds": 2,
                                           "devAutoSaveIntervalMin": 1, "devAutoSaveMinStatus": 3})
        b.paso()
        guardado_en = []
        for _ in range(100):  # 10 s a 10 Hz
            cargar(hw)
            hw.poner(n.INF_MAGNETOMETRO, (1, 2, 3), precision=3)
            antes = hw.guardados
            b.paso()
            if hw.guardados > antes:
                guardado_en.append(reloj())
                self.assertIn("sensors.imu.calibration.lastSaved", valores(emitidos[-1]))
        self.assertEqual(len(guardado_en), 1)  # una vez; no otra hasta pasado el intervalo
        self.assertAlmostEqual(guardado_en[0] - 1000, 2.1, places=5)

    def test_no_guarda_si_no_es_bueno(self):
        b, hw, reloj, _, _ = bucle({"devAutoSaveCalibration": True, "devAutoSaveSeconds": 1})
        b.paso()
        for _ in range(50):
            cargar(hw)  # magnetómetro con precisión 2 < 3
            b.paso()
        self.assertEqual(hw.guardados, 0)

    def test_esperar_calibracion(self):
        reloj = Reloj(0.0)
        estados = iter([0, 1, 2, 2, 1] + [3] * 200)
        ok, e = n.esperar_calibracion(lambda: next(estados), reloj=reloj, dormir=reloj.dormir)
        self.assertTrue(ok)
        self.assertEqual(e, 3)
        reloj = Reloj(0.0)
        ok, e = n.esperar_calibracion(lambda: 1, reloj=reloj, dormir=reloj.dormir)
        self.assertFalse(ok)
        self.assertTrue(50.0 <= reloj() <= 50.2)


if __name__ == "__main__":
    unittest.main()
