## 1.1.0-majareta.1
- **Nuevo `navigation.rateOfTurn`** (rad/s, + a estribor) desde el giróscopo calibrado, como componente vertical (marco Tierra) de la velocidad angular: no depende del montaje ni de la escora.
- **Nueva aceleración lineal sin gravedad**: `sensors.imu.acceleration` {x proa, y estribor, z abajo} en m/s² y `sensors.imu.verticalAcceleration` (marco Tierra, + arriba) para medir las olas. Van bajo prefijo propio porque Signal K 1.8.x no tiene rutas estándar para esto (no existen `navigation.acceleration` ni `environment.*.waves`).
- **Rumbo fusionado de verdad**: nueva opción `devHeadingSource`, por defecto `rotationVector` (acelerómetro + giróscopo + magnetómetro, referido al Norte magnético). Antes el rumbo salía del game rotation vector, que no usa el magnetómetro: no tenía referencia al Norte y derivaba. `gameRotationVector` mantiene el comportamiento anterior.
- Nuevas `sensors.imu.headingAccuracy` (rad, la estimación de precisión del rumbo del rotation vector, que la librería de Adafruit descartaba) y `sensors.imu.calibration.<informe>` (0-3 de cada sensor).
- Calibración dinámica: `devDynamicCalibration` (activa por defecto en cada arranque/reconexión) y `devAutoSaveCalibration` (guarda la calibración cuando el estado es bueno de forma sostenida, con límite de frecuencia). `devCalibRequired` ya no guarda la calibración si se agota el tiempo.
- Frecuencias configurables por grupo: `devAttitudeRate`, `devHeadingRate`, `devAccelRate` (10 Hz por defecto) y `devStatusRate` (1 Hz). `devRefresh` queda obsoleto. Cada informe del chip se pide al doble de su frecuencia de publicación.
- Un único delta por ciclo, con el `timestamp` real de cada muestra; no se republican muestras ya enviadas ni caducadas.
- Robustez: tres fallos de I2C seguidos o `devWatchdogSeconds` (3 s) sin muestras nuevas dan el sensor por perdido; se deja de publicar, se muestra el estado (`sensors.imu.state` y el estado del plugin en la interfaz) y se reconecta con espera creciente hasta 30 s, sin matar el plugin.
- La declinación (NOAA / Signal K) se consulta en un hilo aparte: ya no detiene la actitud. Las peticiones HTTP llevan timeout.
- Los rumbos (`headingCompass`, `headingMagnetic`, `headingTrue`) y el `yaw` se envuelven a [0, 2π).
- `index.js`: junta las líneas cortadas entre trozos de la salida de python, una línea mala ya no hace perder las demás y los mensajes de estado de `plugin.py` llegan al estado del plugin.
- La lógica sin hardware vive en `imu_nucleo.py`, con pruebas en `test/` (`python3 -m unittest discover -s test -v` o `npm test`). El antiguo script `test` pasa a `test/smoke.js`.

## Unreleased
- Fixed crash when no BNO08X is on the I2C bus: the scan error was logged and then `NameError: name 'addr' is not defined` killed the python child. The plugin now logs the missing sensor once and rescans the bus every 30 s, so a sensor connected later is picked up.
- Fixed `scan_for_bno()` leaving the I2C bus locked when no sensor was found (`i2c.unlock` was referenced but not called).
- Fixed `TypeError: 'builtin_function_or_method' object is not subscriptable` (`hex[...]` instead of `hex(...)`) when the configured address differs from the one found on the bus.
- The configured I2C address is now used when that device is on the bus (before, 0x4B always won over 0x4A when both answered).
- Fixed the NOAA declination query never running: `getDeclination()` read a global `resp` that was never assigned (the `NameError` was swallowed). `SignalkPositionAvailable()` now returns the response.
- The python virtualenv is no longer hard-coded to `/home/pi/.env`: it defaults to `~/.env` of the user running Signal K and can be overridden with `SK_PY_BNO08X_VENV`. Editing `index.js` after every install/update is no longer needed.
- Failures to start `python3` or an abnormal exit of `plugin.py` are shown in the plugin status (admin UI); `stdin` errors no longer risk an `EPIPE` crash and `stop()` no longer throws when the child never started.

## 1.0.6
- Corrected stop of plugin if start calibration goes timeout (it goes on signalling it bat doesn't stop anyway)
- Improved management of 'position' key  equal to 'null' in Signalk data when declination for the position has to be queried to NOAA 
## 1.0.5
- Added management of pid of child already not running in 'Stop' branch of index.js 
## 1.0.4
- Added management of exception caused by missing of a valid 'position' in SignalK when query of NOAA declination calculator is active in schema
- updated the note in README indicating the required availabiity of a valid 'position' in SignalK when query of NOAA declination calculator is acyive in schema
## 1.0.3
- some code refactoring/cleaning
- some minor bug removed in declination management
## 1.0.2
- Added management of the exception in NOAA calculator returning malformed/unexpected response
- Added a note in README indicating the required availabiity of a valid 'position' in SignalK when query of NOAA declination calculator is active in schema