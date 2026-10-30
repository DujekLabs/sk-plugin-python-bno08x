# Signalk-9axis-bno08x-imu

SignalK plugin to get heading and attitude sensor data from BNO080/85/86 9-axis IMU family (see [here](https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/introduction/))
There are many other sources of similar breakout boards (let you search the web with "BNO086 breakout") available from e-commerce platforms like Aliexpress, Amazon etc.

In the './doc' folder you can find the CEVA BNO08x (the IMU chip at the heart of the breakout) datasheet.

The adoption of QWIIC(STEMMA) cabling is optional but highly recommended if you wish to avoid the soldering of a pin strip and to minimize physical dimensions of hardware layout.

- ### Board image (example)
!["BNO086 Breakout Board Front"](./img/sensor_front.png "BNO086 Breakout Board Front") !["BNO086 Breakout Board Back"](./img/sensor_back.png "BNO086 Breakout Board Back")

- ### Stemma cable (qwiic) with female dupont endings in order to connect to SBC header pins
!["Stemma Cable"](./img/stemma_cable.webp "Stemma Cable")


The project uses the I2C protocol alternative (but also serial or SPI are available) to let the SBC communicate to the breakout board. Due to some glitch on the I2C protocol implementation in BNO08X chips ensure the bus in your SBC is configured in high speed (clock speed set at 400000), eventually configuring it at "dts" level and rebooting. The project is based on Adafruit Blinka Python Library that let you use CircuitPython environment (with some minor limitations) in a full python3 installation like the one available on Raspbian (Raspberry Pi) or Debian (for other SBCs like Radxa Rock series -tested on Rock-4Se-). Adafruit has a "CircuitPython-based" library for BNO08x family IMU sensors  (see [here](https://docs.circuitpython.org/projects/bno08x/en/latest/)).
##### [Note]
        Depending on the wiring of the breakout board, the default I2C address may be 0x4A [74] or 0x4B [75].
        The plugin scans I2C bus to find the actual address (if any) and compares it with the one defined
        in parameters 'schema' logging a warning if the address found is different from the one defined.
        If no BNO08X is on the bus the plugin does not stop: it shows the state and retries
        (waiting 1, 2, 4... up to 30 s) until the sensor answers.

The parameter schema can enable the calculation of Magnetic Variation (declination) using programmatically the NOAA calculator (see [here](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml)) and lat/lon position data given by Signalk server itself (if this option is enabled, the connection to internet must be available or the estimated value defined in schema is used). The query to NoAA calculator is repeated at the configured time interval in plugin schema (default: 5 Hours).
##### [Note]
        In case of calculation of Magnetic Variation from NoAA enabled the position of the vessel
        should be available in Signalk. This sould be ensured by a valid GPS source or by 
        other systems like the one given by the "fixed-position" plugin.    
(see [here](https://www.npmjs.com/package/signalk-fixed-position)) . With release 1.0.4 the missing of 'position' is managed bypassing the query of NoAA.

### Before installing plugin

the following steps are required only for the first installation of the plugin. 

- ####  Install/refresh system-wide packages the project is depending on (Debian/Raspbian)

>   sudo apt update
>
>   sudo apt install python3 python3-pip3 python3-venv python3-click
>
>   sudo apt install i2c-tools libgpiod-dev python3-libgpiod
>
>   sudo apt install python3-adafruit-blinka python3-adafruit-python-shell



- #### Prepare python3 virtual environment
    
    
> <<REMARK
>        
>The plugin looks for the python virtualenv in "~/.env", i.e. the ".env"
>folder in the home directory of the user that runs the Signal K server
>("/home/pi/.env" for the user "pi"). No edit of "index.js" is needed any more.
>If you wish a different virtualenv path, set the environment variable
>SK_PY_BNO08X_VENV for the Signal K server (e.g. in its systemd unit):
>
>       Environment=SK_PY_BNO08X_VENV=/the/chosen/virtual/env/absolute/path
>
>and subsequently restarting signalk since incorrect environment
>should have **blocked plugin start**. By the way, if another
>virtualenv path is chosen the following shell commands
>*shoud be modified consequently* too.
>    
> REMARK
>
>   cd ${home} # usually /home/pi
>
>   mkdir .env
>
>   python3 -m venv .env --system-site-packages

- #### Activate python3 virtual environment and install/refresh the required packages
    
>   source .env/bin/activate
>
>   pip3 install --upgrade adafruit-python-shell click adafruit-blinka # the prompt begins with the (.env) string to indicate virtual env activation
>
>   pip3 install --upgrade adafruit-circuitpython-bno08x

- #### Apply a required workaround to Adafruit_CircuitPython_BNO08x

    due to some misinterpreted packets at startup of the I2C protocol of BNO08x the "raise error" command in .env/lib/python3.11/site-packages/adafruit_bno08x/__init__.py should be commented out as suggested by Andrew123098 (Andrew Brown) on Aug 9, 2024 (see [here](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/issues/49)). The following "sed" command applies the workaround in the file hosted in the virtual environment previously installed.

    the line to be edited is the only one with that pattern (line number 760 as for the latest release
    of the library at the time of publishing of this README.md)

>   sed -i s/raise\ error/#raise\ error/ .env/lib/python3.11/site-packages/adafruit_bno08x/__init__.py

## Rutas de Signal K que publica (rama `majareta`, 1.1.0)

Todas salen en **un único delta por ciclo**; dentro de él, cada `update` lleva el
`timestamp` real de la muestra del sensor que lo origina (no el de envío). Una muestra
que ya se publicó o que es más vieja que su caducidad no se vuelve a publicar.

### Del modelo estándar de Signal K (especificación 1.8.x)

| Ruta | Unidad | Frecuencia por defecto | Notas |
|---|---|---|---|
| `navigation.attitude` `{roll, pitch, yaw}` | rad | 10 Hz (`devAttitudeRate`) | misma fórmula que antes; `yaw` ahora envuelto a [0, 2π) |
| `navigation.rateOfTurn` | rad/s | 10 Hz (`devAttitudeRate`) | **nuevo**; + = cae a estribor |
| `navigation.headingCompass` | rad | 10 Hz (`devHeadingRate`) | |
| `navigation.headingMagnetic` | rad | 10 Hz | headingCompass + desvío (`devHdgDeviation`) |
| `navigation.headingTrue` | rad | 10 Hz | solo con `devDeclRequired`: headingMagnetic + declinación |
| `navigation.magneticVariation` | rad | al arrancar y cada `devDeclInterval` h | solo con `devDeclRequired` |

### Propias del plugin, bajo `sensors.imu.` (no existen en la especificación)

Signal K no tiene ruta estándar para la aceleración lineal ni para la aceleración
vertical de las olas (en la 1.8.x no hay `navigation.acceleration` ni
`environment.*.waves`; lo más cercano es `environment.heave`, en metros, que este
plugin no calcula). Por eso van bajo un prefijo propio, con su `meta` (unidades y
descripción) enviado al conectar:

| Ruta | Unidad | Frecuencia | Qué es |
|---|---|---|---|
| `sensors.imu.acceleration` `{x, y, z}` | m/s² | 10 Hz (`devAccelRate`) | aceleración lineal **sin gravedad** en el marco del barco: x proa, y estribor, z abajo |
| `sensors.imu.verticalAcceleration` | m/s² | 10 Hz (`devAccelRate`) | aceleración vertical sin gravedad en el marco **Tierra**, + arriba (la útil para la altura de ola) |
| `sensors.imu.headingAccuracy` | rad | 1 Hz (`devStatusRate`) | precisión estimada del rumbo (solo con `rotationVector`) |
| `sensors.imu.calibration.<informe>` | 0-3 | 1 Hz | estado de cada informe: `rotationVector`/`gameRotationVector`, `magnetometer`, `gyroscope`, `linearAcceleration` (0 no fiable, 1 baja, 2 media, 3 alta) |
| `sensors.imu.calibration.lastSaved` | fecha ISO | al guardar | última vez que se guardó la calibración (`devAutoSaveCalibration`) |
| `sensors.imu.state` | texto | al cambiar | `ok` / `reconectando` |

Siguen saliendo, igual que antes, si `devCalibRequired` está activo (cada 100 deltas de actitud):
`sensors.magnetometer.calibration_status` (0-3) y `sensors.magnetometer.calibration_quality` (texto).

El estado del plugin en la interfaz de administración de Signal K muestra si el IMU está
`ok`, perdido o reconectando.

### Convención de ejes

- **Marco Tierra**: el del cuaternión del BNO08x, ENU (x Este, y Norte magnético, z arriba).
  `verticalAcceleration` y `rateOfTurn` se calculan girando la medida del sensor con el
  cuaternión, así que **no dependen de cómo esté montado el sensor** ni de la escora:
  con el barco escorado 20°, una aceleración vertical pura sale entera en `verticalAcceleration`.
- **rateOfTurn** = −(componente vertical Tierra de la velocidad angular del giróscopo).
  El giróscopo da + para giros antihorarios; con z arriba eso es caer a babor, de ahí el
  signo: + = a estribor, como pide Signal K.
- **Marco del barco** (`sensors.imu.acceleration`): el de Signal K / aeronáutico, x proa,
  y estribor, z abajo, construido con la **actitud que publica el plugin** (roll, pitch y
  yaw con sus offsets). Es correcto en la medida en que `navigation.attitude` lo sea.
- **Rumbo con `rotationVector`**: la fórmula de actitud (sin cambios desde el plugin
  original) da como rumbo la dirección del eje **+Y del chip**. Si el +Y del sensor no
  apunta a proa, compénsalo con `devHdgOffset` (p. ej. con +X a proa suele hacer falta
  +90°). Con `gameRotationVector` el origen del rumbo es arbitrario.

### Fuente de rumbo (`devHeadingSource`)

- `rotationVector` (**por defecto**): fusión de acelerómetro, giróscopo y magnetómetro
  hecha en el propio BNO08x, referida al Norte magnético, con corrección de la deriva y
  estimación de la precisión del rumbo. Según el datasheet de CEVA es "la estimación de
  orientación más precisa disponible".
- `gameRotationVector` (comportamiento anterior): solo acelerómetro y giróscopo. El
  rumbo no tiene referencia al Norte y deriva con el tiempo, así que `headingCompass`,
  `headingMagnetic` y `headingTrue` no eran rumbos de verdad. Sirve si hay muchas
  perturbaciones magnéticas a bordo y basta un rumbo relativo.

### Calibración

- `devDynamicCalibration` (sí por defecto): activa la calibración dinámica del BNO08x en
  cada arranque y reconexión (no persiste tras un reset del chip).
- `devAutoSaveCalibration` (no por defecto): guarda la calibración en el chip cuando el
  magnetómetro y la fuente de orientación llevan `devAutoSaveSeconds` (30 s) con estado
  ≥ `devAutoSaveMinStatus` (3), como mucho una vez cada `devAutoSaveIntervalMin` (60 min).
- `devCalibRequired` (opción antigua): calibración bloqueante al arrancar (hasta 50 s);
  ahora solo guarda si se consigue estado ≥ 2.

### Robustez

- Si el bus I2C falla 3 lecturas seguidas, o no llega ninguna muestra nueva en
  `devWatchdogSeconds` (3 s), el sensor se da por perdido: se deja de publicar, se
  muestra el estado y se reconecta con espera creciente (1, 2, 4… hasta 30 s). El plugin
  no se cae. Si no hay sensor en el bus al arrancar, se reintenta igual.
- La consulta de declinación (NOAA / Signal K) va en un hilo aparte y ya no detiene la
  publicación de la actitud.
- Lo que la librería de Adafruit imprime por consola va a `debug.log` (máx. 1 MB), nunca
  a la salida que lee Signal K.

### Pruebas

    python3 -m unittest discover -s test -v      # o: npm test

No hace falta el sensor ni la librería de Adafruit: `test/test_imu.py` prueba el núcleo
(`imu_nucleo.py`) con un BNO08x de mentira y `test/test_plugin.py` arranca `plugin.py`
entero con los módulos de hardware simulados.

## Files generated in plugin directory

>The file 'debug.log' collects the DEBUG messages printed by the Adafruit BNO08x library
>(normally sent to stdout) so that only Signal K deltas reach the server.
>The 'debug.log' file is **limited to 1 Mbyte length** and then *overwritten*.

## Acnowledgments

Acknowledgments to Arancino1 and his plugin "signalk-10axis-ros-imu" which inspired this plugin development (see [here](https://github.com/arancino1/signalk-10axis-ros-imu/README.md))
