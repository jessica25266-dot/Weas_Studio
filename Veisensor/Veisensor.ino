/*
  VEISENSOR - Sensores ultrasónicos (HC-SR04) x3 + acelerómetro MPU6050
  Envía por USB y por Bluetooth (HM-10 / HC-08) el formato que espera la app:

      I:120,D:85,T:300\n      (Izquierdo, Derecho, Trasero, en cm)
      V:1.8\n                 (vibración, m/s²)
      BACHE:alta\n            (o TOPE:media; niveles: baja | media | alta)

  CONEXIONES (Arduino Uno / Nano):
    Sensor IZQUIERDO : TRIG -> D2   ECHO -> D3
    Sensor DERECHO   : TRIG -> D4   ECHO -> D5
    Sensor TRASERO   : TRIG -> D6   ECHO -> D7
    Todos los sensores: VCC -> 5V, GND -> GND

    MPU6050 (I2C, dirección 0x68):
      VCC -> 5V (módulo GY-521) o 3.3V, GND -> GND
      SDA -> A4,  SCL -> A5

    Módulo BLE (HM-10 / HC-08):
      VCC -> 5V (o 3.3V según tu módulo), GND -> GND
      HM-10 TX  -> D10  (Arduino RX por software)
      HM-10 RX  <- D11  (usa divisor de voltaje: 1k en serie + 2k a GND,
                         porque el HM-10 trabaja a 3.3V)

  LIBRERÍAS (Administrador de librerías): Adafruit MPU6050,
  Adafruit Unified Sensor y Adafruit BusIO.
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- Pines ----------
const uint8_t TRIG_I = 2, ECHO_I = 3;   // Izquierdo
const uint8_t TRIG_D = 4, ECHO_D = 5;   // Derecho
const uint8_t TRIG_T = 6, ECHO_T = 7;   // Trasero

const uint8_t BT_RX = 10;               // recibe del TX del módulo BLE
const uint8_t BT_TX = 11;               // envía al RX del módulo BLE

// ---------- Ajustes ultrasónicos ----------
const unsigned long BAUD_USB = 9600;    // la app abre el puerto USB a 9600
const unsigned long BAUD_BT  = 9600;    // velocidad por defecto del HM-10
const unsigned long ECHO_TIMEOUT_US = 18000UL; // ~3 m máx (18 ms)
const int  MAX_CM        = 300;         // sin eco / fuera de rango => 300
const int  MUESTRAS      = 3;           // lecturas por sensor (se toma la mediana)
const unsigned long INTERVALO_MS = 150; // cada cuánto se envía una línea

// ---------- Ajustes del MPU6050 (baches / topes / vibración) ----------
// Desviación respecto a la gravedad (m/s²) para clasificar el golpe.
// Ajústalos con pruebas reales en el vehículo.
const float UMBRAL_BAJA  = 3.0;
const float UMBRAL_MEDIA = 6.0;
const float UMBRAL_ALTA  = 10.0;
const unsigned long VENTANA_MS       = 120;   // tiempo para captar el pico del golpe
const unsigned long ENFRIAMIENTO_MS  = 1500;  // espera entre un evento y el siguiente
const unsigned long MPU_PERIODO_MS   = 5;     // cada cuánto se lee el MPU6050
const bool DEBUG = false;   // true = imprime datos extra por USB (la app NO los entiende)

SoftwareSerial bt(BT_RX, BT_TX);
Adafruit_MPU6050 mpu;

bool  mpuOk = false;
float gBase = 9.81;                 // gravedad medida al arrancar (calibración)
float vibMax = 0;                   // pico de vibración dentro del intervalo
unsigned long ultimoMPU = 0;
unsigned long ultimoEvento = 0;

// Estado de la captura de un golpe
bool  capturando = false;
bool  primeroNegativo = false;      // true: primero "cae" (bache); false: primero "sube" (tope)
float picoGolpe = 0;
unsigned long inicioCaptura = 0;

// Envía una línea completa por USB y por Bluetooth (máx. 20 bytes por paquete BLE).
void enviar(const char* linea) {
  Serial.println(linea);
  bt.println(linea);
}

// Lee el MPU6050 (si ya toca) y detecta baches/topes.
void actualizarMPU() {
  if (!mpuOk) return;
  unsigned long ahora = millis();
  if (ahora - ultimoMPU < MPU_PERIODO_MS) return;
  ultimoMPU = ahora;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Magnitud total: no depende de cómo esté orientado el sensor en el vehículo.
  float mag = sqrt(a.acceleration.x * a.acceleration.x +
                   a.acceleration.y * a.acceleration.y +
                   a.acceleration.z * a.acceleration.z);
  float desv = mag - gBase;         // > 0 sube (tope), < 0 cae (bache)
  float absd = fabs(desv);

  if (absd > vibMax) vibMax = absd;

  if (!capturando) {
    if (absd >= UMBRAL_BAJA && (ahora - ultimoEvento) > ENFRIAMIENTO_MS) {
      capturando = true;
      inicioCaptura = ahora;
      picoGolpe = absd;
      primeroNegativo = (desv < 0);
    }
  } else {
    if (absd > picoGolpe) picoGolpe = absd;
    if (ahora - inicioCaptura >= VENTANA_MS) {
      capturando = false;
      ultimoEvento = ahora;

      const char* nivel = (picoGolpe >= UMBRAL_ALTA)  ? "alta"
                        : (picoGolpe >= UMBRAL_MEDIA) ? "media" : "baja";
      char linea[16];
      snprintf(linea, sizeof(linea), "%s:%s", primeroNegativo ? "BACHE" : "TOPE", nivel);
      enviar(linea);
    }
  }
}

// Espera sin dejar de leer el MPU6050, para no perder golpes breves.
void esperar(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) actualizarMPU();
}

// Lee la distancia en cm de un HC-SR04. Devuelve MAX_CM si no hay eco.
int leerCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) return MAX_CM;          // timeout: nada detectado

  int cm = (int)(dur / 58UL);           // µs -> cm
  if (cm < 2) cm = 2;                   // zona ciega del HC-SR04
  if (cm > MAX_CM) cm = MAX_CM;
  return cm;
}

// Mediana de varias lecturas para filtrar ruido y lecturas falsas.
int distanciaFiltrada(uint8_t trig, uint8_t echo) {
  int v[MUESTRAS];
  for (int i = 0; i < MUESTRAS; i++) {
    v[i] = leerCm(trig, echo);
    esperar(10);                        // deja morir el eco anterior (y sigue leyendo el MPU)
  }
  // ordenar (pocos elementos, burbuja simple)
  for (int i = 0; i < MUESTRAS - 1; i++)
    for (int j = i + 1; j < MUESTRAS; j++)
      if (v[j] < v[i]) { int t = v[i]; v[i] = v[j]; v[j] = t; }
  return v[MUESTRAS / 2];
}

void setup() {
  pinMode(TRIG_I, OUTPUT); pinMode(ECHO_I, INPUT);
  pinMode(TRIG_D, OUTPUT); pinMode(ECHO_D, INPUT);
  pinMode(TRIG_T, OUTPUT); pinMode(ECHO_T, INPUT);

  Serial.begin(BAUD_USB);
  bt.begin(BAUD_BT);

  Wire.begin();
  if (mpu.begin()) {                    // dirección por defecto 0x68
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calibración: promedio de la magnitud con el vehículo quieto al encender.
    delay(100);
    float suma = 0;
    for (int i = 0; i < 40; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      suma += sqrt(a.acceleration.x * a.acceleration.x +
                   a.acceleration.y * a.acceleration.y +
                   a.acceleration.z * a.acceleration.z);
      delay(10);
    }
    gBase = suma / 40.0;
  }
  // Si el MPU6050 no responde, el sketch sigue funcionando solo con los ultrasónicos.

  if (DEBUG) {
    Serial.print(F("MPU6050: ")); Serial.println(mpuOk ? F("OK") : F("NO ENCONTRADO (revisa A4/A5)"));
    Serial.print(F("Gravedad base: ")); Serial.println(gBase);
  }
}

void loop() {
  actualizarMPU();                      // sigue leyendo el acelerómetro entre envíos

  static unsigned long ultimo = 0;
  if (millis() - ultimo < INTERVALO_MS) return;
  ultimo = millis();

  // Se miden uno por uno (no al mismo tiempo) para evitar interferencia entre sensores.
  int i = distanciaFiltrada(TRIG_I, ECHO_I);
  int d = distanciaFiltrada(TRIG_D, ECHO_D);
  int t = distanciaFiltrada(TRIG_T, ECHO_T);

  // Línea con el formato exacto que lee la app: "I:120,D:85,T:300"
  char linea[24];
  snprintf(linea, sizeof(linea), "I:%d,D:%d,T:%d", i, d, t);
  enviar(linea);

  // Vibración (pico del intervalo) en su propia línea: "V:1.8"
  if (mpuOk) {
    int v10 = (int)(vibMax * 10.0 + 0.5);
    char lineaV[12];
    snprintf(lineaV, sizeof(lineaV), "V:%d.%d", v10 / 10, v10 % 10);
    enviar(lineaV);
    vibMax = 0;
  }
}
