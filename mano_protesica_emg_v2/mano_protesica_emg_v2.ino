// ===== MANO PROTÉSICA CON AD8232 – VERSIÓN 2 =====
// Comportamiento: haces fuerza -> la mano se cierra y SE MANTIENE cerrada
//                 mientras sigas haciendo fuerza; relajas -> recién se abre.
// Monitor Serie / Serial Plotter a 115200 baudios.
// Enviar la letra  r  por el Monitor Serie para volver a calibrar.
//
// Conexión AD8232: 3.3V -> 3.3V | GND -> GND | OUTPUT -> A0 | LO+ -> D11 | LO- -> D12
#include <Servo.h>

// ---------- Configuración de hardware ----------
const int NUM_DEDOS = 5;
const int servoPins[NUM_DEDOS] = {3, 5, 6, 9, 10};
const int EMG_PIN  = A0;
const int LO_MAS   = 11;
const int LO_MENOS = 12;
const bool USAR_DETECCION_ELECTRODOS = true;   // false si no conectaste LO+ / LO-

const int ANGULO_ABIERTO = 0;
const int ANGULO_CERRADO = 90;
const float PASO_GRADOS  = 4.0;   // velocidad del movimiento (grados cada 20 ms)

// Si algún dedo está montado al revés, cambia su valor a true
const bool invertido[NUM_DEDOS] = {false, false, false, false, false};

// ---------- AJUSTES DE COMPORTAMIENTO (aquí se afina la mano) ----------
// Porcentaje del rango (reposo -> fuerza de calibración):
const float PORC_CERRAR = 0.35;   // nivel para CERRAR la mano
const float PORC_ABRIR  = 0.12;   // nivel para ABRIR. Más bajo = se mantiene cerrada con menos fuerza

// Tiempos de confirmación (evitan que un pico o un bajón momentáneo muevan la mano)
const unsigned long T_CONFIRMAR_CIERRE   = 80;   // ms seguidos con fuerza para cerrar
const unsigned long T_CONFIRMAR_APERTURA = 400;  // ms seguidos relajado para abrir

// ---------- Filtros ----------
const float ALFA_BASE = 0.001;    // línea base: seguimiento lento (~1 s)
const float ALFA_ENV  = 0.01;     // envolvente: suavizado (~100 ms), más estable que antes

Servo servoDedos[NUM_DEDOS];

float baseline   = 512;
float envolvente = 0;
float umbralCerrar = 0;
float umbralAbrir  = 0;
bool  manoCerrada  = false;
bool  calibrando   = true;
float anguloActual = ANGULO_ABIERTO;

bool contandoCierre = false;
bool contandoApertura = false;
unsigned long tInicioCierre = 0;
unsigned long tInicioApertura = 0;

unsigned long tMuestra = 0;
unsigned long tServo   = 0;

// ---------- Procesamiento de una muestra (1 kHz) ----------
void procesarMuestra() {
  int raw = analogRead(EMG_PIN);

  // La línea base solo se actualiza en reposo (o durante la calibración),
  // así la fuerza sostenida no "contamina" la referencia.
  if (calibrando || (!manoCerrada && envolvente < umbralAbrir)) {
    baseline += ALFA_BASE * (raw - baseline);
  }

  float rect  = fabs(raw - baseline);             // rectificación
  envolvente += ALFA_ENV * (rect - envolvente);   // envolvente suavizada
}

// Promedio de la envolvente durante 'duracionMs', muestreando a 1 kHz
float medirPromedio(unsigned long duracionMs) {
  unsigned long inicio = millis();
  unsigned long ultimo = micros();
  float suma = 0;
  long n = 0;
  while (millis() - inicio < duracionMs) {
    if (micros() - ultimo >= 1000) {
      ultimo += 1000;
      procesarMuestra();
      suma += envolvente;
      n++;
    }
  }
  return (n > 0) ? suma / n : 0;
}

void moverDedos(int angulo) {
  for (int i = 0; i < NUM_DEDOS; i++) {
    int a = invertido[i] ? (ANGULO_CERRADO + ANGULO_ABIERTO - angulo) : angulo;
    servoDedos[i].write(a);
  }
}

bool electrodosSueltos() {
  if (!USAR_DETECCION_ELECTRODOS) return false;
  return digitalRead(LO_MAS) == HIGH || digitalRead(LO_MENOS) == HIGH;
}

void cuentaRegresiva(const char* mensaje) {
  Serial.println(mensaje);
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }
}

void calibrar() {
  calibrando  = true;
  manoCerrada = false;
  anguloActual = ANGULO_ABIERTO;
  moverDedos(ANGULO_ABIERTO);

  // Línea base inicial
  long suma = 0;
  for (int i = 0; i < 500; i++) { suma += analogRead(EMG_PIN); delay(1); }
  baseline = suma / 500.0;
  envolvente = 0;

  cuentaRegresiva("CALIBRACION: RELAJA el brazo en");
  medirPromedio(500);                       // estabilizar filtros
  float reposo = medirPromedio(3000);

  cuentaRegresiva("Ahora APRIETA el puno y MANTEN la fuerza (como agarrando un objeto) en");
  medirPromedio(700);                       // descartar el tiempo de reacción
  float contraccion = medirPromedio(2500);

  Serial.println("Ya puedes relajar.");
  medirPromedio(1500);                      // pausa para relajar antes de activar la mano
  Serial.print("Reposo: ");      Serial.println(reposo);
  Serial.print("Contraccion: "); Serial.println(contraccion);

  if (contraccion < reposo * 1.5) {
    Serial.println("AVISO: poca diferencia entre reposo y fuerza.");
    Serial.println("Revisa los electrodos y envia 'r' para recalibrar.");
  }

  float rango  = contraccion - reposo;
  umbralCerrar = reposo + PORC_CERRAR * rango;
  umbralAbrir  = reposo + PORC_ABRIR  * rango;

  Serial.print("Umbral cerrar: "); Serial.println(umbralCerrar);
  Serial.print("Umbral abrir: ");  Serial.println(umbralAbrir);
  Serial.println("Listo. Envia 'r' para recalibrar.");

  contandoCierre = false;
  contandoApertura = false;
  calibrando = false;
  tMuestra = micros();
  tServo   = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(LO_MAS, INPUT);
  pinMode(LO_MENOS, INPUT);

  // Arranque escalonado de servos para no provocar caída de voltaje
  for (int i = 0; i < NUM_DEDOS; i++) {
    servoDedos[i].attach(servoPins[i]);
    servoDedos[i].write(invertido[i] ? ANGULO_CERRADO : ANGULO_ABIERTO);
    delay(250);
  }

  if (electrodosSueltos()) {
    Serial.println("AVISO: electrodos desconectados. Revisa LO+/LO- y los parches.");
  }

  calibrar();
}

void actualizarEstado() {
  unsigned long ahora = millis();
  bool sueltos = electrodosSueltos();

  if (!manoCerrada) {
    // ---- Mano ABIERTA: ¿hay fuerza sostenida para cerrar? ----
    if (!sueltos && envolvente > umbralCerrar) {
      if (!contandoCierre) { contandoCierre = true; tInicioCierre = ahora; }
      if (ahora - tInicioCierre >= T_CONFIRMAR_CIERRE) {
        manoCerrada = true;
        contandoCierre = false;
        contandoApertura = false;
      }
    } else {
      contandoCierre = false;
    }
  } else {
    // ---- Mano CERRADA: se mantiene mientras haya fuerza ----
    // Solo se abre si el brazo está relajado (o electrodos sueltos)
    // de forma CONTINUA durante T_CONFIRMAR_APERTURA.
    bool relajado = sueltos || envolvente < umbralAbrir;
    if (relajado) {
      if (!contandoApertura) { contandoApertura = true; tInicioApertura = ahora; }
      if (ahora - tInicioApertura >= T_CONFIRMAR_APERTURA) {
        manoCerrada = false;
        contandoApertura = false;
        contandoCierre = false;
      }
    } else {
      contandoApertura = false;   // volvió la fuerza: se reinicia el conteo
    }
  }
}

void loop() {
  // Comando de recalibración desde el Monitor Serie
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') calibrar();
  }

  // Muestreo a 1 kHz
  if (micros() - tMuestra >= 1000) {
    tMuestra += 1000;
    procesarMuestra();
  }

  // Control de servos cada 20 ms
  if (millis() - tServo >= 20) {
    tServo += 20;

    actualizarEstado();

    float objetivo = manoCerrada ? ANGULO_CERRADO : ANGULO_ABIERTO;
    if (anguloActual < objetivo) {
      anguloActual += PASO_GRADOS;
      if (anguloActual > objetivo) anguloActual = objetivo;
    } else if (anguloActual > objetivo) {
      anguloActual -= PASO_GRADOS;
      if (anguloActual < objetivo) anguloActual = objetivo;
    }
    moverDedos((int)anguloActual);

    // Formato para el Serial Plotter
    Serial.print("Env:");     Serial.print(envolvente);
    Serial.print(" Cerrar:"); Serial.print(umbralCerrar);
    Serial.print(" Abrir:");  Serial.print(umbralAbrir);
    Serial.print(" Mano:");   Serial.println(manoCerrada ? umbralCerrar * 1.3 : 0);
  }
}
