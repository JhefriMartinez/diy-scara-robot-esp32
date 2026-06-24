#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Instancia del controlador driver PCA9685 por bus I2C estándar
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Constantes físicas del robot SCARA (idénticas a la matriz del informe)
const float L2 = 7.99;
const float L3 = 8.61;

// Calibración sintonizada de microsegundos nominales para los actuadores
// NOTA: Ajusta el offset de calibración física sumando/restando grados AQUÍ
const float OFFSET_BASE_MOTOR = 0.0; // Desfase fino del piñón en grados
const float OFFSET_CODO_MOTOR = 0.0; // Ajuste fino del codo en grados

void setup() {
  // Inicialización de la comunicación serial maestro-esclavo a 115200 bps
  Serial.begin(115200);
  
  // Inicialización del controlador driver PCA9685 en dirección por defecto 0x40
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // Frecuencia de actualización nominal a 50 Hz (período de 20 ms)

  // Mapeo inicial preventivo: Mover el robot a postura de calibración Home seguro
  moverRobotA_Coordenadas(16.60, 0.00, 3.11);
  delay(500);
}

void loop() {
  // Escucha activa y parsing serial asíncrono sin bloqueo de ciclo
  if (Serial.available() > 0) {
    String trama = Serial.readStringUntil('\n');
    trama.trim();
    
    if (trama.length() > 0) {
      int primerComa = trama.indexOf(',');
      int segundoComa = trama.indexOf(',', primerComa + 1);
      
      if (primerComa != -1 && segundoComa != -1) {
        // Extracción de componentes numéricos de la cadena serial
        float px = trama.substring(0, primerComa).toFloat();
        float py = trama.substring(primerComa + 1, segundoComa).toFloat();
        float pz = trama.substring(segundoComa + 1).toFloat();
        
        // Ejecución en vivo de la capa de cálculo cinemático inverso
        moverRobotA_Coordenadas(px, py, pz);
      }
    }
  }
}

void moverRobotA_Coordenadas(float px, float py, float pz) {
  // 1. RESOLUCIÓN DE LA ARTICULACIÓN PRISMÁTICA q3 (Componente vertical matricial)
  if (pz < 0.0) pz = 0.0;
  if (pz > 3.11) pz = 3.11;
  float q3 = 3.11 - pz;

  // 2. RESOLUCIÓN ALGEBRAICA EN PLANO HORIZONTAL (Deducción exacta de las láminas)
  float radio_sq = (px * px) + (py * py);
  
  float num_th2 = radio_sq - (L2 * L2) - (L3 * L3);
  float den_th2 = 2.0 * L2 * L3;
  float cos_th2 = num_th2 / den_th2;
  
  // Protección estricta del firmware contra desborde de memoria (NaN / Imaginarios)
  if (cos_th2 > 1.0) cos_th2 = 1.0;
  if (cos_th2 < -1.0) cos_th2 = -1.0;

  // Configuración por defecto: Codo Abajo (Signo algebraico positivo)
  float sin_th2 = sqrt(1.0 - (cos_th2 * cos_th2));
  float theta2 = atan2(sin_th2, cos_th2);

  // Solución analítica para Theta 1
  float num_th1 = py * (L2 + L3 * cos_th2) - px * (L3 * sin_th2);
  float den_th1 = px * (L2 + L3 * cos_th2) + py * (L3 * sin_th2);
  float theta1 = atan2(num_th1, den_th1);

  // Convertir radianes del cálculo matricial a grados decimales prácticos
  float th1_deg = theta1 * 180.0 / M_PI;
  float th2_deg = theta2 * 180.0 / M_PI;

  // 3. CAPA DE POTENCIA Y EJECUCIÓN FÍSICA (SATURACIÓN Y MAPEO PWM)
  
  // --- ARTICULACIÓN 1: NUEVO MOTOR DE 270 GRADOS ---
  // Rango cinemático [-135, 135] se desvía mediante un offset de +135 a rango positivo [0, 270]
  float th1_positivo = th1_deg + 135.0 + OFFSET_BASE_MOTOR;
  if (th1_positivo < 0.0) th1_positivo = 0.0;
  if (th1_positivo > 270.0) th1_positivo = 270.0;
  
  // Mapeo lineal exacto sobre el rango total de 270 grados
  int pulsoMicros0 = map(th1_positivo, 0, 270, 500, 2500);
  int pasosPCA0 = (pulsoMicros0 * 4096) / 20000;
  pwm.setPWM(0, 0, pasosPCA0); // Canal 0: Base Física

  // --- ARTICULACIÓN 2: MOTOR DEL CODO DE 180 GRADOS ---
  // Rango cinemático [-90, 90] se desvía mediante un offset de +90 a rango positivo [0, 180]
  float th2_positivo = th2_deg + 90.0 + OFFSET_CODO_MOTOR;
  if (th2_positivo < 0.0) th2_positivo = 0.0;
  if (th2_positivo > 180.0) th2_positivo = 180.0;
  
  int pulsoMicros1 = map(th2_positivo, 0, 180, 500, 2500);
  int pasosPCA1 = (pulsoMicros1 * 4096) / 20000;
  pwm.setPWM(1, 0, pasosPCA1); // Canal 1: Codo Físico

  // --- ARTICULACIÓN 3: EJE PRISMÁTICO (PIÑÓN-CREMALLERA) ---
  // Carrera lineal mapeada directamente de 0 a 3.11 cm hacia los límites del motor
  if (q3 < 0.0) q3 = 0.0;
  if (q3 > 3.11) q3 = 3.11;
  
  int pulsoMicros2 = map(q3 * 100.0, 0, 311, 500, 2500); // Conversión a enteros en mm
  int pasosPCA2 = (pulsoMicros2 * 4096) / 20000;
  pwm.setPWM(2, 0, pasosPCA2); // Canal 2: Elevación Lineal
}