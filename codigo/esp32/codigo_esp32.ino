#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <driver/mcpwm.h>

// ---------- WiFi ----------
const char* ssid = "Wifi Homer";
const char* password = "Ana101064*";

// ---------- MQTT ----------
const char* mqtt_server = "192.168.1.140";
const char* mqtt_username = "miguelmqtt";
const char* mqtt_password = "miguelmqtt";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- LEDs ----------
#define PIN 2
#define NUMPIXELS 9
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
bool controlLed9Activo = false;
String estadoDiaNoche = "es de noche";

// ---------- Pines sensores ----------
#define SENSOR_PIN 35 // PIR
int sensorPin = 34;   // Sensor de luz
const int trigPin = 25;
const int echoPin = 33;
const int sensor1 = 12;
const int sensor2 = 13;

// ---------- Motor Puente H ----------
const int gpioPWM0A = 18;
const int gpioDir1 = 5;
const int gpioDir2 = 17;
const int gpioPWM1A = 26;
const int gpioDir3 = 27;
const int gpioDir4 = 14;

// ---------- MQTT Topics ----------
const char* ledControlTopic[] = {
  "home/led_1/set", "home/led_2/set", "home/led_3/set",
  "home/led_4/set", "home/led_5/set", "home/led_6/set",
  "home/led_7/set", "home/led_8/set", "home/led_9/set"
};
const char* diaNocheTopic = "casa/luz/dia_noche";
const char* luxTopic = "casa/luz/lux";
const char* motionTopic = "home/motion_sensor";
const char* distanceTopic = "home/ultrasonic/distance";
const char* topic_motor = "home/motor/speed/set";
const char* topic_motor2 = "home/motor2/speed/set";
const char* topic_estado_caja = "home/caja/estado/set";
const char* topic_luz_nocturna = "home/luz/nocturna";


// ---------- Variables ----------
int valorSensor = 0;
float voltaje = 0.0;
float lux = 0.0;
long duration;
float distance;
bool abrirCaja = true;
int velocidadCaja = 100;

unsigned long ultimoMovimiento = 0;
const unsigned long tiempoInactividad = 18000; // 3 minutos en ms
bool lucesApagadasPorInactividad = false;



enum EstadoCaja {
  ESTADO_INVALIDO,
  CAJA_CERRADA,
  CAJA_ABIERTA,
  EN_TRANSICION
};

EstadoCaja ultimoEstado = ESTADO_INVALIDO;

// ---------- Prototipos ----------
void callback(char*, byte*, unsigned int);
void procesarMensajeJSON(char*, StaticJsonDocument<256>&);

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pixels.begin();
  configurarPines();
  conectarWiFi();
  configurarMQTT();
  configurarMCPWM();
  Serial.println("✅ Configuración completada");
  ultimoMovimiento = millis();

}

void configurarPines() {
  pinMode(SENSOR_PIN, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(gpioDir1, OUTPUT);
  pinMode(gpioDir2, OUTPUT);
  pinMode(gpioDir3, OUTPUT);
  pinMode(gpioDir4, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  digitalWrite(gpioDir1, LOW);
  digitalWrite(gpioDir2, LOW);
  digitalWrite(gpioDir3, LOW);
  digitalWrite(gpioDir4, LOW);
}

void conectarWiFi() {
  Serial.println("Iniciando conexión WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Conectado a WiFi");
}

void configurarMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  reconnect();

  for (int i = 0; i < 9; i++) {
    client.subscribe(ledControlTopic[i]);
  }

  client.subscribe(diaNocheTopic);
  client.subscribe(topic_motor);
  client.subscribe(topic_motor2);
  client.subscribe(topic_estado_caja);
  client.subscribe(topic_luz_nocturna);

}

void configurarMCPWM() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpioPWM0A);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 10000;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, gpioPWM1A);
  mcpwm_config_t pwm_config2;
  pwm_config2.frequency = 10000;
  pwm_config2.cmpr_a = 0;
  pwm_config2.cmpr_b = 0;
  pwm_config2.counter_mode = MCPWM_UP_COUNTER;
  pwm_config2.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config2);
}

// ---------- Loop ----------
void loop() {
  if (!client.connected()) reconnect();
  client.loop();
  controlarCaja();

  static unsigned long lastTime = 0;
  if (millis() - lastTime >= 5000) {
    lastTime = millis();
    medirLux();
    medirDistancia();
    detectarMovimiento();

    if (millis() - ultimoMovimiento >= tiempoInactividad && !lucesApagadasPorInactividad) {
      apagarTodasLasLuces();
      lucesApagadasPorInactividad = true;
    }

    if (controlLed9Activo) actualizarLed9();
  }
}


// ---------- Lógica de sensores ----------
void medirLux() {
  valorSensor = analogRead(sensorPin);
  voltaje = (valorSensor / 4095.0) * 3.3;
  lux = (600.0 * voltaje) / 2.3;
  char luxMsg[8];
  dtostrf(lux, 1, 2, luxMsg);
  client.publish(luxTopic, luxMsg);
  Serial.print("Lux: ");
  Serial.println(luxMsg);
}

void medirDistancia() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  char msg[15];
  sprintf(msg, "%.2f cm", distance);
  client.publish(distanceTopic, msg);
  Serial.print("Distancia: ");
  Serial.println(msg);
}

void detectarMovimiento() {
  int sensorValue = digitalRead(SENSOR_PIN);

  // Si se detecta movimiento, actualiza el tiempo
  if (sensorValue == HIGH) {
    ultimoMovimiento = millis();
    lucesApagadasPorInactividad = false; // Marcar que las luces pueden volver a encenderse
  }

  // Publicar al broker MQTT como lo hacías
  char motionState[2];
  sprintf(motionState, "%d", sensorValue);
  client.publish(motionTopic, motionState);

  Serial.print("Movimiento: ");
  Serial.println(sensorValue ? "Detectado" : "No detectado");
}


// ---------- Caja lógica ----------
void controlarCaja() {
  int s1 = digitalRead(sensor1);
  int s2 = digitalRead(sensor2);

  EstadoCaja estadoActual;

  if (s1 == HIGH && s2 == HIGH) {
    estadoActual = CAJA_CERRADA;
  } else if (s1 == LOW && s2 == HIGH) {
    estadoActual = CAJA_ABIERTA;
  } else if (s1 == LOW && s2 == LOW) {
    estadoActual = EN_TRANSICION;
  } else {
    estadoActual = ESTADO_INVALIDO;
  }

  // Solo imprimir si hay cambio de estado
  if (estadoActual != ultimoEstado) {
    switch (estadoActual) {
      case CAJA_CERRADA:
        Serial.println("📦 Caja cerrada");
        if (abrirCaja) {
          Serial.println("🔓 Abriendo...");
        }
        break;
      case CAJA_ABIERTA:
        Serial.println("✅ Caja abierta");
        if (!abrirCaja) {
          Serial.println("🔒 Cerrando...");
        }
        break;
      case EN_TRANSICION:
        Serial.println("↔️ Caja en transición...");
        if (abrirCaja) {
          Serial.println("🔓 Abriendo...");
        } else {
          Serial.println("🔒 Cerrando...");
        }
        break;
      case ESTADO_INVALIDO:
        Serial.println("⚠️ Estado de sensores inválido.");
        break;
    }
    ultimoEstado = estadoActual;
  }

  // Control del motor, esto siempre se ejecuta
  if (estadoActual == CAJA_CERRADA) {
    if (abrirCaja) {
      moverCaja(velocidadCaja);
    } else {
      detenerMotorCaja();
    }
  } else if (estadoActual == CAJA_ABIERTA) {
    if (!abrirCaja) {
      moverCaja(-velocidadCaja);
    } else {
      detenerMotorCaja();
    }
  } else if (estadoActual == EN_TRANSICION) {
    // Aquí ahora sí actúa el motor según abrirCaja
    if (abrirCaja) {
      moverCaja(velocidadCaja);
    } else {
      moverCaja(-velocidadCaja);
    }
  } else {
    detenerMotorCaja();
  }
}


void moverCaja(int velocidad) {
  float duty = map(abs(velocidad), 1, 100, 60, 100);
  digitalWrite(gpioDir1, velocidad > 0);
  digitalWrite(gpioDir2, velocidad < 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
}

void detenerMotorCaja() {
  digitalWrite(gpioDir1, LOW);
  digitalWrite(gpioDir2, LOW);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
}

// ---------- MQTT Callback ----------
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("📩 MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == topic_luz_nocturna) {
  if (message == "encender") {
    controlLed9Activo = true;
    actualizarLed9();
    Serial.println("💡 Luz nocturna activada (azul o rojo según hora)");
  } else if (message == "apagar") {
    controlLed9Activo = false;
    pixels.setPixelColor(8, pixels.Color(0, 0, 0)); // Apaga LED 9
    pixels.show();
    Serial.println("💡 Luz nocturna desactivada");
  }
  return;
}


  if (String(topic) == topic_motor2) {
    procesarMotor(message, gpioDir3, gpioDir4, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    return;
  }

  if (String(topic) == topic_motor) {
    velocidadCaja = constrain(message.toInt(), 0, 100);
    Serial.print("⚙️ Velocidad caja: ");
    Serial.println(velocidadCaja);
    
    if (velocidadCaja == 0) {
    Serial.println("⛔ Velocidad 0 - Deteniendo motor");
    detenerMotorCaja();
  }


    
    return;
  }

  if (String(topic) == topic_estado_caja) {
    abrirCaja = (message == "abrir");
    Serial.print("🧭 Acción recibida: ");
    Serial.println(abrirCaja ? "ABRIR" : "CERRAR");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print("❌ Error JSON: ");
    Serial.println(error.c_str());
    return;
  }

  procesarMensajeJSON(topic, doc);
}

void procesarMotor(String message, int pinDir1, int pinDir2, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t opr) {
  int val = message.toInt();
  if (val == 0) {
    digitalWrite(pinDir1, LOW);
    digitalWrite(pinDir2, LOW);
    mcpwm_set_duty(unit, timer, opr, 0);
    return;
  }

  float duty = map(abs(val), 1, 100, 60, 100);
  digitalWrite(pinDir1, val > 0);
  digitalWrite(pinDir2, val < 0);
  mcpwm_set_duty(unit, timer, opr, duty);
}

// ---------- JSON MQTT ----------
void procesarMensajeJSON(char* topic, StaticJsonDocument<256>& doc) {
  String state = doc["state"];
  
  int r = doc["color"]["r"];
  int g = doc["color"]["g"];
  int b = doc["color"]["b"];

  if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) return;

  actualizarLed(topic, r, g, b);

  // 🔄 Reset por actividad manual desde Home Assistant
  ultimoMovimiento = millis();
  lucesApagadasPorInactividad = false;

  Serial.println("📩 Actividad manual detectada: reiniciando temporizador");
}


void actualizarLed(const char* topic, int r, int g, int b) {
  for (int i = 0; i < 9; i++) {
    if (String(topic) == ledControlTopic[i]) {
      pixels.setPixelColor(i, pixels.Color(r, g, b));
      pixels.show();
      return;
    }
  }
}


void actualizarLed9() {
  if (estadoDiaNoche == "es de noche") {
    pixels.setPixelColor(8, pixels.Color(0, 0, 255));
  } else {
    pixels.setPixelColor(8, pixels.Color(255, 0, 0));
  }
  pixels.show();
}

void apagarTodasLasLuces() {
  for (int i = 0; i < 9; i++) {
    actualizarLed(ledControlTopic[i], 0, 0, 0);  // Apaga cada LED
  }
  Serial.println("🕯️ Todas las luces apagadas por inactividad");
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando al servidor MQTT...");
    if (client.connect("ESP32_Client", mqtt_username, mqtt_password)) {
      Serial.println("✅ Conectado");
      configurarMQTT(); // Re-suscribe tópicos
    } else {
      Serial.print("❌ Error: ");
      Serial.print(client.state());
      Serial.println(" Reintentando...");
      delay(5000);
    }
  }
}
