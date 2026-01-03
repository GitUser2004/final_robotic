/*
 * ROBOT HOSPITAL - SENSOR VL53L0X 
 */

#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// micro-ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>

// ═══════════════════════════════════════════════════════════
// PINES
// ═══════════════════════════════════════════════════════════
#define SERVO_PIN 18
#define LED_MISSION 2
#define LED_PAUSE 4
#define BTN_ACK 5

// TB6612
#define AIN1 33
#define AIN2 32
#define PWMA 25
#define BIN1 14
#define BIN2 12
#define PWMB 13
#define STBY 26

// Encoders
#define ENCODER_LEFT_A 23
#define ENCODER_LEFT_B 19
#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 35

// ═══════════════════════════════════════════════════════════
// CONFIGURACIÓN SENSOR 
// ═══════════════════════════════════════════════════════════
#define ANGLE_CENTER 90
#define ANGLE_RIGHT 10
#define ANGLE_LEFT 170
#define SCAN_DELAY 700 
#define SERVO_STABILIZATION_DELAY 250  // Pausa extra después de mover servo

//Control de estado del sensor
bool sensor_paused = false;  // Pausar durante barridos visuales
bool sensor_needs_reset = false;
unsigned long last_sensor_reset = 0;
const unsigned long SENSOR_RESET_INTERVAL = 5000;  // Reintentar cada 5s
int consecutive_sensor_errors = 0;
const int MAX_SENSOR_ERRORS = 5;

// Filtro de distancias
float last_valid_distance = 2.0;
unsigned long last_valid_reading = 0;
const unsigned long READING_TIMEOUT = 2000;  // 2s sin lectura válida = problema

// ═══════════════════════════════════════════════════════════
// WIFI Y MQTT
// ═══════════════════════════════════════════════════════════

// in this secction is neccesary to define the WiFi credentials and server configuration from the local network
const char* ssid = "HOME-GT";
const char* password = "Casa.24GT";
const char* agent_ip = "192.168.100.87";
const size_t agent_port = 8888;

const char* mqtt_server = "broker.hivemq.com";
#define TOPIC_COMMAND "robot/command"
#define TOPIC_STATUS  "robot/status"
#define TOPIC_PING    "robot/ping"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastPing = 0;
const unsigned long pingInterval = 2000;

// ═══════════════════════════════════════════════════════════
// ESTADOS DE MISIÓN
// ═══════════════════════════════════════════════════════════
enum MissionState {
  IDLE, BUSCANDO, PAUSA, VOLVER
};

MissionState missionState = IDLE;
String target_color = "NINGUNO";
String current_color = "NINGUNO";
String delivered_color = "NINGUNO";

// ═══════════════════════════════════════════════════════════
// SERVIDOR WEB Y OBJETOS
// ═══════════════════════════════════════════════════════════
WebServer server(80);
VL53L0X sensor;
Servo servo;

// ═══════════════════════════════════════════════════════════
// CONSTANTES ROBOT
// ═══════════════════════════════════════════════════════════
#define WHEEL_DIAMETER 0.04
#define METERS_PER_PULSE 0.00003574

const int MOTOR_FORWARD = 1;
const int MOTOR_BACKWARD = -1;
const int MOTOR_STOP = 0;

const float LEFT_MOTOR_FACTOR = 0.54;
const float RIGHT_MOTOR_FACTOR = 0.53;
const unsigned long CMD_VEL_TIMEOUT = 500;

volatile long encoder_left_pulses = 0;
volatile int encoder_left_last_state = 0;
volatile long encoder_right_pulses = 0;
volatile int encoder_right_last_state = 0;

float total_distance = 0.0;
float left_distance = 0.0;
float right_distance = 0.0;

unsigned long last_cmd_vel_time = 0;
bool scanning_mode = false;
int scan_step = 0;
int current_servo_angle = 90;
unsigned long last_servo_move = 0; 

// ═══════════════════════════════════════════════════════════
// MICRO-ROS
// ═══════════════════════════════════════════════════════════
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t distance_pub;
rcl_publisher_t distance_left_pub;
rcl_publisher_t distance_right_pub;
rcl_publisher_t odom_pub;
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t scan_cmd_sub;
rcl_subscription_t servo_angle_sub;
rclc_executor_t executor;
rcl_timer_t distance_timer;
rcl_timer_t safety_timer;
rcl_timer_t odom_timer;

std_msgs__msg__Float32 distance_msg;
std_msgs__msg__Float32 distance_left_msg;
std_msgs__msg__Float32 distance_right_msg;
std_msgs__msg__Float32 odom_msg;
std_msgs__msg__Bool scan_cmd_msg;
std_msgs__msg__Int16 servo_angle_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// ═══════════════════════════════════════════════════════════
// FUNCIÓN: REINICIAR SENSOR
// ═══════════════════════════════════════════════════════════
bool reset_sensor() {
  Serial.println("Reiniciando sensor VL53L0X...");
  
  sensor.setTimeout(500);
  
  if (!sensor.init()) {
    Serial.println("Fallo al reiniciar sensor");
    return false;
  }
  
  sensor.setMeasurementTimingBudget(33000);
  delay(50);  // Pausa para estabilización
  
  // Limpiar lecturas previas
  for (int i = 0; i < 3; i++) {
    sensor.readRangeSingleMillimeters();
    delay(20);
  }
  
  consecutive_sensor_errors = 0;
  last_sensor_reset = millis();
  Serial.println("Sensor reiniciado exitosamente");
  
  return true;
}

// ═══════════════════════════════════════════════════════════
// LECTURA ROBUSTA DE DISTANCIA
// ═══════════════════════════════════════════════════════════
float get_distance() {
  // Si el sensor está pausado, retornar última lectura válida
  if (sensor_paused) {
    return last_valid_distance;
  }
  
  // Si se movió el servo recientemente, esperar estabilización
  if (millis() - last_servo_move < SERVO_STABILIZATION_DELAY) {
    return last_valid_distance;
  }
  
  // Si hay muchos errores consecutivos, intentar reset
  if (consecutive_sensor_errors >= MAX_SENSOR_ERRORS) {
    if (millis() - last_sensor_reset > SENSOR_RESET_INTERVAL) {
      if (reset_sensor()) {
        consecutive_sensor_errors = 0;
      }
    }
    return last_valid_distance;
  }
  
  // Intentar lectura con timeout
  uint16_t distance_mm = sensor.readRangeSingleMillimeters();
  
  // Verificar timeout
  if (sensor.timeoutOccurred()) {
    consecutive_sensor_errors++;
    Serial.printf("⚠️ Sensor timeout (errores: %d/%d)\n", 
                  consecutive_sensor_errors, MAX_SENSOR_ERRORS);
    
    // Si timeout pero tenemos lectura reciente, usarla
    if (millis() - last_valid_reading < READING_TIMEOUT) {
      return last_valid_distance;
    }
    return 2.0;
  }
  
  float distance_m = distance_mm / 1000.0;
  // Rango válido: 3cm a 2m 
  if (distance_m < 0.03) {
    // Objeto MUY cerca - probablemente pegado al sensor
    consecutive_sensor_errors++;
    Serial.printf("⚠️ Objeto muy cerca: %.1fcm\n", distance_m * 100);
    return 0.03;  // Retornar mínimo detectable en vez de 2.0
  }
  
  if (distance_m > 2.0) {
    consecutive_sensor_errors++;
    return 2.0;
  }
  
  //Lectura válida - resetear contador de errores
  consecutive_sensor_errors = 0;
  last_valid_distance = distance_m;
  last_valid_reading = millis();
  
  return distance_m;
}

// ═══════════════════════════════════════════════════════════
// UTILIDADES
// ═══════════════════════════════════════════════════════════
String norm(String s) {
  s.trim();
  s.toUpperCase();
  return s;
}

bool resolveTargetColor(const JsonDocument& doc, String &outColor) {
  outColor = "NINGUNO";
  if (!doc.containsKey("room")) return false;

  if (doc["room"].is<int>()) {
    int n = doc["room"].as<int>();
    if (n == 1) { outColor = "ROJO"; return true; }
    if (n == 2) { outColor = "VERDE"; return true; }
    if (n == 3) { outColor = "AZUL"; return true; }
    return false;
  }

  if (doc["room"].is<const char*>()) {
    String r = norm(String((const char*)doc["room"]));
    if (r == "1" || r == "A") { outColor = "ROJO"; return true; }
    if (r == "2" || r == "B") { outColor = "VERDE"; return true; }
    if (r == "3" || r == "C") { outColor = "AZUL"; return true; }
    if (r == "ROJO" || r == "VERDE" || r == "AZUL") {
      outColor = r;
      return true;
    }
  }

  return false;
}

void publishStatusCompleted(const String& reason, const String& color) {
  StaticJsonDocument<160> statusDoc;
  statusDoc["status"] = "COMPLETED";
  statusDoc["reason"] = reason;
  statusDoc["color"] = color;

  char buffer[160];
  serializeJson(statusDoc, buffer);
  mqttClient.publish(TOPIC_STATUS, buffer);
}

// ═══════════════════════════════════════════════════════════
// MQTT CALLBACK
// ═══════════════════════════════════════════════════════════
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) != TOPIC_COMMAND) return;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) {
    Serial.println("Error parseando JSON");
    return;
  }

  const char* cmdC = doc["cmd"];
  if (!cmdC) return;

  String cmd = norm(String(cmdC));

  if (cmd == "START_MISSION" && missionState == IDLE) {
    String target;
    if (!resolveTargetColor(doc, target)) {
      Serial.println("START_MISSION: room inválida");
      return;
    }

    target_color = target;
    current_color = "NINGUNO";
    missionState = BUSCANDO;

    Serial.printf("Misión iniciada: Buscar %s\n", target_color.c_str());
    
    digitalWrite(LED_MISSION, HIGH);
    digitalWrite(LED_PAUSE, LOW);
    
    // Asegurar que el sensor esté activo
    sensor_paused = false;
    consecutive_sensor_errors = 0;
  }
}

// ═══════════════════════════════════════════════════════════
// MQTT RECONNECT
// ═══════════════════════════════════════════════════════════
void reconnect_mqtt() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32_Hospital_Robot")) {
      mqttClient.subscribe(TOPIC_COMMAND);
      Serial.println("✓ MQTT conectado");
    } else {
      delay(2000);
    }
  }
}

// ═══════════════════════════════════════════════════════════
// HTTP ENDPOINTS
// ═══════════════════════════════════════════════════════════
void handle_color() {
  if (!server.hasArg("c")) {
    server.send(400, "text/plain", "Missing param c");
    return;
  }

  current_color = norm(server.arg("c"));

  if (missionState == BUSCANDO &&
      (current_color == "ROJO" || current_color == "VERDE" || current_color == "AZUL")) {
    
    Serial.printf("Llegó a habitación: %s\n", current_color.c_str());
    digitalWrite(LED_PAUSE, HIGH);
    missionState = PAUSA;
  }
  else if (missionState == VOLVER && current_color == "AMARILLO") {
    Serial.println("Llegó a almacén (AMARILLO)");
    
    digitalWrite(LED_MISSION, LOW);
    digitalWrite(LED_PAUSE, LOW);

    publishStatusCompleted("RETURN_YELLOW", current_color);

    missionState = IDLE;
    target_color = "NINGUNO";
    current_color = "NINGUNO";
  }

  server.send(200, "text/plain", "OK");
}

void handle_target() {
  if (missionState == BUSCANDO) {
    server.send(200, "text/plain", target_color);
  } else if (missionState == VOLVER) {
    server.send(200, "text/plain", "AMARILLO");
  } else {
    server.send(200, "text/plain", "NINGUNO");
  }
}

void handle_mission_state() {
  String state_str;
  switch (missionState) {
    case IDLE: state_str = "IDLE"; break;
    case BUSCANDO: state_str = "BUSCANDO"; break;
    case PAUSA: state_str = "PAUSA"; break;
    case VOLVER: state_str = "VOLVER"; break;
  }
  server.send(200, "text/plain", state_str);
}

void handle_status() {
  String state_str;
  switch (missionState) {
    case IDLE: state_str = "IDLE"; break;
    case BUSCANDO: state_str = "BUSCANDO"; break;
    case PAUSA: state_str = "PAUSA"; break;
    case VOLVER: state_str = "VOLVER"; break;
  }

  String json = "{";
  json += "\"state\":\"" + state_str + "\",";
  json += "\"target\":\"" + target_color + "\",";
  json += "\"current\":\"" + current_color + "\",";
  json += "\"odom\":" + String(total_distance, 3) + ",";
  json += "\"servo\":" + String(current_servo_angle) + ",";
  json += "\"sensor_errors\":" + String(consecutive_sensor_errors);
  json += "}";
  
  server.send(200, "application/json", json);
}

// ═══════════════════════════════════════════════════════════
// ISR ENCODERS
// ═══════════════════════════════════════════════════════════
void IRAM_ATTR encoder_left_isr() {
  int state_a = digitalRead(ENCODER_LEFT_A);
  if (state_a == HIGH && encoder_left_last_state == LOW) {
    encoder_left_pulses++;
  }
  encoder_left_last_state = state_a;
}

void IRAM_ATTR encoder_right_isr() {
  int state_a = digitalRead(ENCODER_RIGHT_A);
  if (state_a == HIGH && encoder_right_last_state == LOW) {
    encoder_right_pulses++;
  }
  encoder_right_last_state = state_a;
}

// ═══════════════════════════════════════════════════════════
// CONTROL MOTORES
// ═══════════════════════════════════════════════════════════
void set_motor(int motor, int direction, int pwm) {
  if (motor == 0) {
    pwm = constrain(pwm * LEFT_MOTOR_FACTOR, 0, 255);
  } else {
    pwm = constrain(pwm * RIGHT_MOTOR_FACTOR, 0, 255);
  }
  
  if (motor == 0) {
    if (direction == MOTOR_FORWARD) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    } else if (direction == MOTOR_BACKWARD) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    } else {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
    }
    analogWrite(PWMA, pwm);
  } else {
    if (direction == MOTOR_FORWARD) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    } else if (direction == MOTOR_BACKWARD) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    } else {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMB, pwm);
  }
}

void stop_motors() {
  set_motor(0, MOTOR_STOP, 0);
  set_motor(1, MOTOR_STOP, 0);
}

// ═══════════════════════════════════════════════════════════
//CALLBACKS MICRO-ROS 
// ═══════════════════════════════════════════════════════════
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = -msg->linear.x;
  float angular_z = msg->angular.z;
  
  last_cmd_vel_time = millis();
  
  // Si los motores están activos, el sensor puede leer normalmente
  sensor_paused = false;
  
  if (fabs(angular_z) > 0.1) {
    int turn_pwm = (int)(fabs(angular_z) * 127.5);
    turn_pwm = constrain(turn_pwm, 80, 220);
    
    if (angular_z > 0) {
      set_motor(0, MOTOR_BACKWARD, turn_pwm);
      set_motor(1, MOTOR_FORWARD, turn_pwm);
    } else {
      set_motor(0, MOTOR_FORWARD, turn_pwm);
      set_motor(1, MOTOR_BACKWARD, turn_pwm);
    }
  } 
  else if (fabs(linear_x) > 0.1) {
    int pwm = abs((int)(linear_x * 255.0));
    pwm = constrain(pwm, 100, 180);
    
    if (linear_x > 0) {
      set_motor(0, MOTOR_FORWARD, pwm);
      set_motor(1, MOTOR_FORWARD, pwm);
    } else {
      set_motor(0, MOTOR_BACKWARD, pwm);
      set_motor(1, MOTOR_BACKWARD, pwm);
    }
  } 
  else {
    stop_motors();
  }
}

void scan_cmd_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  scanning_mode = msg->data;
  
  if (scanning_mode) {
    scan_step = 0;
    // PAUSAR SENSOR DURANTE BARRIDO DE OBSTÁCULOS
    sensor_paused = true;
    Serial.println("Barrido iniciado - sensor pausado");
  }
}

void servo_angle_callback(const void * msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  int angle = constrain(msg->data, 0, 180);
  
  if (!scanning_mode) {
    servo.write(angle);
    current_servo_angle = angle;
    last_servo_move = millis();  //Registrar movimiento
    
    // Si el servo se mueve mucho (barrido visual), pausar sensor
    if (abs(angle - 90) > 30) {
      sensor_paused = true;
    } else {
      sensor_paused = false;
    }
  }
}

void safety_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  
  if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT) {
    stop_motors();
  }
}

void distance_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  
  if (scanning_mode) {
    // BARRIDO DE OBSTÁCULOS CON ESTABILIZACIÓN
    if (scan_step == 0) {
      servo.write(ANGLE_RIGHT);  // Servo mira DERECHA física
      last_servo_move = millis();
      delay(SCAN_DELAY);
      delay(SERVO_STABILIZATION_DELAY);  
      float dist_right = get_distance();
      distance_right_msg.data = dist_right;  
      rcl_publish(&distance_right_pub, &distance_right_msg, NULL);
      Serial.printf("Derecha física: %.2fm\n", dist_right);
      scan_step = 1;
    }
    else if (scan_step == 1) {
      servo.write(ANGLE_LEFT);  // Servo mira IZQUIERDA física
      last_servo_move = millis();
      delay(SCAN_DELAY);
      delay(SERVO_STABILIZATION_DELAY); 
      
      float dist_left = get_distance();
      distance_left_msg.data = dist_left;  
      rcl_publish(&distance_left_pub, &distance_left_msg, NULL);
      
      Serial.printf("Izquierda física: %.2fm\n", dist_left);
      scan_step = 2;
    }
    else if (scan_step == 2) {
      servo.write(ANGLE_CENTER);
      last_servo_move = millis();
      delay(SCAN_DELAY);
      delay(SERVO_STABILIZATION_DELAY);  
      
      float dist_center = get_distance();
      distance_msg.data = dist_center;
      rcl_publish(&distance_pub, &distance_msg, NULL);
      
      Serial.printf("Centro: %.2fm\n", dist_center);
      
      scan_step = 0;
      scanning_mode = false;
      current_servo_angle = ANGLE_CENTER;
      
      // REACTIVAR SENSOR después del barrido
      sensor_paused = false;
      Serial.println("✓ Barrido completo - sensor reactivado");
      
      //Esperar un poco más antes de permitir movimiento
      delay(200);
    }
  }
  else {
    //LECTURA NORMAL - solo si no está pausado
    if (!sensor_paused) {
      float dist = get_distance();
      distance_msg.data = dist;
      rcl_publish(&distance_pub, &distance_msg, NULL);
    } else {
      // Publicar última lectura válida
      distance_msg.data = last_valid_distance;
      rcl_publish(&distance_pub, &distance_msg, NULL);
    }
  }
}

void odom_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  
  left_distance = encoder_left_pulses * METERS_PER_PULSE;
  right_distance = encoder_right_pulses * METERS_PER_PULSE;
  total_distance = (left_distance + right_distance) / 2.0;
  
  odom_msg.data = total_distance;
  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// ═══════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  pinMode(LED_MISSION, OUTPUT);
  pinMode(LED_PAUSE, OUTPUT);
  pinMode(BTN_ACK, INPUT_PULLUP);
  
  digitalWrite(LED_MISSION, LOW);
  digitalWrite(LED_PAUSE, LOW);
  
  Serial.begin(115200);
  delay(2000);
  // TB6612
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  digitalWrite(STBY, HIGH);
  stop_motors();

  // Encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  encoder_left_last_state = digitalRead(ENCODER_LEFT_A);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoder_left_isr, CHANGE);
  
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  encoder_right_last_state = digitalRead(ENCODER_RIGHT_A);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoder_right_isr, CHANGE);

  //VL53L0X CON INICIALIZACIÓN ROBUSTA
  Wire.begin();
  sensor.setTimeout(500);
  
  int init_attempts = 0;
  while (!sensor.init() && init_attempts < 5) {
    Serial.println("⚠️ Reintentando inicialización VL53L0X...");
    delay(1000);
    init_attempts++;
  }
  
  if (init_attempts >= 5) {
    Serial.println("ERROR CRÍTICO: VL53L0X no responde");
    while (1) {
      digitalWrite(LED_MISSION, !digitalRead(LED_MISSION));
      delay(200);
    }
  }
  
  sensor.setMeasurementTimingBudget(33000);
  
  // Limpiar buffer inicial del sensor
  for (int i = 0; i < 5; i++) {
    sensor.readRangeSingleMillimeters();
    delay(50);
  }
  
  Serial.println("✓ VL53L0X inicializado correctamente");

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(ANGLE_CENTER);
  current_servo_angle = ANGLE_CENTER;
  last_servo_move = millis();
  delay(500);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✓ WiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // MQTT
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqtt_callback);

  // HTTP Server
  server.on("/color", handle_color);
  server.on("/target", handle_target);
  server.on("/mission_state", handle_mission_state);
  server.on("/status", handle_status);
  server.begin();
  Serial.println("✓ HTTP server activo");

  // micro-ROS
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "hospital_robot", "", &support);

  rclc_publisher_init_default(&distance_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "distance_front");
  rclc_publisher_init_default(&distance_left_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "distance_left");
  rclc_publisher_init_default(&distance_right_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "distance_right");
  rclc_publisher_init_default(&odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "odom");

  rclc_subscription_init_default(&cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_subscription_init_default(&scan_cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "scan_command");
  rclc_subscription_init_default(&servo_angle_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "servo_angle");

  std_msgs__msg__Float32__init(&distance_msg);
  std_msgs__msg__Float32__init(&distance_left_msg);
  std_msgs__msg__Float32__init(&distance_right_msg);
  std_msgs__msg__Float32__init(&odom_msg);
  std_msgs__msg__Bool__init(&scan_cmd_msg);
  std_msgs__msg__Int16__init(&servo_angle_msg);
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);

  rclc_timer_init_default(&distance_timer, &support, RCL_MS_TO_NS(50), distance_callback);
  rclc_timer_init_default(&safety_timer, &support, RCL_MS_TO_NS(100), safety_callback);
  rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(100), odom_callback);

  rclc_executor_init(&executor, &support.context, 6, &allocator);
  rclc_executor_add_timer(&executor, &distance_timer);
  rclc_executor_add_timer(&executor, &safety_timer);
  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &scan_cmd_sub, &scan_cmd_msg, &scan_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_angle_sub, &servo_angle_msg, &servo_angle_callback, ON_NEW_DATA);

  last_cmd_vel_time = millis();
  
  Serial.println("\nSISTEMA LISTO");
  Serial.println("   • Sensor VL53L0X: ESTABILIZADO");
  Serial.println("   • Pausa automática durante barridos");
  Serial.println("   • Recuperación de errores activa\n");
}

// ═══════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  // MQTT
  if (!mqttClient.connected()) reconnect_mqtt();
  mqttClient.loop();
  
  // Heartbeat
  if (millis() - lastPing > pingInterval) {
    mqttClient.publish(TOPIC_PING, "alive");
    lastPing = millis();
  }
  
  // Botón (GPIO5) - confirmar entrega
  static unsigned long last_btn_check = 0;
  if (millis() - last_btn_check > 100) {
    if (missionState == PAUSA && digitalRead(BTN_ACK) == LOW) {
      Serial.println("Botón presionado - volviendo a almacén");
      
      digitalWrite(LED_PAUSE, LOW);
      missionState = VOLVER;
      
      // Reactivar sensor al reanudar misión
      sensor_paused = false;
      consecutive_sensor_errors = 0;
      delay(300);
    }
    last_btn_check = millis();
  }
  
  // HTTP
  server.handleClient();
  
  // micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  delay(1);
}