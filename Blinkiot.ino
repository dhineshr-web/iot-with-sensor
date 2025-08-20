#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// -------- Wi-Fi ----------
const char* WIFI_SSID     = "DHINESH";
const char* WIFI_PASSWORD = "12345678";

// -------- HiveMQ Cloud (TLS) ----------
const char* MQTT_HOST     = "47c32f0a0bef4745ba3e3fb896bc5056.s1.eu.hivemq.cloud";
const int   MQTT_PORT     = 8883; // TLS
const char* MQTT_USER     = "hivemq.webclient.1755667754607";
const char* MQTT_PASS     = "317HfxDb>M8:AF&Q<jls";

// -------- Topics (match HTML exactly) ----------
const char* T_LDR         = "esp32/ldr";
const char* T_IR          = "esp32/ir";
const char* T_DIST        = "esp32/distance";
const char* T_SERVO_CMD   = "esp32/servo";       // receives "open"/"close"/"on"/"off" OR an angle "0..180"
const char* T_SERVO_ACK   = "esp32/servo/ack";   // optional ack back to UI

// -------- Pins ----------
#define LDR_PIN   34     // ADC1 only (good)
#define IR_PIN    16
#define TRIG_PIN  32
#define ECHO_PIN  33
#define SERVO_PIN 27

// -------- Globals ----------
WiFiClientSecure tls;
PubSubClient mqtt(tls);
Servo servoMotor;
int currentAngle = 0;

// ---------- Helpers ----------
void wifiConnect() {
  Serial.print("WiFi: connecting to "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void mqttConnect() {
  while (!mqtt.connected()) {
    Serial.print("MQTT: connecting...");
    String cid = "ESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
      mqtt.subscribe(T_SERVO_CMD);   // listen for servo commands
      mqtt.publish("esp32/status", "online");
    } else {
      Serial.print("failed, rc="); Serial.print(mqtt.state()); Serial.println(" retry in 3s");
      delay(3000);
    }
  }
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // timeout ~ 25ms (≈ 4m range)
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1;  // no echo
  return (long)(duration * 0.034 / 2.0);
}

bool isNumber(const String& s) {
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  servoMotor.write(angle);
  currentAngle = angle;
  char ack[32]; snprintf(ack, sizeof(ack), "angle:%d", angle);
  mqtt.publish(T_SERVO_ACK, ack, true);
}

// ---------- MQTT callback ----------
void onMessage(char* topic, byte* payload, unsigned int len) {
  String t = String(topic);
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();

  Serial.printf("MQTT msg [%s]: %s\n", topic, msg.c_str());

  if (t == T_SERVO_CMD) {
    String low = msg; low.toLowerCase();
    if (low == "open" || low == "on") {
      setServoAngle(90);
    } else if (low == "close" || low == "off") {
      setServoAngle(0);
    } else if (isNumber(low)) {
      setServoAngle(low.toInt()); // raw angle
    } else if (low.startsWith("angle:")) {
      int a = low.substring(6).toInt();
      setServoAngle(a);
    } else {
      mqtt.publish(T_SERVO_ACK, "err:bad_cmd");
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(LDR_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoMotor.setPeriodHertz(50); // standard 50Hz
  servoMotor.attach(SERVO_PIN, 500, 2400); // uS for 0..180 (adjust if needed)
  setServoAngle(0);

  wifiConnect();

  tls.setInsecure(); // using TLS without cert (OK for testing with HiveMQ Cloud)
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);

  Serial.println("ESP32 ready.");
}

// ---------- Loop ----------
unsigned long lastPub = 0;
void loop() {
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  unsigned long now = millis();
  if (now - lastPub >= 1000) { // publish every 1s
    lastPub = now;

    int  ldr  = analogRead(LDR_PIN);   // 0..4095
    int  ir   = digitalRead(IR_PIN);   // 0/1 (module-specific, may be active LOW)
    long dist = readDistanceCM();

    char buf[16];
    itoa(ldr, buf, 10);   mqtt.publish(T_LDR, buf, true);
    itoa(ir,  buf, 10);   mqtt.publish(T_IR,  buf, true);

    if (dist >= 0) {
      itoa((int)dist, buf, 10);
      mqtt.publish(T_DIST, buf, true);
    } else {
      mqtt.publish(T_DIST, "NA", true);
    }

    // debug
    Serial.printf("LDR:%d  IR:%d  DIST:%ld cm  angle:%d\n", ldr, ir, dist, currentAngle);
  }
}
