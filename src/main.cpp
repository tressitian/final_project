// #include <Arduino.h>
// #include <WiFi.h>
// #include <HTTPClient.h>
// #include <ArduinoJson.h>
// #include <Adafruit_NeoPixel.h>
// #include <SwitecX25.h>

// // ✅ WiFi Credentials
// #define WIFI_SSID "Tressi"
// #define WIFI_PASSWORD "00000000"

// // ✅ Flask Server Endpoint - Need to create this endpoint on your server
// #define SERVER_URL "http://10.19.70.23:5001/get_emotion"

// // ✅ Pin Definitions
// #define BUZZER_PIN 2    // GPIO 2 for Buzzer
// #define LED_PIN 4       // GPIO 4 for WS2812B LED
// #define MOTOR_PIN1 23
// #define MOTOR_PIN2 22
// #define MOTOR_PIN3 21
// #define MOTOR_PIN4 19

// // ✅ Stepper Motor Setup (X27 168 Motor)
// SwitecX25 motor(315, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);

// // ✅ LED Strip Setup
// #define NUM_LEDS 1
// Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// WiFiClient client;
// HTTPClient http;

// // ✅ Function to Map Emotions to LED Colors & Motor Positions
// struct Emotion {
//     const char* name;
//     uint32_t color;
//     int motorPos;
// };

// // 🎭 Emotion Mapping
// Emotion emotions[] = {
//     {"happy", strip.Color(255, 255, 0), 0},      
//     {"sad", strip.Color(0, 0, 255), 60},         
//     {"angry", strip.Color(255, 0, 0), 120},      
//     {"calm", strip.Color(0, 255, 255), 180},     
//     {"surprised", strip.Color(255, 165, 0), 240},
//     {"neutral", strip.Color(255, 255, 255), 300}  
// };

// void setup() {
//     Serial.begin(115200);
//     pinMode(BUZZER_PIN, OUTPUT);
//     strip.begin();
//     strip.show();

//     // ✅ Initialize Stepper Motor (Zero Position)
//     motor.zero();
//     while (motor.currentStep != 0) {
//         motor.update();  // Ensure it fully resets
//     }

//     // 🔄 Connect to WiFi
//     WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//     Serial.print("Connecting to WiFi...");
    
//     int attempts = 0;
//     while (WiFi.status() != WL_CONNECTED && attempts < 20) { 
//         Serial.print(".");
//         delay(1000);
//         attempts++;
//     }

//     if (WiFi.status() == WL_CONNECTED) {
//         Serial.println("\n🚀 WiFi Connected!");
//         Serial.print("IP Address: ");
//         Serial.println(WiFi.localIP());
//     } else {
//         Serial.println("\n❌ WiFi Connection Failed!");
//     }
// }

// void loop() {
//     Serial.println("🔄 Checking for Emotion Data...");
    
//     // 🔽 Get Data from Flask Server
//     http.begin(client, SERVER_URL);
//     int httpResponseCode = http.GET();
    
//     if (httpResponseCode > 0) {
//         String response = http.getString();
//         Serial.println("📥 Received: " + response);

//         // 📜 Parse JSON Response
//         StaticJsonDocument<200> jsonDoc;
//         DeserializationError error = deserializeJson(jsonDoc, response);
//         if (error) {
//             Serial.println("❌ JSON Parsing Failed!");
//         } else {
//             String emotion = jsonDoc["emotion"];

//             // 🔊 Buzzer Alert
//             tone(BUZZER_PIN, 1000, 200);
//             delay(200);
//             noTone(BUZZER_PIN);

//             // 🌈 Set LED Color & Move Stepper Motor
//             for (Emotion e : emotions) {
//                 if (emotion == e.name) {
//                     strip.setPixelColor(0, e.color);
//                     strip.show();

//                     // ✅ Correct Stepper Motor Movement
//                     motor.setPosition(e.motorPos);  
//                     while (motor.currentStep != motor.targetStep) {
//                         motor.update();  // Ensures smooth motion
//                     }

//                     Serial.print("🎭 Emotion Detected: ");
//                     Serial.println(emotion);
//                     break;
//                 }
//             }
//         }
//     } else {
//         Serial.println("❌ Error Fetching Data from Server!");
//     }

//     http.end();
//     delay(5000); // Wait before checking again
// }

// #include <Arduino.h>
// #include <esp_now.h>
// #include <WiFi.h>

// #define BUZZER_PIN 5  // 蜂鸣器
// #define MOTOR_PIN 6   // 电机
// #define LED_R 9       // 红色 LED
// #define LED_G 10      // 绿色 LED
// #define LED_B 11      // 蓝色 LED

// typedef struct {
//     uint8_t r;
//     uint8_t g;
//     uint8_t b;
// } SensorData;

// SensorData receivedData;

// void onReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
//     memcpy(&receivedData, incomingData, sizeof(receivedData));

//     Serial.printf("🎨 Received Color: R=%d, G=%d, B=%d\n", receivedData.r, receivedData.g, receivedData.b);

//     // **控制 LED 显示颜色**
//     analogWrite(LED_R, receivedData.r);
//     analogWrite(LED_G, receivedData.g);
//     analogWrite(LED_B, receivedData.b);

//     // **触发蜂鸣器**
//     digitalWrite(BUZZER_PIN, HIGH);
//     delay(200);
//     digitalWrite(BUZZER_PIN, LOW);

//     // **转动电机**
//     digitalWrite(MOTOR_PIN, HIGH);
//     delay(500);
//     digitalWrite(MOTOR_PIN, LOW);
// }

// void setup() {
//     Serial.begin(115200);
//     pinMode(BUZZER_PIN, OUTPUT);
//     pinMode(MOTOR_PIN, OUTPUT);
//     pinMode(LED_R, OUTPUT);
//     pinMode(LED_G, OUTPUT);
//     pinMode(LED_B, OUTPUT);

//     WiFi.mode(WIFI_STA);
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("❌ ESP-NOW Init Failed");
//         return;
//     }
//     esp_now_register_recv_cb(onReceive);
    
// }

// void loop() {
//     delay(100);
// }

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>
#include <SwitecX25.h>  // X27 168 步进电机驱动库

#define BUZZER_PIN 2    
#define LED_PIN 4       
#define MOTOR_PIN1 23  // A+
#define MOTOR_PIN2 22  // A-
#define MOTOR_PIN3 21  // B+
#define MOTOR_PIN4 19  // B-

// **UUID**
static BLEUUID serviceUUID("135a9a46-55dc-41a0-8671-b4dae2a33cd8");
static BLEUUID charUUID("10b9e029-fe24-41fa-939e-1f6c189f5593");

bool deviceConnected = false;
BLERemoteCharacteristic* pRemoteCharacteristic;
BLEAdvertisedDevice* myDevice;

#define NUM_LEDS 20  
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// **X27 168 电机对象**
SwitecX25 motor(315 * 3, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);  // 315° 电机

// **上一次接收到的 RGB 数据**
String lastReceivedData = "";

// **步进电机旋转角度（只选择 5°, 10°, 15°, 20°）**
const int motorAngles[] = {45, 90};

// **BLE 设备发现回调**（**确保它在 setup() 之前定义**）
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("🔍 Found Device: ");
        Serial.println(advertisedDevice.getName().c_str());

        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            Serial.println("✅ Found Sensor Device!");
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            deviceConnected = true;
        }
    }
};

// **BLE 连接回调**
class MyClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) { 
        Serial.println("✅ Connected to Sensor!"); 
    }

    void onDisconnect(BLEClient* pclient) { 
        Serial.println("❌ Disconnected! Retrying...");
        deviceConnected = false;
        BLEDevice::getScan()->start(5, false);
    }
};

// **初始化**
void setup() {
    Serial.begin(115200);

    // **初始化 LED**
    strip.begin();
    strip.show();  
    strip.setBrightness(50);

    // **初始化蜂鸣器**
    pinMode(BUZZER_PIN, OUTPUT);

    // **初始化 X27 168 电机**
    motor.zero();  // 归零（重要！）
    delay(1000);

    // **初始化 BLE**
    Serial.println("🚀 Starting BLE Client...");
    BLEDevice::init("");
    BLEDevice::setMTU(517);

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());  // ✅ 确保这个类已经定义
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
}

// **设置 LED 颜色**
void setLEDColor(int r, int g, int b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
    Serial.println("💡 LED Strip Updated!");
}

// **播放卡农**
void playCanon() {
    Serial.println("🎵 Playing Canon...");
    int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
    int durations[] = {400, 400, 400, 400, 400, 400, 400, 400};

    for (int i = 0; i < 8; i++) {
        tone(BUZZER_PIN, melody[i], durations[i]);
        delay(durations[i] * 1.2);
    }
    noTone(BUZZER_PIN);
}

// **旋转 X27 168**
void rotateMotor() {
    int randomIndex = random(0, 2);  // 随机选择 45° 或 90°，确保索引不会越界
    int angle = motorAngles[randomIndex];

    Serial.print("🔄 Rotating Motor: ");
    Serial.print(angle);
    Serial.println("°");

    // **检查是否已经在该位置，防止重复指令无效**
    int targetPosition = (angle * 315) / 360;
    if (motor.targetStep == targetPosition) {
        Serial.println("⚠️ Motor already at this position, skipping...");
        return;
    }

    // **设置目标位置**
    motor.setPosition(targetPosition);

    // **确保 motor.update() 逐步执行，直到到达目标**
    while (motor.currentStep != motor.targetStep) {
        motor.update();
        delay(1);  // **小延时，防止过快导致电机卡住**
    }

    Serial.println("✅ Motor move completed!");
}

// **BLE 数据回调**
static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify) {

    Serial.print("🎨 Received Data: ");
    String receivedData = "";
    for (size_t i = 0; i < length; i++) receivedData += (char)pData[i];
    Serial.println(receivedData);

    // **如果收到的数据和上次一样，则不执行**
    if (receivedData == lastReceivedData) {
        Serial.println("⚠️ Same data received, skipping...");
        return;
    }

    int r, g, b;
    if (sscanf(receivedData.c_str(), "%d,%d,%d", &r, &g, &b) == 3) {
        Serial.println("✅ Valid RGB data received!");
        setLEDColor(r, g, b);
        playCanon();
        rotateMotor();  // **确保电机只在接收新数据时转动**

        // **存储这次的 RGB 数据**
        lastReceivedData = receivedData;
    } else {
        Serial.println("⚠️ Invalid data received, ignoring...");
    }
}

// **主循环**
void loop() {
    if (deviceConnected && myDevice) {
        BLEClient* pClient = BLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallbacks());

        if (pClient->connect(myDevice)) {
            Serial.println("✅ Connected to Sensor!");
            BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
            if (pRemoteService) {
                pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
                if (pRemoteCharacteristic) {
                    if (pRemoteCharacteristic->canNotify()) {  
                        pRemoteCharacteristic->registerForNotify(notifyCallback);
                        Serial.println("✅ Registered for Notifications!");
                    }
                }
            }
        }
    }

    // **保持 BLE 连接**
    if (deviceConnected) {
        Serial.println("✅ BLE Connected, waiting for data...");
        delay(1000);
    } else {
        Serial.println("🔄 Re-scanning BLE devices...");
        BLEDevice::getScan()->start(5, false);
    }
}