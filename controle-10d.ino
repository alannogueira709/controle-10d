#include <WiFi.h>                               //Adição da biblioteca Wifi.h para permitir o ESP 32 se conectar a uma rede Wifi
#include <ESPmDNS.h>                            //Adição da biblioteca do protocolo MQTT
#include <WiFiUdp.h>                            //Adição da biblioteca WiFiUdp.h usada para enviar e receber dados de um servidor
#include <ArduinoOTA.h>                         //Adição da biblioteca ArduinoOTA.h que faz a atualizacao "wireless" do microcontrolador
#include <PubSubClient.h>                       //Adição da biblioteca PubSubClient.h que possibilita do protocolo MQTT  
#include "mpu9250.h"

bfs::Mpu9250 imu;

const char* ssid = "10D"; 
const char* pass = "12345678"

bool MQTT_connected = false;

WifiClient espClient;
PubSubClient client(espClient);
IPAddress mqtt_server(192.168.1.100);

const int JOYSTICK_X = 14;     // X-axis pin
const int JOYSTICK_Y = 27;     // Y-axis pin
const int JOYSTICK_BUTTON = 18;

volatile int controlMode = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Position tracking variables
float posX = 0;
float posY = 0;
float posZ = 0;

void IRAM_ATTR handleModeChange() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastDebounceTime > debounceDelay) {
    controlMode = !controlMode;
    lastDebounceTime = currentTime;
    
    // Reset position when changing modes
    posX = 0;
    posY = 0;
    posZ = 0;
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  
  Wire.begin();
  Wire.setClock(400000);
  
  // IMU Initialization
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  
  // Configure Sample Rate Divider
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while(1) {}
  }
  
  // Set up interrupt for mode changing
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON), handleModeChange, FALLING);
}

void loop() {
  int xValue = analogRead(JOYSTICK_X);
  int yValue = analogRead(JOYSTICK_Y);
  
  if (controlMode == 0) {
    Serial.println("Joystick Control Mode");
    handleJoystickPosition(xValue, yValue);
  } else {
    Serial.println("Motion Control Mode");
    handleMotionPosition();
  }
  
  delay(100);
}

void handleJoystickPosition(int xValue, int yValue) {
    // Map analog values to position
    posX = map(xValue, 0, 4095, -100, 100);
    posY = map(yValue, 0, 4095, -100, 100);
    
    Serial.print("Joystick Position - X: ");
    Serial.print(posX);
    Serial.print(" | Y: ");
    Serial.println(posY);
    
    // Determine direction based on position
    if (posX > 50 && posY < 50 && posY > -50) {
        Serial.println("Cima");
    } else if (posX < -50 && posY < 50 && posY > -50) {
        Serial.println("Baixo");
    } else if (posX < 50 && posX > -50 && posY > 50) {
        Serial.println("Direita");
    } else if (posX < 50 && posX > -50 && posY < -50) {
        Serial.println("Esquerda");
    } else if (posX > 50 && posY > 50) {
        Serial.println("Nordeste");
    } else if (posX < -50 && posY > 50) {
        Serial.println("Sudeste");
    } else if (posX < -50 && posY < -50) {
        Serial.println("Sudoeste");
    } else if (posX > 50 && posY < -50) {
        Serial.println("Noroeste");
    } else {
        Serial.println("Idle");
    }
}

void handleMotionPosition() {
    if (imu.Read()) {
        // Accelerometer readings
        float accelX = imu.accel_x_mps2();
        float accelY = imu.accel_y_mps2();
        float accelZ = imu.accel_z_mps2();
        
        // Gyroscope readings
        float gyroX = imu.gyro_x_radps();
        float gyroY = imu.gyro_y_radps();
        float gyroZ = imu.gyro_z_radps();
        
        // Calculate tilt angles using accelerometer
        float tiltX = atan2(accelY, accelZ) * 180.0 / PI;
        float tiltY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
        
        // Determine primary direction based on acceleration
        String primaryDirection = "Stationary";
        float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        
        // Direction detection thresholds
        const float MOVEMENT_THRESHOLD = 1.0; // m/s²
        const float TILT_THRESHOLD = 30.0; // degrees
        
        // Horizontal movement detection
        if (abs(accelX) > MOVEMENT_THRESHOLD) {
            primaryDirection = (accelX > 0) ? "Right" : "Left";
        } else if (abs(accelY) > MOVEMENT_THRESHOLD) {
            primaryDirection = (accelY > 0) ? "Forward" : "Backward";
        }
        
        // Vertical movement detection
        String verticalDirection = "Horizontal";
        if (abs(tiltX) > TILT_THRESHOLD) {
            verticalDirection = (tiltX > 0) ? "Tilting Up" : "Tilting Down";
        }
        
        // Rotation detection using gyroscope
        String rotationStatus = "No Rotation";
        float rotationMagnitude = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
        const float ROTATION_THRESHOLD = 0.1; // rad/s
        
        if (rotationMagnitude > ROTATION_THRESHOLD) {
            if (abs(gyroZ) > abs(gyroX) && abs(gyroZ) > abs(gyroY)) {
                rotationStatus = (gyroZ > 0) ? "Rotating Clockwise" : "Rotating Counterclockwise";
            } else if (abs(gyroX) > abs(gyroY) && abs(gyroX) > abs(gyroZ)) {
                rotationStatus = (gyroX > 0) ? "Rolling Right" : "Rolling Left";
            } else {
                rotationStatus = (gyroY > 0) ? "Pitching Up" : "Pitching Down";
            }
        }
        
        // Comprehensive motion analysis output
        Serial.println("Motion Analysis:");
        Serial.print("Primary Direction: ");
        Serial.println(primaryDirection);
        
        Serial.print("Vertical Orientation: ");
        Serial.println(verticalDirection);
        
        Serial.print("Rotation Status: ");
        Serial.println(rotationStatus);
        
        // Detailed sensor data
        Serial.print("Acceleration (m/s²): X=");
        Serial.print(accelX);
        Serial.print(" Y=");
        Serial.print(accelY);
        Serial.print(" Z=");
        Serial.print(accelZ);
        Serial.print(" (Total: ");
        Serial.print(totalAccel);
        Serial.println(")");
        
        Serial.print("Tilt Angles: X=");
        Serial.print(tiltX);
        Serial.print("° Y=");
        Serial.print(tiltY);
        Serial.println("°");
        
        Serial.print("Gyroscope (rad/s): X=");
        Serial.print(gyroX);
        Serial.print(" Y=");
        Serial.print(gyroY);
        Serial.print(" Z=");
        Serial.print(gyroZ);
        Serial.print(" (Rotation Magnitude: ");
        Serial.print(rotationMagnitude);
        Serial.println(")");
        
        Serial.println("-------------------");
    }
}