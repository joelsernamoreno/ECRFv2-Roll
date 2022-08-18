// Evil Crow RF Jam v1.0

#include <esp_now.h>
#include <WiFi.h>
#include "ELECHOUSE_CC1101_SRC_DRV.h"

int config_jammer;
int power_jammer;
float freq_jammer;

int jammer_config = 0;
int start_jammer = 0;
int jammer;
float freq;
int power;
byte jammertx[11] = {0xff,0xff,};

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int config_jammer;
  int power_jammer;
  float freq_jammer;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println(myData.config_jammer);
  jammer = myData.config_jammer;
  freq = myData.freq_jammer;
  power = myData.power_jammer;
  loop();
}

void ConfigJammer() {
  jammer = 0;
  pinMode(25,OUTPUT);
  ELECHOUSE_cc1101.setModul(1);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(freq);
  ELECHOUSE_cc1101.setPA(power);
  ELECHOUSE_cc1101.SetTx();
  start_jammer = 1;
}

void StopJammer() {
  start_jammer = 0;
  jammer = 3;
  ELECHOUSE_cc1101.setSidle();
}

void setup() {
  // put your setup code here, to run once:
  // Initialize Serial Monitor
  Serial.begin(38400);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // CC1101 Config
  ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 27, 1); // Evil Crow RF V2

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  esp_now_register_recv_cb(OnDataRecv);
  
  if(jammer == 1) {
    ConfigJammer();
  }
  if(start_jammer == 1) {

    // TX Jammer
    
    for (int i = 0; i<12; i+=2){
      digitalWrite(25,HIGH);
      delayMicroseconds(jammertx[i]);
      digitalWrite(25,LOW);
      delayMicroseconds(jammertx[i+1]);
    }
  }

  if(jammer == 2) {
    StopJammer();
  }
}
