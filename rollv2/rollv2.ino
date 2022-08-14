#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <esp_now.h>
#include <WiFi.h>

#if defined(ESP8266)
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
#elif defined(ESP32)
    #define RECEIVE_ATTR IRAM_ATTR
#else
    #define RECEIVE_ATTR
#endif

#define samplesize 2000

// Radio config
float frequency = 433.92;
int mod = 2;
float rxbw = 58;
float datarate = 5;
float deviation = 0;
int powerjammer = 5;
float freqjammer = 433.80;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xB3, 0x7F, 0x68};

int RXPin = 26;
int RXPin0 = 4;
int TXPin0 = 2;
int Gdo0 = 25;

//Pushbutton Pins
int push1 = 34;
int push2 = 35;
int pushbutton1 = 0;
int pushbutton2 = 0;

int samplecount;
unsigned long sample[samplesize];
int error_toleranz = 200;
unsigned long samplesmooth[samplesize];
unsigned long secondsamplesmooth[samplesize];
long transmit_push[2000];
static unsigned long lastTime = 0;
const int minsample = 30;
int smoothcount=0;
int secondsmoothcount=0;

int start = 0;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int config_jammer;
  int power_jammer;
  float freq_jammer;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool checkReceived(void){
  delay(1000);
  if (samplecount >= minsample && micros()-lastTime >100000){
    detachInterrupt(RXPin0);
    //detachInterrupt(RXPin);
    return 1;
  }else{
    return 0;
  }
}

void RECEIVE_ATTR receiver() {
  const long time = micros();
  const unsigned int duration = time - lastTime;

  if (duration > 100000){
    samplecount = 0;
  }

  if (duration >= 100){
    sample[samplecount++] = duration;
  }

  if (samplecount>=samplesize){
    detachInterrupt(RXPin0);
    //detachInterrupt(RXPin);
    checkReceived();
  }
  lastTime = time;
}

void signalanalyse(){
  #define signalstorage 10

  int signalanz=0;
  int timingdelay[signalstorage];
  float pulse[signalstorage];
  long signaltimings[signalstorage*2];
  int signaltimingscount[signalstorage];
  long signaltimingssum[signalstorage];
  long signalsum=0;

  for (int i = 0; i<signalstorage; i++){
    signaltimings[i*2] = 100000;
    signaltimings[i*2+1] = 0;
    signaltimingscount[i] = 0;
    signaltimingssum[i] = 0;
  }
  for (int i = 1; i<samplecount; i++){
    signalsum+=sample[i];
  }

  for (int p = 0; p<signalstorage; p++){

  for (int i = 1; i<samplecount; i++){
    if (p==0){
      if (sample[i]<signaltimings[p*2]){
        signaltimings[p*2]=sample[i];
      }
    }else{
      if (sample[i]<signaltimings[p*2] && sample[i]>signaltimings[p*2-1]){
        signaltimings[p*2]=sample[i];
      }
    }
  }

  for (int i = 1; i<samplecount; i++){
    if (sample[i]<signaltimings[p*2]+error_toleranz && sample[i]>signaltimings[p*2+1]){
      signaltimings[p*2+1]=sample[i];
    }
  }

  for (int i = 1; i<samplecount; i++){
    if (sample[i]>=signaltimings[p*2] && sample[i]<=signaltimings[p*2+1]){
      signaltimingscount[p]++;
      signaltimingssum[p]+=sample[i];
    }
  }
  }

  signalanz=signalstorage;
  for (int i = 0; i<signalstorage; i++){
    if (signaltimingscount[i] == 0){
      signalanz=i;
      i=signalstorage;
    }
  }

  for (int s=1; s<signalanz; s++){
  for (int i=0; i<signalanz-s; i++){
    if (signaltimingscount[i] < signaltimingscount[i+1]){
      int temp1 = signaltimings[i*2];
      int temp2 = signaltimings[i*2+1];
      int temp3 = signaltimingssum[i];
      int temp4 = signaltimingscount[i];
      signaltimings[i*2] = signaltimings[(i+1)*2];
      signaltimings[i*2+1] = signaltimings[(i+1)*2+1];
      signaltimingssum[i] = signaltimingssum[i+1];
      signaltimingscount[i] = signaltimingscount[i+1];
      signaltimings[(i+1)*2] = temp1;
      signaltimings[(i+1)*2+1] = temp2;
      signaltimingssum[i+1] = temp3;
      signaltimingscount[i+1] = temp4;
    }
  }
  }

  for (int i=0; i<signalanz; i++){
    timingdelay[i] = signaltimingssum[i]/signaltimingscount[i];
  }

  bool lastbin=0;
  for (int i=1; i<samplecount; i++){
    float r = (float)sample[i]/timingdelay[0];
    int calculate = r;
    r = r-calculate;
    r*=10;
    if (r>=5){calculate+=1;}
    if (calculate>0){
      if (lastbin==0){
        lastbin=1;
      }else{
      lastbin=0;
    }
      if (lastbin==0 && calculate>8){
        Serial.print(" [Pause: ");
        Serial.print(sample[i]);
        Serial.println(" samples]");
      }else{
        for (int b=0; b<calculate; b++){
          Serial.print(lastbin);
        }
      }
    }
  }
  Serial.println();
  Serial.print("Samples/Symbol: ");
  Serial.println(timingdelay[0]);
  Serial.println();

  if(start == 1) {
    secondsmoothcount=0;
    for (int i=1; i<samplecount; i++){
        float r = (float)sample[i]/timingdelay[0];
        int calculate = r;
        r = r-calculate;
        r*=10;
        if (r>=5){calculate+=1;}
          if (calculate>0){
            secondsamplesmooth[secondsmoothcount] = calculate*timingdelay[0];
            secondsmoothcount++;
        }
    }
    Serial.println("Rawdata corrected:");
    Serial.print("Count=");
    Serial.println(secondsmoothcount+1);
  
    for (int i=0; i<secondsmoothcount; i++){
      Serial.print(secondsamplesmooth[i]);
      Serial.print(",");
      transmit_push[i] = secondsamplesmooth[i];
    }
    start = 2;
  }
 
  if(start == 0) {
    smoothcount=0;
    for (int i=1; i<samplecount; i++){
      float r = (float)sample[i]/timingdelay[0];
      int calculate = r;
      r = r-calculate;
      r*=10;
      if (r>=5){calculate+=1;}
      if (calculate>0){
        samplesmooth[smoothcount] = calculate*timingdelay[0];
        smoothcount++;
      }
    }
    Serial.println("Rawdata corrected:");
    Serial.print("Count=");
    Serial.println(smoothcount+1);
  
    for (int i=0; i<smoothcount; i++){
      Serial.print(samplesmooth[i]);
      Serial.print(",");
      transmit_push[i] = samplesmooth[i];
    }
    start = 1;
  }
  
  Serial.println();
  Serial.println();
  return;
}

void enableReceive(){
  pinMode(RXPin0,INPUT);
  RXPin0 = digitalPinToInterrupt(RXPin0);
  ELECHOUSE_cc1101.SetRx();
  samplecount = 0;
  attachInterrupt(RXPin0, receiver, CHANGE);
  samplecount = 0;
}

void printReceived(){
  ELECHOUSE_cc1101.setSidle();
  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setPktFormat(3);
  ELECHOUSE_cc1101.setModulation(mod);
  ELECHOUSE_cc1101.setRxBW(rxbw);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setDeviation(deviation);
  ELECHOUSE_cc1101.setDRate(datarate);
  //ELECHOUSE_cc1101.setDcFilterOff(1);
  
  Serial.print("Count=");
  Serial.println(samplecount);
  
  for (int i = 1; i<samplecount; i++){
    Serial.print(sample[i]);
    Serial.print(",");
  }
  Serial.println();
  Serial.println();
}

void setup() {
  delay(2000);
  // put your setup code here, to run once:
  Serial.begin(38400);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print(",");
  ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 5, 0); // Evil Crow RF V2
  ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 27, 1); // Evil Crow RF V2
  //ELECHOUSE_cc1101.addSpiPin(18, 19, 23, 5, 0); // Evil Crow RF V1
  //ELECHOUSE_cc1101.addSpiPin(18, 19, 23, 27, 1); // Evil Crow RF V1
  
  pinMode(push1, INPUT);
  pinMode(push2, INPUT);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Set values to send
  myData.config_jammer = 1;
  myData.power_jammer = powerjammer;
  myData.freq_jammer = freqjammer;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setPktFormat(3);
  ELECHOUSE_cc1101.setModulation(mod);
  ELECHOUSE_cc1101.setRxBW(rxbw);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setDeviation(deviation);
  ELECHOUSE_cc1101.setDRate(datarate);
  //ELECHOUSE_cc1101.setDcFilterOff(1);
  enableReceive();
}

void replayfirstsignal(){

  ELECHOUSE_cc1101.setSidle();
  
  pinMode(2,OUTPUT);
  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(mod);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setDeviation(deviation);
  ELECHOUSE_cc1101.SetTx();
  delay(1000);

  for (int i = 0; i<2000; i+=2){
    digitalWrite(2,HIGH);
    delayMicroseconds(samplesmooth[i]);
    digitalWrite(2,LOW);
    delayMicroseconds(samplesmooth[i+1]);
  }
  ELECHOUSE_cc1101.setSidle();
}

void TXfirstsignal(){

  ELECHOUSE_cc1101.setSidle();
   // Turn off jammer
  myData.config_jammer = 2;
  myData.power_jammer = powerjammer;
  myData.freq_jammer = freqjammer;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  pinMode(2,OUTPUT);
  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(mod);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setDeviation(deviation);
  ELECHOUSE_cc1101.SetTx();
  delay(1000);

  for (int i = 0; i<2000; i+=2){
    digitalWrite(2,HIGH);
    delayMicroseconds(samplesmooth[i]);
    digitalWrite(2,LOW);
    delayMicroseconds(samplesmooth[i+1]);
  }
  ELECHOUSE_cc1101.setSidle();
}

void TXsecondsignal(){

  ELECHOUSE_cc1101.setSidle();
  pinMode(2,OUTPUT);
  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(mod);
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setDeviation(deviation);
  ELECHOUSE_cc1101.SetTx();
  delay(500);

  for (int i = 0; i<2000; i+=2){
    digitalWrite(2,HIGH);
    delayMicroseconds(secondsamplesmooth[i]);
    digitalWrite(2,LOW);
    delayMicroseconds(secondsamplesmooth[i+1]);
  }
  ELECHOUSE_cc1101.setSidle();
}

void shift_fsk(){
  for (int i = 1; i<samplecount; i++){
    sample[i] = sample[i+1];
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  pushbutton1 = digitalRead(push1);
  pushbutton2 = digitalRead(push2);
  
  if(start == 0) {
    if(checkReceived()){
      //shift_fsk();
      printReceived();
      signalanalyse();
      enableReceive();
    }
  }
  if(start == 1) {
    if(checkReceived()){
      //shift_fsk();
      printReceived();
      signalanalyse();
      enableReceive();
      delay(500);
      TXfirstsignal();
    }
  }

   if (pushbutton1 == LOW) {
    Serial.println("TX second signal");
    TXsecondsignal();
   }

   if (pushbutton2 == LOW) {
    Serial.println("TX first signal");
    replayfirstsignal();
   }
}
