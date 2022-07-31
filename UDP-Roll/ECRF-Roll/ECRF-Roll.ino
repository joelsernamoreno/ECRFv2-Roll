#include "WiFi.h"
#include "AsyncUDP.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"

const char * ssid = "testing";
const char * password = "123456789";

#define samplesize 2000

//Pushbutton Pins
int push1 = 34;
int push2 = 35;

int error_toleranz = 200;

int RXPin = 26;
int RXPin0 = 4;
int TXPin0 = 2;
int Gdo0 = 25;

const int minsample = 30;
unsigned long sample[samplesize];
unsigned long samplesmooth[samplesize];
unsigned long first_signal_rx[samplesize];
unsigned long second_signal_rx[samplesize];
int samplecount;
static unsigned long lastTime = 0;
String transmit = "";
long data_to_send[2000];
long data_button1[2000];
long data_button2[2000];
long data_button3[2000];
long transmit_push[2000];

String tmp_datarate;
String tmp_powerjammer;
int power_jammer;
int attemps;
String raw_rx = "0";
String jammer_tx = "0";
const bool formatOnFail = true;
String webString;
String bindata;
int samplepulse;
String tmp_samplepulse;
String tmp_transmissions;
String tmp_attemps;
int counter=0;
int pos = 0;
int transmissions;
int pushbutton1 = 0;
int pushbutton2 = 0;
int pushbutton3 = 0;
byte jammer[11] = {0xff,0xff,};

//BTN Sending Config
int btn_set_int;
String btn_set;
String btn1_frequency;
String btn1_mod;
String btn1_rawdata;
String btn1_deviation;
String btn1_transmission;
String btn2_frequency;
String btn2_mod;
String btn2_rawdata;
String btn2_deviation;
String btn2_transmission;
String btn3_frequency;
float tmp_btn1_deviation;
float tmp_btn2_deviation;
float tmp_btn1_frequency;
float tmp_btn2_frequency;
int tmp_btn1_mod;
int tmp_btn2_mod;
int tmp_btn1_transmission;
int tmp_btn2_transmission;

// Rolling check
int firstsignal = 0;
int smoothcountfirstsignal;
int smoothcountsecondsignal;
int smoothcount;

// Jammer
int jammer_pin;

#if defined(ESP8266)
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
#elif defined(ESP32)
    #define RECEIVE_ATTR IRAM_ATTR
#else
    #define RECEIVE_ATTR
#endif

AsyncUDP udp;

bool checkReceived(void){
  
  delay(1);
  if (samplecount >= minsample && micros()-lastTime >100000){
    detachInterrupt(RXPin0);
    detachInterrupt(RXPin);
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
    detachInterrupt(RXPin);
    checkReceived();
  }
  lastTime = time;
}

void enableReceive(){
  pinMode(RXPin0,INPUT);
  RXPin0 = digitalPinToInterrupt(RXPin0);
  ELECHOUSE_cc1101.SetRx();
  samplecount = 0;
  attachInterrupt(RXPin0, receiver, CHANGE);
  pinMode(RXPin,INPUT);
  RXPin = digitalPinToInterrupt(RXPin);
  ELECHOUSE_cc1101.SetRx();
  samplecount = 0;
  attachInterrupt(RXPin, receiver, CHANGE);
}

void setup()
{
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    pinMode(push1, INPUT);
    pinMode(push2, INPUT);
  
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi Failed");
        while(1) {
            delay(1000);
        }
    }
    if(udp.connect(IPAddress(192,168,4,1), 1234)) {
        Serial.println("UDP connected");
        udp.onPacket([](AsyncUDPPacket packet) {
        });
    }

    udp.print("1");

    ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 5, 0);
    ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 27, 1);
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setModul(0);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setSyncMode(0);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    ELECHOUSE_cc1101.setPktFormat(3);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.

    ELECHOUSE_cc1101.setModulation(2);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setRxBW(58);
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.setDeviation(0);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    ELECHOUSE_cc1101.setDRate(5);           // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
 
    enableReceive();
    raw_rx = "1";
    firstsignal = 0;
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

  // Second signal
  if(firstsignal == 1){
    smoothcountfirstsignal = smoothcount;

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
      }else{
        for (int b=0; b<calculate; b++){
        }
      }
    }
  }

  smoothcount=0;
  for (int i=1; i<samplecount; i++){
    float r = (float)sample[i]/timingdelay[0];
    int calculate = r;
    r = r-calculate;
    r*=10;
    if (r>=5){calculate+=1;}
    if (calculate>0){
      second_signal_rx[smoothcount] = calculate*timingdelay[0];
      smoothcount++;
    }
  }
  
  smoothcountsecondsignal = smoothcount;
  firstsignal = 3;
  Serial.println("Second signal captured");
  TXfirstsignal();
  return;
  }

  else if(firstsignal == 0){

  Serial.println("");
  Serial.println("First Signal: ");
  Serial.println("");

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
      }else{
        for (int b=0; b<calculate; b++){
        }
      }
    }
  }

  smoothcount=0;
  for (int i=1; i<samplecount; i++){
    float r = (float)sample[i]/timingdelay[0];
    int calculate = r;
    r = r-calculate;
    r*=10;
    if (r>=5){calculate+=1;}
    if (calculate>0){
      first_signal_rx[smoothcount] = calculate*timingdelay[0];
      smoothcount++;
    }
  }
  
  Serial.println("First signal captured");
  delay(2000);
  firstsignal = 1;
  
  ELECHOUSE_cc1101.setSidle();
  ELECHOUSE_cc1101.setModul(0);
  Serial.println("Module 1");

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setSyncMode(0);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  ELECHOUSE_cc1101.setPktFormat(3);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.

  ELECHOUSE_cc1101.setModulation(2);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setRxBW(58);
  ELECHOUSE_cc1101.setMHZ(433.92);
  ELECHOUSE_cc1101.setDeviation(0);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
  ELECHOUSE_cc1101.setDRate(5);           // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
 
  enableReceive();
  //return;
  }
}

void replayFirstsignal() {
  ELECHOUSE_cc1101.setSidle();
  pinMode(2,OUTPUT);
  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(433.92);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.SetTx();
  
  for (int i = 0; i<smoothcountfirstsignal; i+=2){
    digitalWrite(2,HIGH);
    delayMicroseconds(first_signal_rx[i]);
    digitalWrite(2,LOW);
    delayMicroseconds(first_signal_rx[i+1]);
  }
}

void TXfirstsignal(){
  //delay(2000);
  // Turn off RX
  raw_rx = "0";
  ELECHOUSE_cc1101.setSidle();

  udp.print("0");

  // Transmit First Signal
  delay(400);
  
  pinMode(2,OUTPUT);
  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(433.92);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.SetTx();
  
  for (int i = 0; i<smoothcountfirstsignal; i+=2){
    digitalWrite(2,HIGH);
    delayMicroseconds(first_signal_rx[i]);
    digitalWrite(2,LOW);
    delayMicroseconds(first_signal_rx[i+1]);
  }

  Serial.println("First signal transmitted");

  // Show Second Signal Captured
  
  Serial.println("");
  Serial.println("Finish");
  Serial.println("");
  Serial.println("Second signal: ");
  Serial.println("");
}

void TXsecondsignal(){

  ELECHOUSE_cc1101.setSidle();
  pinMode(2,OUTPUT);
  ELECHOUSE_cc1101.setModul(0);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(433.92);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.SetTx();

  for (int i = 0; i<smoothcountsecondsignal; i+=2){
    digitalWrite(2,HIGH);
    delayMicroseconds(second_signal_rx[i]);
    digitalWrite(2,LOW);
    delayMicroseconds(second_signal_rx[i+1]);
  }
  ELECHOUSE_cc1101.setSidle();
}

void loop()
{
  pushbutton1 = digitalRead(push1);
  pushbutton2 = digitalRead(push2);

  if(raw_rx == "1") {
    if(checkReceived()){
      ELECHOUSE_cc1101.setModul(0);
      ELECHOUSE_cc1101.setSidle();
      signalanalyse();
      enableReceive();
    }
  }
  if(pushbutton1 == LOW) {
    TXsecondsignal();
  }

  if(pushbutton2 == LOW) {
    replayFirstsignal();
  }
}
