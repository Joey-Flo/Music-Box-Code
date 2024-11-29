#include "BluetoothA2DPSink.h"
#include "Arduino.h"
#include "driver/ledc.h"
#include  "arduinoFFT.h"
const int PWM_FREQ = 5000;   // Frequency in Hz
const int PWM_CHANNEL = 0;   // PWM channel (0-15)
const int PWM_RESOLUTION = 8; // Resolution of PWM (8 bits)
const int PWM_PIN = 2; //PWM pin
int PreviousLowState = 0;
int PreviousHighState = 0;
int PreviousLowMenu = 0;
int PreviousHighMenu = 0;
const int colorCBR = 1;
const int colorCBG = 2;  // Color Bass R/G/B
const int colorCBB = 3;
const int colorCFR = 4;
const int colorCFG = 5;  // Color Freq R/G/B
const int colorCFB = 6;
int colorModeHIGH = 0;
int colorModeLOW = 0;
int colorSetHIGH = 0; // Color status variables
int colorSetLOW = 0;
int colorSetting = 2;
float Intensity1HIGH = 0;
float Intensity2HIGH = 0; //RGB intensity for high frequency 
float Intensity3HIGH = 0;
float Intensity1LOW = 0;
float Intensity2LOW = 0; //RGB intensity for low frequency
float Intensity3LOW = 0;
int RGBtimerLow = 0;
int RGBvalLow = 0; //RGB timing variables for low freq
int RGBsetLow = 0;
int RGBtimerHigh = 0;
int RGBvalHigh = 127.5; //RGB timing variables for high freq
int RGBsetHigh = 3;
bool conditionMet = false;   //store the condition state of button hold
bool PreviousCondition = false;
unsigned long startTime = 0; //store the start time of button hold
bool pinsHigh = false;
unsigned long newTime;
bool isBluetoothConnected = false;
BluetoothA2DPSink a2dp_sink;
#define SAMPLES 1024             //SAMPLES-pt FFT. Must  be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 40000  //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
#define NUM_BANDS       16          // To change this, you will need to change the bunch of if statements describing the mapping from bins to bands
#define NOISE           800           // Used as a crude noise filter, values below this are ignored
#define LOW_RED_PIN    5
#define LOW_GREEN_PIN  18
#define LOW_BLUE_PIN   19
#define HIGH_RED_PIN    4
#define HIGH_GREEN_PIN  16  //Pin Assignments
#define HIGH_BLUE_PIN   17
#define buttonPinLOW   22
#define buttonPinHIGH  23
unsigned int samplingPeriod;
unsigned long microSeconds;
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vRealHigh[SAMPLES]; //create vector of size SAMPLES to hold real values
double vRealLow[SAMPLES]; //create vector of size SAMPLES to hold real values
double  vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values
double vRealLOW[SAMPLES]; //create vector of size SAMPLES to hold real values
double vRealHIGH[SAMPLES]; //create vector of size SAMPLES to hold real values
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);


void RGBsetup(){ //Set's up RGB timing variables
  RGBtimerLow = RGBtimerLow + 1;
  RGBtimerHigh = RGBtimerHigh + 1;
  if ((RGBtimerLow % 3) == 0){
    RGBvalLow = RGBvalLow + 1;
  }
  if (RGBvalLow == 256){
    RGBsetLow = RGBsetLow + 1;
    RGBvalLow = 0;
  }
  if (RGBsetLow == 6){
    RGBsetLow = 0;
  }
    if ((RGBtimerHigh % 3) == 0){
    RGBvalHigh = RGBvalHigh + 1;
  }
  if (RGBvalHigh == 256){
    RGBsetHigh = RGBsetHigh + 1;
    RGBvalHigh = 0;
  }
  if (RGBsetHigh == 6){
    RGBsetHigh = 0;
  }
}
void RGBdriverHIGH(){ //Set's intensity of RGB vals for high freq light strip
  if (RGBsetHigh == 0){
    Intensity1HIGH = 255;
    Intensity2HIGH = RGBvalHigh;
    Intensity3HIGH = 0;
  }
  if (RGBsetHigh == 1){
    Intensity1HIGH = 255 - RGBvalHigh;
    Intensity2HIGH = 255;
    Intensity3HIGH = 0;
  }
  if (RGBsetHigh == 2){
    Intensity1HIGH = 0;
    Intensity2HIGH = 255;
    Intensity3HIGH = RGBvalHigh;
  }
  if (RGBsetHigh == 3){
    Intensity1HIGH = 0;
    Intensity2HIGH = 255 - RGBvalHigh;
    Intensity3HIGH = 255;
  }
  if (RGBsetHigh == 4){
    Intensity1HIGH = RGBvalHigh;
    Intensity2HIGH = 0;
    Intensity3HIGH = 255;
  }
  if (RGBsetHigh == 5){
    Intensity1HIGH = 255;
    Intensity2HIGH = 0;
    Intensity3HIGH = 255 - RGBvalHigh;
  }
}
void RGBdriverLOW(){ //Set's intensity of RGB vals for low freq light strip
  if (RGBsetLow == 0){
    Intensity1LOW = 255;
    Intensity2LOW = RGBvalLow;
    Intensity3LOW = 0;
  }
  if (RGBsetLow == 1){
    Intensity1LOW = 255 - RGBvalLow;
    Intensity2LOW = 255;
    Intensity3LOW = 0;
  }
  if (RGBsetLow == 2){
    Intensity1LOW = 0;
    Intensity2LOW = 255;
    Intensity3LOW = RGBvalLow;
  }
  if (RGBsetLow == 3){
    Intensity1LOW = 0;
    Intensity2LOW = 255 - RGBvalLow;
    Intensity3LOW = 255;
  }
  if (RGBsetLow == 4){
    Intensity1LOW = RGBvalLow;
    Intensity2LOW = 0;
    Intensity3LOW = 255;
  }
  if (RGBsetLow == 5){
    Intensity1LOW = 255;
    Intensity2LOW = 0;
    Intensity3LOW = 255 - RGBvalLow;
  }
}
void colorsSetting(){ //Function for the constant color setting
  digitalWrite(15,HIGH);
  Serial.println("function call test");
  int highState = digitalRead(buttonPinHIGH);
  int lowState = digitalRead(buttonPinLOW);
  if ((highState == HIGH) and (PreviousHighState == LOW) and (lowState == LOW)){
    colorSetHIGH = colorSetHIGH +1;
  }
  if ((lowState == HIGH) and (PreviousLowState == LOW) and (highState == LOW)){  //Establish Color Setting
    colorSetLOW = colorSetLOW + 1;
  }
  PreviousLowState = lowState;
  PreviousHighState = highState;
  if (colorSetLOW == 9){
    colorSetLOW = 0;
  }
  if (colorSetHIGH == 8){
    colorSetHIGH = 0;
  }

  if (colorSetLOW == 0){
    Intensity1LOW = 255;
    Intensity2LOW = 0;
    Intensity3LOW = 0;
  }
  if (colorSetLOW == 1){
    Intensity1LOW = 255;
    Intensity2LOW = 255;
    Intensity3LOW = 0;
  }
  if (colorSetLOW == 2){
    Intensity1LOW = 0;
    Intensity2LOW = 255;
    Intensity3LOW = 0;
  }
  if (colorSetLOW == 3){
    Intensity1LOW = 0;
    Intensity2LOW = 255;
    Intensity3LOW = 255;
  }
  if (colorSetLOW == 4){
    Intensity1LOW = 0;
    Intensity2LOW = 0;
    Intensity3LOW = 255;
  }
  if (colorSetLOW == 5){
    Intensity1LOW = 255;
    Intensity2LOW = 0;

    Intensity3LOW = 255;
  }
  if (colorSetLOW == 6){
    Intensity1LOW = 255;

    Intensity2LOW = 255;
    Intensity3LOW = 255;
  }
  if (colorSetLOW == 7){
    RGBdriverLOW();
  }
  if (colorSetLOW == 8){
    RGBdriverHIGH();
    Intensity1LOW = Intensity1HIGH;
    Intensity2LOW = Intensity2HIGH;
    Intensity3LOW = Intensity3HIGH;
  }


  if (colorSetHIGH == 0){
    Intensity1HIGH = 255;
    Intensity2HIGH = 0;
    Intensity3HIGH = 0;
  }
  if (colorSetHIGH == 1){
    Intensity1HIGH = 255;
    Intensity2HIGH = 255;
    Intensity3HIGH = 0;
  }
  if (colorSetHIGH == 2){
    Intensity1HIGH = 0;
    Intensity2HIGH = 255;
    Intensity3HIGH = 0;
  }
  if (colorSetHIGH == 3){
    Intensity1HIGH = 0;
    Intensity2HIGH = 255;
    Intensity3HIGH = 255;
  }
  if (colorSetHIGH == 4){
    Intensity1HIGH = 0;
    Intensity2HIGH = 0;
    Intensity3HIGH = 255;
  }
  if (colorSetHIGH == 5){
    Intensity1HIGH = 255;
    Intensity2HIGH = 0;
    Intensity3HIGH = 255;
  }
  if (colorSetHIGH == 6){
    Intensity1HIGH = 255;
    Intensity2HIGH = 255;
    Intensity3HIGH = 255;
  }
  if (colorSetHIGH == 7){
    RGBdriverHIGH();
  }
}

//Function for reactive music setting
void musicSetting(){ 
  digitalWrite(15,LOW);

//set's hardware logic state for bluetooth status
  if (isBluetoothConnected == true){
  digitalWrite(26, HIGH);
  digitalWrite(27, LOW);
}                       
if (isBluetoothConnected == false){
  digitalWrite(26, LOW);
  digitalWrite(27, HIGH);
}

// fills vector with analog data from waveform
  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(32); // A conversion takes about 9.7uS on an ESP32
    vImag[i] = 0;
    while ((micros() - newTime) < samplingPeriod) { /* chill */ }
  }

//FFT Operations
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);   
  FFT.ComplexToMagnitude();
  for (int i = 0; i < SAMPLES; i++) {
    vRealHigh[i] = vReal[i];
    vRealLow[i] = vReal[i];
  }

// frequency filters
  for (int i = 2; i < 19; i++){
    vRealHigh[i] = 0;  //High Pass Filter
  }
  for (int i = 150; i < 264; i++){
    vRealHigh[i] = 0; //Noise Filter
  }
  for (int i = 25; i < 264; i++){
    vRealLow[i] = 0; //Low Pass Filter
  }

 //Counts Bin entrees for the FFT of high and low freq
  int totalEntreesHigh = 0;
  int totalEntreesLow = 0;
  for (int i=2; i < 264; i++){
    totalEntreesHigh = totalEntreesHigh + vRealHigh[i];
  }
  for (int i=2; i < 264; i++){
    totalEntreesLow = totalEntreesLow + vRealLow[i];
  }
  int signalMax = 0;
  int signalMin = 4095;

// Take multiple samples and determine the min and max values
  for (int i = 0; i < 1000; i++) {
    int sample = analogRead(32);

    if (sample > signalMax) {
      signalMax = sample;
    }

    if (sample < signalMin) {
      signalMin = sample;
    }
  }

// Calculate peak-to-peak voltage
  int peakToPeakADC = signalMax - signalMin;
  float peakToPeakVoltage = (peakToPeakADC / 4095.0) * 3.3;  // Convert ADC value to voltage

// Establishes the most dominant high and low frequency
  double peakHigh = FFT.MajorPeak(vRealHigh,  SAMPLES, SAMPLING_FREQUENCY);
  double peakLow = FFT.MajorPeak(vRealLow,  SAMPLES, SAMPLING_FREQUENCY);
  float actualHigh = peakHigh / 440;
  float actualLow = peakLow / 440;

  //invalidates dominate high/low freq if not enough bin entrees
  if (totalEntreesHigh < 100000){
    actualHigh = 0;
  }
  if (totalEntreesLow < 25000){  
    actualLow = 0;
  }
  if ((peakToPeakVoltage < 0.1) and (isBluetoothConnected == false)){
    actualLow = 0;
    actualHigh = 0;
  }

// RGB VALUE OPERATIONS START
if (actualLow < 0.2){
  Intensity1LOW = 0;
  Intensity2LOW = 0;
  Intensity3LOW = 0;
}
if ((actualLow <= 0.4) and (actualLow > 0.3)){
  Intensity1LOW = 255;
  Intensity2LOW = 0;
  Intensity3LOW = 0;
}
if ((actualLow <= 0.63) and (actualLow > 0.4)){
  Intensity1LOW = 255;
  Intensity2LOW = (actualLow - 0.4) * 554;
  Intensity3LOW = 0;
}
if ((actualLow <= 0.95) and (actualLow > 0.63)){
  Intensity1LOW = 255;
  Intensity2LOW = (actualLow - 0.63) * 398 + 127.5;
  Intensity3LOW = 0;
}
if ((actualLow <= 1.44) and (actualLow > 0.95)){
  Intensity1LOW = (1.44 - actualLow) * 260.2 + 127.5;
  Intensity2LOW = 255;
  Intensity3LOW = 0;
}
if ((actualLow <= 2.13) and (actualLow > 1.44)){
  Intensity1LOW = (2.13 - actualLow) * 184;
  Intensity2LOW = 255;
  Intensity3LOW = 0;
}
if (actualLow == 0){
  Intensity1LOW = 0;
  Intensity2LOW = 0;
  Intensity3LOW = 0;
}
if (actualLow > 2.13){
  Intensity1LOW = 0;
  Intensity2LOW = 0;
  Intensity3LOW = 0;
}
if (actualHigh > 50){
  Intensity1HIGH = 0;
  Intensity2HIGH = 0;
  Intensity3HIGH = 0;
}
if ((actualHigh >= 9.69) and (actualHigh <= 50)){
  Intensity1HIGH = 255;
  Intensity2HIGH = 150;
  Intensity3HIGH = 127.5;
}
if ((actualHigh < 9.69) and (actualHigh > 7)){
  Intensity1HIGH = 255;
  Intensity2HIGH = (actualHigh - 7) * 27.87 + 75;
  Intensity3HIGH = 127.5;
}
if ((actualHigh <=7) and (actualHigh > 5.03)){
  Intensity1HIGH = 255;
  Intensity2HIGH = (actualHigh - 5.03) * 38.07;
  Intensity3HIGH = 127.5;
}
if ((actualHigh <= 5.03) and (actualHigh > 3.56)){
  Intensity1HIGH = 255;
  Intensity2HIGH = 0;
  Intensity3HIGH = (actualHigh - 3.56) * 21.68 + 31.875;
}
if((actualHigh <=3.56) and (actualHigh > 2.57)){
  Intensity1HIGH = 255;
  Intensity2HIGH = 0;
  Intensity3HIGH = (actualHigh - 2.57) * 32.195;
}
if ((actualHigh <=2.57) and (actualHigh > 1.85)){
  Intensity1HIGH = 255;
  Intensity2HIGH = (2.57 - actualHigh) * 44.25;
  Intensity3HIGH = 0;
}
if ((actualHigh <=1.85) and (actualHigh > 1.32)){
  Intensity1HIGH = 255;
  Intensity2HIGH = (1.85 - actualHigh) * 60.1 + 31.875;
  Intensity3HIGH = 0;
}
if (actualHigh <= 1.32){
  Intensity1HIGH = 255;
  Intensity2HIGH = 127.5;
  Intensity3HIGH = 0;
}
if (actualHigh == 0){
  Intensity1HIGH = 0;
  Intensity2HIGH = 0;
  Intensity3HIGH = 0;
}
if ((actualLow <= 2.14) and (actualLow >= 0.55)){  
  float Scalar = (2.13 - actualLow) * .61;
  Intensity1LOW = Intensity1LOW * Scalar;
  Intensity2LOW = Intensity2LOW * Scalar;
  Intensity3LOW = Intensity3LOW * Scalar;
}
// RGB VALUE OPERATIONS END


//Saves Previous Intensity Values
float Intensity1HIGHPREV =  Intensity1HIGH;
float Intensity2HIGHPREV =  Intensity2HIGH;
float Intensity3HIGHPREV =  Intensity3HIGH;
float Intensity1LOWPREV = Intensity1LOW;
float Intensity2LOWPREV = Intensity2LOW;
float Intensity3LOWPREV = Intensity3LOW;

// Checks for color change operations
int highState = digitalRead(buttonPinHIGH);
int lowState = digitalRead(buttonPinLOW);
  if ((highState == HIGH) and (PreviousHighState == LOW) and (lowState == LOW)){
    colorModeHIGH = colorModeHIGH +1;
  }
  if ((lowState == HIGH) and (PreviousLowState == LOW) and (highState == LOW)){  //Establish Color Setting
    colorModeLOW = colorModeLOW + 1;
  }
  PreviousLowState = lowState;
  PreviousHighState = highState;

  if (colorModeHIGH >= 3){
    colorModeHIGH = 0;
  }
  if (colorModeLOW >= 3){
    colorModeLOW = 0;
  }
  if (colorModeHIGH == 1){
     Intensity1HIGH =  Intensity2HIGHPREV;
     Intensity2HIGH =  Intensity3HIGHPREV;
     Intensity3HIGH =  Intensity1HIGHPREV;  //High Color Change Operation
  }
  if (colorModeHIGH == 2){
     Intensity1HIGH =  Intensity3HIGHPREV;
     Intensity2HIGH =  Intensity1HIGHPREV;
     Intensity3HIGH =  Intensity2HIGHPREV;
  }
  if (colorModeLOW == 1){
     Intensity1LOW =  Intensity2LOWPREV;  // Low Color Change Operation
     Intensity2LOW =  Intensity3LOWPREV;
     Intensity3LOW =  Intensity1LOWPREV;
     
  }
  if (colorModeLOW == 2){
    Intensity1LOW = Intensity3LOWPREV;
    Intensity2LOW = Intensity1LOWPREV;
    Intensity3LOW = Intensity2LOWPREV;
  }

}



void setup() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 44100,                          // corrected by info from bluetooth
    .bits_per_sample = (i2s_bits_per_sample_t)16,  //the DAC module will only take the 8bits from MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,  // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };
  //Setup Bass Pins
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcSetup(colorCBR, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LOW_RED_PIN, colorCBR);
  ledcSetup(colorCBG, PWM_FREQ, PWM_RESOLUTION);    
  ledcAttachPin(LOW_GREEN_PIN, colorCBG);
  ledcSetup(colorCBB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LOW_BLUE_PIN, colorCBB);

  //Setup High Pins
  ledcSetup(colorCFR, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HIGH_RED_PIN, colorCFR);
  ledcSetup(colorCFG, PWM_FREQ, PWM_RESOLUTION);   
  ledcAttachPin(HIGH_GREEN_PIN, colorCFG);
  ledcSetup(colorCFB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HIGH_BLUE_PIN, colorCFB);
  ledcWrite(PWM_CHANNEL, 128); 
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start("Avas Music Box");
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds 
  pinMode(buttonPinHIGH, INPUT);
  pinMode (buttonPinLOW, INPUT);  //Set up Color Change Pins
  pinMode(15, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(25, OUTPUT); // Set audio output pin
  Serial.begin(115200);
}




void loop() {
isBluetoothConnected = a2dp_sink.is_connected(); // Establish Bluetooth Connectivity
  RGBsetup();
  int highState = digitalRead(buttonPinHIGH);
  int lowState = digitalRead(buttonPinLOW);

  if (highState == HIGH and lowState == HIGH) {
    if (!pinsHigh) { // Start timing when pins go HIGH
      startTime = millis();
      pinsHigh = true;
    }
    
    // Check if 1 seconds have passed since both pins were HIGH
    if (millis() - startTime >= 1000) {
      conditionMet = true;
    }
  }

  else {
    pinsHigh = false; // Reset if any pin goes LOW
    conditionMet = false;
  }
  if ((conditionMet == true) and (PreviousCondition == false)){
    colorSetting = colorSetting + 1;
  }
  PreviousCondition = conditionMet;
  PreviousLowMenu = lowState;
  PreviousHighMenu = highState;
  int Mode = colorSetting % 2;
  if ((Mode == 0) and (isBluetoothConnected == false)){
    colorsSetting();
  }
  if ((Mode == 1) or (isBluetoothConnected == true)){
    musicSetting();
  }

// Reduces green brightness ITS TOO DOMINATING
Intensity2LOW = Intensity2LOW * .5;
Intensity2HIGH = Intensity2HIGH * .5;

//Send RGB Values
ledcWrite(colorCBR, Intensity1LOW);
ledcWrite(colorCBG, Intensity2LOW);
ledcWrite(colorCBB, Intensity3LOW); 
ledcWrite(colorCFR, Intensity1HIGH);
ledcWrite(colorCFG, Intensity2HIGH);
ledcWrite(colorCFB, Intensity3HIGH);
}

