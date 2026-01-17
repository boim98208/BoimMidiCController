#define TWO_PI 6.283185307179586476925286766559
#include <math.h>

constexpr int cc1Pot = A0;
constexpr int WaveDepthPot = A1;
constexpr int WaveFreqPot = A2;

constexpr int InflectionPointBut = 12;
constexpr int SwitchModeBut = 11;
constexpr int SwitchWaveBut = 10;

constexpr int CCMAX = 127;
constexpr int CCMID = 63;
constexpr int CCMIN = 0;

constexpr int PBMAX = 16383;
constexpr int PBMID = 8192;
constexpr int PBMIN = 0;

constexpr int SINESTATE = 1;
constexpr int SAWSTATE = 2;
constexpr int SQUARESTATE = 3;

int waveMode = 1;

int smoothenInput = 1023;
// smoothenInput is set at 1023 out of the box so that the IIR filter gets the full 1023 range
int smoothenFinalOutput = 1000;


const int EMAsmoothenFactor = 60;

const int inflectionPointOutput = 30;
//This is the output value, or y-value, of the inflection point on the curve

bool isSwitchModeButPres = false;
bool isSwitchWaveButPres = false;

bool sendingCCData = true;

int inflectionPointCC = 0;
int inflectionPointPB = 511;

byte channel = 0;
byte control = 1;

//unsigned long timer = 0;
const int WaveMax = 1;
const int WaveMin = -1;
//these constants define the mapping with the automatic waves.

unsigned long previousTime;
bool isPrevTimeCaptured = false;
float depth;

int rawWaveVal = 0;
int mapWaveVal = 0;

int smoothModVal = 8683;
float smoothFreqVal = 0;

// change this to constexpr later in arduinoIDE

static double sineModulation(float depth, float freq, long startTimeMS, long currentTimeMS) {

  return depth * sinWave(TWO_PI * (currentTimeMS - startTimeMS) / (1000.0 / freq));

}

static double sawModulation(float depth, float freq, long startTimeMS, long currentTimeMS) {

  return depth * sawWave(2.0 * (currentTimeMS - startTimeMS) / (1000.0 / freq));

}

static double squareModulation(float depth, float freq, long startTimeMS, long currentTimeMS) {

  return depth * squareWave(2.0 * (currentTimeMS - startTimeMS) / (1000.0 / freq));

}

static float EMAFilter(float smoothen, float raw, bool sendingCCData) {
  int smoothenFactor;

  if(sendingCCData){
    smoothenFactor = 4;
  }else{
    smoothenFactor = 6;
  }

  if(abs(raw - smoothen) <= 2){
    return raw;
  }else{
    return (long)(smoothen * smoothenFactor + raw) / (smoothenFactor + 1);
  }
  //return raw;
}

static double sawWave(float input) {
  //saw will have it's top peak be at 0.5, and it's bottom be -1.5
  //so input = 0, saw = 0. 
  //   input = 0.5, saw = 1, 
  //   input = 1.0, saw = 0, 
  //   input = 1.5, saw = -1,
  //   input = 2, saw = 0

  float temp = fmod(input, 2);
  if (temp <= 0.5) {
    return 2.0 * temp;
  } else if (temp <= 1.5) {
    return -2.0 * temp + 2;
  } else {
    return 2.0 * temp - 4;
  }

}

static double squareWave(float input) {
  //   input = [0,1], sqaure = 1, 
  //   input = (1,2], square = -1, 

  float temp = fmod(input, 2);
  if (temp <= 1) {
    return 1.0;
  } else {
    return -1.0;
  }

}

static double sinWave(float input) {
  return sin(input);
}

static float curvePBFreq(float input) {
  //designed so low to halfway point makes vibrato and above that gets cool aliasing stuff
  if (input <= 510) {
    return input * 0.0156862745f;
  } else {
    float x = input;
    return 0.000375292142661f * x * x -
      0.175712718247423f * x;
  }
}

float mapFloat(float x, float in_min, float in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

static int smoothen(int smoothInput, int rawInput, int smoothenFactor) {
  if (abs(rawInput - smoothInput) <= 10) {
    //ensuring exact endpoints
    return rawInput;
  }

  //using exponential moving average
  //greater smoothing factor means more gradual steps
  long temp = (long) smoothInput * smoothenFactor + rawInput;
  // datatype long is needed because of 16-bit int limit
  return temp / (smoothenFactor + 1);
}

static void sendControlChannelData(byte channel, byte control, byte value) {
  Serial.write(0xB0 | channel);
  Serial.write(control);
  Serial.write(value);
}

static void sendPitchBendData(byte channel, int value) {
  // Clamp value to valid MIDI pitch bend range

  byte lsb = value & 0x7F;
  byte msb = (value >> 7) & 0x7F;

  Serial.write(0xE0 | channel);
  Serial.write(lsb);
  Serial.write(msb);
}

static int curveFittingCC(int input, int inflectionPoint) {
  int output;

  if (input <= inflectionPoint) {
    output = map(input, 0, inflectionPoint, 0, inflectionPointOutput);
  } else {
    output = map(input, inflectionPoint + 1, 1023, inflectionPointOutput + 1, 127);
  }
  return output;

}

static int curveFittingPB(int input, int centerPoint) {
  //since I'm using a pot, I can't easily recenter, so I just press the button to go back

  if (input <= centerPoint) {
    return map(input, 0, centerPoint, 0, 8192);
  } else {
    return map(input, centerPoint + 1, 1023, 8193, 16383);
  }

}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(InflectionPointBut, INPUT_PULLUP);
  pinMode(SwitchModeBut, INPUT_PULLUP);
  pinMode(cc1Pot, INPUT);
  pinMode(WaveDepthPot, INPUT);
  pinMode(WaveFreqPot, INPUT);
  pinMode(SwitchWaveBut, INPUT_PULLUP);

}

void loop() {
  int finalOutput;

  // put your main code here, to run repeatedly:
  int mainPotRaw = analogRead(cc1Pot);

  int waveDepthPotRaw = analogRead(WaveDepthPot);
  int inflectionPointButRaw = digitalRead(InflectionPointBut);

  int mainPotMapped = map(mainPotRaw, 1023, 0, 0, 1023);

  float freqPotRaw = map(analogRead(WaveFreqPot), 1023, 0, 0, 1023);
  float freqMapped = curvePBFreq(freqPotRaw);
  float depthPotMapped = mapFloat(analogRead(WaveDepthPot), 1023, 0, 0, 1);

  //only when the WaveDepthPot has an actual value does it add onto the signal
  if (depthPotMapped > 0.01) {
    if (isPrevTimeCaptured == false) {
      previousTime = millis();
      isPrevTimeCaptured = true;
    }

    smoothFreqVal = EMAFilter(smoothFreqVal, freqMapped, sendingCCData);
    //rawModVal = mapFloat(sinModulation(depthPotMapped, smoothFreqVal, previousTime, millis()), -1, 1, 0, 16383);
    
    if(waveMode == SINESTATE){
      rawWaveVal = mapFloat(sineModulation(depthPotMapped, smoothFreqVal, previousTime, millis()), WaveMin, WaveMax, PBMIN, PBMAX);
    }else if(waveMode == SAWSTATE){
      rawWaveVal = mapFloat(sawModulation(depthPotMapped, smoothFreqVal, previousTime, millis()), WaveMin, WaveMax, PBMIN, PBMAX);
    }else if(waveMode == SQUARESTATE){
      rawWaveVal = mapFloat(squareModulation(depthPotMapped, smoothFreqVal, previousTime, millis()), WaveMin, WaveMax, PBMIN, PBMAX);
    }else{
      Serial.println("SOMETHING IS MISSING");
    }
    // rawModVal = mapFloat(rawModVal, WaveMin, WaveMax, PBMIN, PBMAX);
    //Serial.println(smoothFreqVal);
  } else {
    isPrevTimeCaptured = false;
  }


  if (digitalRead(SwitchModeBut) == LOW && !isSwitchModeButPres) {
    sendingCCData = !sendingCCData;
    inflectionPointPB = mainPotMapped;
    //inflectionPointPB being remapped here ensures no sudden jump when just randomly switching

    isSwitchModeButPres = true;

  } else if (digitalRead(SwitchModeBut) == HIGH) {
    isSwitchModeButPres = false;
  }

  

    if (digitalRead(SwitchWaveBut) == LOW && !isSwitchWaveButPres) {
    if(waveMode == SQUARESTATE){
      waveMode = SINESTATE;
    }else{
      waveMode++;
    }
    //inflectionPointPB being remapped here ensures no sudden jump when just randomly switching

    isSwitchWaveButPres = true;

  } else if (digitalRead(SwitchWaveBut) == HIGH) {
    isSwitchWaveButPres = false;
  }
  

  //1023 is the first parameter to inverse input as my pot reads most input at most counterclockwise,
  //which is counter intuitive for my later processing

  if (inflectionPointButRaw == LOW) {
      //when sending cc data
      if (mainPotMapped > 5 & mainPotMapped < 1015) {

        inflectionPointCC = mainPotMapped;
        inflectionPointPB = mainPotMapped;

      } else {
        inflectionPointCC = CCMIN;
        //resets the cc to be a linear curve
        inflectionPointPB = PBMID;
      }
      //when sending PB data
  }

    smoothenInput = mainPotMapped;
  // don't forget to use different mapping settings if you're sending to
  // cc values or pitch wheel values. 


  if (sendingCCData) {
    //sending control channel data
    finalOutput = curveFittingCC(smoothenInput, inflectionPointCC);
    mapWaveVal = map(rawWaveVal, PBMIN, PBMAX, CCMIN, CCMAX) - CCMID;

    if (mapWaveVal > 1) {
      finalOutput += mapWaveVal;
    }

    smoothenFinalOutput = EMAFilter(smoothenFinalOutput, finalOutput, sendingCCData);
    smoothenFinalOutput = constrain(smoothenFinalOutput, CCMIN, CCMAX);

   sendControlChannelData(channel, control, (int) smoothenFinalOutput);
  } else {
    //sending pitch bend data

    finalOutput = curveFittingPB(smoothenInput, inflectionPointPB);

    if(rawWaveVal > 0){
      finalOutput += rawWaveVal - PBMID;
    }
    //subtracting by PBMid so that it actually goes down because I normalized rawWaveVal to be positive

    smoothenFinalOutput = EMAFilter(smoothenFinalOutput, finalOutput, sendingCCData);
    smoothenFinalOutput = constrain(smoothenFinalOutput, PBMIN, PBMAX);


    sendPitchBendData(channel, (int) smoothenFinalOutput);
  }

}