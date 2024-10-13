

#include <EEPROM.h>
#include "SevSeg.h"
#include <Bounce2.h>
#include <STM32TimerInterrupt.h>
#include <math.h>
SevSeg sevseg;
#define Ko PA15
#define Kp PF7
#define Kn PF6
#define Kb PA12
#define S1 PB5
#define S2 PB4
#define LS1 PB6
#define LS2 PB3
#define LOUT PA1
#define Loff PA0
#define L1 PA2
#define L2 PA3
#define L3 PA4
#define L4 PA5
#define NTC PA6
#define LDama PB0
#define Pot PA7

//Effect
#define LL 75
#define TL 50
#define LCC 125
#define ThC 1 
#define LP 400
#define TP 1
#define LS 150
#define TS 1
#define LO 75
#define TO 1
#define FON 300
#define FOF 100
#define SP 1500
#define ScAB 1 

//EEPROM
#define eepL 10
int eepromAddress = 0;

// تعریف متغیرهای مربوط به LEDها
#define L_C 4
int leds[L_C] = {L1, L2, L3, L4};
int cL = 0;

bool SH1 = 0; bool SH2 = 0; bool SHM = 0; bool lLS = 0; volatile unsigned long lM1 = 0; volatile unsigned long lM2 = 0; volatile unsigned long lLT1 = 0; volatile unsigned long lLT2 = 0; bool ledState1 = false;
bool ledState2 = false; volatile unsigned long lastDebounceTime1 = 0; volatile unsigned long lastDebounceTime2 = 0; int cM = 0; bool inMenu = false; volatile unsigned long kopt = 0; bool kowp = false;
bool fS = false; unsigned long fT = 0; bool showPASS = false; volatile unsigned long firstTrueSHM = 0; unsigned long PASSti = 0;

// تعریف متغیرهای مربوط به سنسورها
volatile unsigned long pC1 = 0;
volatile unsigned long pC2 = 0;
volatile unsigned long rpm1 = 0;
volatile unsigned long rpm2 = 0;

// تعریف متغیرهای مربوط به حالت‌ها
enum State { RR, LA, LB, LC, TT };
State state = RR;

enum State_2 { RR_2, LC_2, LA_2, LB_2 };
State_2 state_2 = RR_2;

// تعریف متغیرهای مربوط به منو

String mOL1[5] = {"FAN1", "FAN2", "CLOC", "REST", "CHNL"};
String mOL2F1[6] = {"LEV1", "LEV2", "LEV3", "LEV4", "CALB", "PONT"};
String mOL2F2[6] = {"LEV1", "LEV2", "LEV3", "LEV4", "CALB", "PONT"};
String mOL2T[2] = {"SHFT", "OUTP"};
String mOL2R[3] = {"FAN1", "FAN2", "CLOC"};
String mCHNL[1] = {"FAST"};
//
String mOL1_2[4] = {"FAN1", "CLOC", "REST", "CHNL"};
String mOL2F1_2[6] = {"LEV1", "LEV2", "LEV3", "LEV4", "CALB", "PONT"};
String mOL2T_2[1] = {"OUTP"};
String mOL2R_2[2] = {"FAN1", "CLOC"};
String* cMO; int pMI = 0; int mI = 0; int xmI = 3;

//
int iL1F1 = 1000;   int iL2F1 = 1050;   int iL3F1 = 1100;   int iL4F1 = 1150;   int iCF1 = 1001;   int iPFF1 = 1;
int iL1F2 = 1200;   int iL2F2 = 1250;   int iL3F2 = 1300;   int iL4F2 = 1350;   int iCF2 = 1002;   int iPFF2 = 2;
int iSC = 1;   int iTC = 2;
//
int L1F1 = iL1F1;   int L2F1 = iL2F1;   int L3F1 = iL3F1;   int L4F1 = iL4F1;   int CF1 = iCF1;   int PFF1 = iPFF1;
int L1F2 = iL1F2;   int L2F2 = iL2F2;   int L3F2 = iL3F2;   int L4F2 = iL4F2;   int CF2 = iCF2;   int PFF2 = iPFF2;
int SC = iSC;   int TC = iTC;
int qCHNL = 1;
int qCHNLt = qCHNL;
//
int L1F1temp = L1F1;   int L2F1temp = L2F1;   int L3F1temp = L3F1;   int L4F1temp = L4F1;   int CF1temp  =  CF1;   int PFF1temp = PFF1;
int L1F2temp = L1F2;   int L2F2temp = L2F2;   int L3F2temp = L3F2;   int L4F2temp = L4F2;   int CF2temp  =  CF2;   int PFF2temp = PFF2;
int SCtemp = SC;   int TCtemp = TC;

// تعریف متغیرهای مربوط به تایمرها
unsigned long ti; unsigned long Lg = 0; unsigned long lastChangeTime = 0; const unsigned long saveDelay = eepL * 1000; bool savePending = false;

// تعریف متغیرهای مربوط به دما
volatile int Shift = 3; volatile int TPc = 1; const int errorThreshold = 5; int errorCounter = 0; volatile int LDamaOFF = 0; unsigned long LOST = 0; int AllTry = 0; unsigned long prM = 0; const long interval = 400;
bool displayDISN = false; float tSUM = 0; int spC = 0; float temperature = 0; unsigned long BPST = 0; bool BP = false; unsigned long BT = 0; bool BS = false; bool SDR = false; unsigned long DDD = 0; unsigned long TPST = 0;
bool TPDA = false; float LRT = 0; unsigned long RTDST = 0; bool referenceTempChanged = false; unsigned long LPRT = 0; float RefreshDama = 0.5; int RateDama = 10; unsigned long RefreshPot = 300; float Hysteresis = 3.1; int DelayTemp = 7;
const float A = 1.009249522e-03;
const float B = 2.378405444e-04;
const float C = 2.019202697e-07;
STM32Timer tempRefreshTimer(TIM3);

Bounce debouncerKo = Bounce(); Bounce debouncerKp = Bounce(); Bounce debouncerKn = Bounce(); Bounce debouncerKb = Bounce();

bool checkLEVs(int value) { for (int i = 50; i <= 9950;  i += 50) { if (value == i) { return true; } } return false; }
bool checkCFs (int value) { for (int i = 1;  i <= 10000; i++    ) { if (value == i) { return true; } } return false; }
bool checkPFFs(int value) { for (int i = 1;  i <= 100;   i++    ) { if (value == i) { return true; } } return false; }
bool checkSC  (int value) { for (int i = 1;  i <= 61;    i++    ) { if (value == i) { return true; } } return false; }
bool checkTC  (int value) { for (int i = 1;  i <= 10000; i++    ) { if (value == i) { return true; } } return false; }

void countPulse1() { pC1++; }

void countPulse2() { pC2++; }

void setup() {

  byte numDigits = 4; byte digitPins[] = {
  PB2, //DIGIT-1
  PB15, //DIGIT-2
  PA9, //DIGIT-3
  PA10 //DIGIT-4
  }; byte segmentPins[] = {
  PB11, //seg-A
  PA11, //seg-B
  PB14, //seg-C
  PB10, //seg-D
  PB1, //seg-E
  PB13, //seg-F
  PA8, //seg-G
  PB12 //seg-DC
  }; bool resistorsOnSegments = false; byte hardwareConfig = COMMON_CATHODE; bool updateWithDelays = false; bool leadingZeros = true; bool disableDecPoint = false;
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros, disableDecPoint); sevseg.setBrightness(20);

  // تنظیمات مربوط به پین‌ها
  pinMode(S1, INPUT); attachInterrupt(digitalPinToInterrupt(S1), countPulse1, RISING); pinMode(S2, INPUT); attachInterrupt(digitalPinToInterrupt(S2), countPulse2, RISING);
  
  pinMode(Kn, INPUT_PULLUP); pinMode(Kp, INPUT_PULLUP); pinMode(Kb, INPUT_PULLUP); pinMode(LS1, OUTPUT); pinMode(LS2, OUTPUT); pinMode(LOUT, OUTPUT); pinMode(Loff, OUTPUT); digitalWrite(Loff, HIGH);

  for (int i = 0; i < L_C; i++) { pinMode(leds[i], OUTPUT); digitalWrite(leds[i], LOW); }

  // تنظیمات مربوط به دماسنج
  pinMode(LDama, OUTPUT); pinMode(Pot, INPUT); digitalWrite(LDama, HIGH);

  // تنظیمات مربوط به تایمر
  tempRefreshTimer.attachInterruptInterval(RefreshDama / RateDama * 1000000, sampleTemperature);

  // تنظیمات مربوط به دکمه‌ها
  debouncerKo.attach(Ko, INPUT_PULLUP); debouncerKo.interval(5); debouncerKp.attach(Kp, INPUT_PULLUP); debouncerKp.interval(5); debouncerKn.attach(Kn, INPUT_PULLUP); debouncerKn.interval(5); debouncerKb.attach(Kb, INPUT_PULLUP); debouncerKb.interval(5);

  //
  cL    = EEPROM.read(eepromAddress); if (cL < 0 || cL > L_C) { cL = 0; EEPROM.write(eepromAddress, cL); }
  qCHNL = EEPROM.read(2); if (qCHNL != 1 && qCHNL != 2) { qCHNL = 1; EEPROM.write(2, qCHNL); }

  EEPROM.get(3,  L1F1); if (!checkLEVs(L1F1)) { L1F1 = iL1F1; EEPROM.put(3,  L1F1); }
  EEPROM.get(7,  L2F1); if (!checkLEVs(L2F1)) { L2F1 = iL2F1; EEPROM.put(7,  L2F1); }
  EEPROM.get(11, L3F1); if (!checkLEVs(L3F1)) { L3F1 = iL3F1; EEPROM.put(11, L3F1); }
  EEPROM.get(15, L4F1); if (!checkLEVs(L4F1)) { L4F1 = iL4F1; EEPROM.put(15, L4F1); }
  EEPROM.get(19,  CF1); if (!checkCFs  (CF1)) { CF1  =  iCF1; EEPROM.put(19,  CF1); }
  EEPROM.get(23, PFF1); if (!checkPFFs(PFF1)) { PFF1 = iPFF1; EEPROM.put(23, PFF1); }

  EEPROM.get(27, L1F2); if (!checkLEVs(L1F2)) { L1F2 = iL1F2; EEPROM.put(27, L1F2); }
  EEPROM.get(31, L2F2); if (!checkLEVs(L2F2)) { L2F2 = iL2F2; EEPROM.put(31, L2F2); }
  EEPROM.get(35, L3F2); if (!checkLEVs(L3F2)) { L3F2 = iL3F2; EEPROM.put(35, L3F2); }
  EEPROM.get(39, L4F2); if (!checkLEVs(L4F2)) { L4F2 = iL4F2; EEPROM.put(39, L4F2); }
  EEPROM.get(43,  CF2); if (!checkCFs  (CF2)) { CF2  =  iCF2; EEPROM.put(43,  CF2); }
  EEPROM.get(47, PFF2); if (!checkPFFs(PFF2)) { PFF2 = iPFF2; EEPROM.put(47, PFF2); }

  EEPROM.get(51, SC); if (!checkSC(SC)) { SC = iSC; EEPROM.put(51, SC); }
  EEPROM.get(55, TC); if (!checkTC(TC)) { TC = iTC; EEPROM.put(55, TC); }
  //
  L1F1temp = L1F1;   L2F1temp = L2F1;   L3F1temp = L3F1;   L4F1temp = L4F1;   CF1temp = CF1;   PFF1temp = PFF1;
  L1F2temp = L1F2;   L2F2temp = L2F2;   L3F2temp = L3F2;   L4F2temp = L4F2;   CF2temp = CF2;   PFF2temp = PFF2;
  SCtemp = SC;   TCtemp = TC;
  //
 if (cL == 0) { digitalWrite(Loff, HIGH); for (int i = 0; i < L_C; i++) { digitalWrite(leds[i], LOW); } }
 else { digitalWrite(Loff, LOW); for (int i = 0; i < L_C; i++) { digitalWrite(leds[i], (i == cL - 1) ? HIGH : LOW); } }
}

void rV(String rT) {
 if      (rT == "F1CR") { L1F1temp = iL1F1; EEPROM.put(3,  L1F1temp); L2F1temp = iL2F1; EEPROM.put(7,  L2F1temp); L3F1temp = iL3F1; EEPROM.put(11, L3F1temp); L4F1temp = iL4F1; EEPROM.put(15, L4F1temp); CF1temp  =  iCF1; EEPROM.put(19,  CF1temp); PFF1temp = iPFF1; EEPROM.put(23, PFF1temp); }
 else if (rT == "F2CR") { L1F2temp = iL1F2; EEPROM.put(27, L1F2temp); L2F2temp = iL2F2; EEPROM.put(31, L2F2temp); L3F2temp = iL3F2; EEPROM.put(35, L3F2temp); L4F2temp = iL4F2; EEPROM.put(39, L4F2temp); CF2temp  =  iCF2; EEPROM.put(43,  CF2temp); PFF2temp = iPFF2; EEPROM.put(47, PFF2temp); }
 else if (rT ==  "CCR") { SCtemp = iSC; EEPROM.put(51, SCtemp); TCtemp = iTC; EEPROM.put(55, TCtemp); }
}

void loop() {
  debouncerKo.update(); debouncerKp.update(); debouncerKn.update(); debouncerKb.update();
  static bool ds1 = true; static volatile unsigned long lastSC = 0; static volatile unsigned long lastTC = 0;

 if (state != LC || (state == LC && pMI != 3)) { 
   if (millis() - lastSC >= SC * 1000) { ds1 = !ds1; lastSC = millis(); }
   if (millis() - lM1 >= 1000) {
     detachInterrupt(digitalPinToInterrupt(S1)); rpm1 = (60 * 1000 / float(millis() - lM1)) * pC1 / PFF1; pC1 = 0; lM1 = millis();
     attachInterrupt(digitalPinToInterrupt(S1), countPulse1, RISING); int cR1 = int((rpm1 / 1000.0) * CF1); if (cR1 < 0) { cR1 = 0; }
     if (ds1 || qCHNL == 2) { 
       if (state != TT) { // اضافه کردن شرط برای جلوگیری از نمایش RPM در حالت TT
         sevseg.setNumber(cR1, 0); 
       }
       digitalWrite(LS1, HIGH); digitalWrite(LS2, LOW); 
     } 

     if (digitalRead(L1) == HIGH) { SH1 = (rpm1 > L1F1) ? 1 : 0; if (qCHNL == 2) { SH2 = 1; } else { SH2 = (rpm2 > L1F2) ? 1 : 0; } }
     else if (digitalRead(L2) == HIGH) { SH1 = (rpm1 > L2F1) ? 1 : 0; if (qCHNL == 2) { SH2 = 1; } else { SH2 = (rpm2 > L2F2) ? 1 : 0; } }
     else if (digitalRead(L3) == HIGH) { SH1 = (rpm1 > L3F1) ? 1 : 0; if (qCHNL == 2) { SH2 = 1; } else { SH2 = (rpm2 > L3F2) ? 1 : 0; } }
     else if (digitalRead(L4) == HIGH) { SH1 = (rpm1 > L4F1) ? 1 : 0; if (qCHNL == 2) { SH2 = 1; } else { SH2 = (rpm2 > L4F2) ? 1 : 0; } }
     else { SH1 = 0; SH2 = 0; }
   }

   if (millis() - lM2 >= 1000) {
     detachInterrupt(digitalPinToInterrupt(S2)); rpm2 = (60 * 1000 / float(millis() - lM2)) * pC2 / PFF2; pC2 = 0; lM2 = millis();
     attachInterrupt(digitalPinToInterrupt(S2), countPulse2, RISING); int cR2 = int((rpm2 / 1000.0) * CF2); if (cR2 < 0) { cR2 = 0; }
     if (!ds1 && qCHNL != 2) { 
       if (state != TT) { // اضافه کردن شرط برای جلوگیری از نمایش RPM در حالت TT
         sevseg.setNumber(cR2, 0); 
       }
       digitalWrite(LS1, LOW); digitalWrite(LS2, HIGH); 
     }
   }
 }

 SHM = (SH1 && SH2) ? 1 : 0; if (SHM) { if (firstTrueSHM == 0) { firstTrueSHM = millis(); }
 if (millis() - firstTrueSHM >= TC * 1000) { digitalWrite(LOUT, HIGH); lLS = 1; lastTC = millis(); }
 }
 else { firstTrueSHM = 0; if (lLS) { digitalWrite(LOUT, LOW); lLS = 0; } }
 sevseg.refreshDisplay();

   switch (state) { // Menu1 ------------------------------------

    case RR: // Layer RR of Menu1 / 1 - 4 FAN Mode
      if (debouncerKb.fell()) { state = TT; }
      else {

        if (debouncerKp.fell()) {
          if (cL < L_C) { cL++; } cL = min(cL, L_C); lastChangeTime = millis(); savePending = true; }
          else if (debouncerKn.fell()) {
            if (cL > 0) { cL--; } lastChangeTime = millis(); savePending = true;
          }

        if (savePending && (millis() - lastChangeTime >= saveDelay)) {
          int currentCL = EEPROM.read(eepromAddress);
          if (currentCL != cL) { EEPROM.write(eepromAddress, cL); }
          savePending = false;
        }

        if (cL == 0) {
          digitalWrite(Loff, HIGH);
          for (int i = 0; i < L_C; i++) { digitalWrite(leds[i], LOW); }
        }

        else {
          digitalWrite(Loff, LOW);
          for (int i = 0; i < L_C; i++) { digitalWrite(leds[i], (i == cL - 1) ? HIGH : LOW); }
        }
        
        if (debouncerKo.fell()) { state = LA; cMO = mOL1; xmI = 4; mI = 0; ti = millis(); }

      }
    break;

    case LA: //start Layer LA of Menu1
      if      (debouncerKn.fell()) { if (++mI > xmI) mI = xmI; }
      else if (debouncerKp.fell()) { if (--mI < 0) mI = 0; } sevseg.setChars(cMO[mI].c_str());
      if      (debouncerKo.fell()) { pMI = mI;
        if (mI == 4) { state = LC; mI = 0; }
        else { state = LB;
          switch (mI) {
            case 0: cMO = mOL2F1; xmI = 5; break; // "FAN1"
            case 1: cMO = mOL2F2; xmI = 5; break; // "FAN2"
            case 2: cMO = mOL2T;  xmI = 1; break; // "CLOC"
            case 3: cMO = mOL2R;  xmI = 2; break; // "REST"
            case 4: cMO = mCHNL;  xmI = 0; break; // "CHNL"
          } mI = 0;
        } ti = millis();
      } 
      else if (debouncerKb.fell()) { state = RR; ti = millis(); }
    break; //END Layer LA of Menu1

    case LB: //start Layer LB of Menu1
     if      (debouncerKn.fell()) { if (++mI > xmI) mI = xmI; }
     else if (debouncerKp.fell()) { if (--mI < 0) mI = 0; } sevseg.setChars(cMO[mI].c_str());
     if      (debouncerKo.fell()) { state = LC; ti = millis(); }
     else if (debouncerKb.fell()) { state = LA; cMO = mOL1; xmI = 4; mI = pMI; ti = millis(); }
    break; //END Layer LB of Menu1

    case TT: // حالت دماسنج
      if (spC >= RateDama) { temperature = tSUM / spC; tSUM = 0; spC = 0; char tempStr[5]; dtostrf(temperature, 4, 1, tempStr); sevseg.setChars(tempStr); uint8_t segments[4]; sevseg.getSegments(segments); segments[3] = 0b01011000; sevseg.setSegments(segments); }
      if (debouncerKb.fell()) { state = RR; }
      else {
        //ShowAllTry
        if (Shift == 3 && digitalRead(Kp) == LOW) { sevseg.setNumber(AllTry, 0); }

        //ShowCountTemp
        if (Shift == 3 && digitalRead(Kn) == LOW) { sevseg.setNumber(LDamaOFF, 0); }

        //ResetCountTemp
        if (Shift == 3 && digitalRead(Ko) == LOW) {
          if (!BP) { BPST = millis(); BP = true; } unsigned long BPD = millis() - BPST;
          if      (BPD < 1000) { sevseg.setChars("PUSH"); }
          else if (BPD < 2000) { sevseg.setChars(" TO "); }
          else if (BPD < 3000) { sevseg.setChars("REST"); }
          else {
            if (millis() % 100 < 50) { sevseg.setChars("PASS"); }
            else { sevseg.setChars("    "); } LDamaOFF = 0;
          }
        } else {
          BP = false; // Reset the process if the button is released
        }
      }
    break;
  }

  sevseg.refreshDisplay();
}

void sampleTemperature() {
  int adcValue = analogRead(NTC);
  if (adcValue == 0) {
    errorCounter++;
    if (errorCounter >= errorThreshold) {
      if (millis() - prM >= interval) { prM = millis(); displayDISN = !displayDISN; sevseg.setChars(displayDISN ? "Eror" : "    "); } digitalWrite(LDama, LOW); TPc = 2;
    }
  }
  else {
    errorCounter = 0; float resistance = (1023.0 / adcValue - 1) * 10000; float temp = 1.0 / (A + B * log(resistance) + C * pow(log(resistance), 3)) - 273.15; tSUM += temp; spC++;

    if (millis() - LPRT >= RefreshPot) {
      LPRT = millis(); float RefT = map(analogRead(Pot), 0, 1023, 10, 90);
      if (abs(RefT - LRT) >= 1) { LRT = RefT; referenceTempChanged = true; RTDST = millis(); }
    }

    if (temp >= LRT) {
if (TPc != 2) {
        if (millis() - LOST >= 2000) { LDamaOFF++; AllTry++; } LOST = millis(); TPc = 2;
      }
      digitalWrite(LDama, LOW); TPDA = false;
    }
    
    else if (temp <= LRT - Hysteresis) {
      if (TPc != 1 && !TPDA) { TPST = millis(); TPDA = true; }
      if (TPDA) {
        if (millis() - TPST >= DelayTemp * 1000) { TPc = 1; digitalWrite(LDama, HIGH); TPDA = false; }
        else {
          if (millis() % 1000 < 500) { digitalWrite(LDama, HIGH); }
          else { digitalWrite(LDama, LOW); }
        }
      }
    }
  }
}
