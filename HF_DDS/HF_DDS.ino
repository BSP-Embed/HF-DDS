/* Main application program/sketch for clock generation in HF Rigs. 3-30 Mhz
System generates two clock frequency i.e for VFO & BFO 
Each board has to be calibrated. Calibration sketch in File/Examples/si5351Arduino-Jason 
Thanks to AK2B, SK9NJE, NT7S , VU2SWX, VU3VRL, VU2SPF and many others who have shared
their valuable knowledge for the development of HAM radio. This program is largely 
based on their efforts and in their spirit it is made available to every Ham for 
use in their lovely homebrewed rigs. 
More information(To buy) about the Board, contact Mr Ramesh VU3VRL (vu3vrl@gmail.com)
and Demo available on YouTube BSPEmbed  */

#include <LiquidCrystal.h>
#include <si5351.h> 
#include <Rotary.h>         
#include <Wire.h>                   /* For I2C devices */
#include <EEPROM.h>

#define CALL_SIGN       "VU3SWL"    /* Ham Call Sign */
#define F_MIN           3000000UL     /* in Hz */    
#define F_MAX           30000000UL
#define FREQ_MUL        1UL         /* CLK O/P in 1Hz */
#define SI5351_CALIB    145500UL 
#define BFO_CLK         11997500UL
#define _10MHZ          10000000UL    

#define FREQ_DIVISION   1000.0      /* Decimal Point Position */
#define DEF_STEP_SIZE   1000        /* in Hz */

#define FREQ_MODE       0
#define CHAN_MODE       1

#define LSB             0
#define USB             1

#define ENCODER_A       3                      
#define ENCODER_B       2                      
#define ENCODER_BTN     4                     

#define LCD_RS          5
#define LCD_E           6
#define LCD_D4          7
#define LCD_D5          8
#define LCD_D6          9
#define LCD_D7          10

#define RELAY_PIN       A0      /* For Frontend Filter  switching*/

#define MAGIC_NO        56      /* For EEPROM Detection */
#define MAGIC_ADD       0       /* EEPROM ADDRESS */
#define MODE_ADD        1
#define STEP_SIZE_ADD   2
#define CHAN_FREQ_BASE  15
#define NO_CHANNELS     10 + 1 + 1    /* Channels+VFO+BFO */
#define BFO_MODE        11            /* Last Element of Channels */

#define TRUE            1
#define FALSE           0

#define ERROR_PRESS     0
#define SHORT_PRESS     1
#define MEDIUM_PRESS    2
#define LONG_PRESS      3
#define VLONG_PRESS     4

#define S_PRESS_TIME    500     /* in milli seconds */
#define M_PRESS_TIME    1500
#define L_PRESS_TIME    3000      
#define DLY_SEC         1000    /* Display Hold for Info*/

struct MemoryChan { 
  uint32_t RxFreq;
} ;
/* Default Repeaters Frequencies */
struct MemoryChan Channels[NO_CHANNELS] = {
                                7050000, 
                                7050000, 
                                7080000, 
                                7085000, 
                                7123456, 
                                7010000, 
                                7020000, 
                                7030000, 
                                7040000,  
                                7060000, 
                                7070000,
                                BFO_CLK, 
                                } ;
                              
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);       
Rotary r = Rotary(ENCODER_A, ENCODER_B);
Si5351 si5351;

/* Global Variables */
volatile uint32_t StepSize;        /* step size in KHz. It can vary 1Hz, 10Hz, 100Hz */ 
volatile uint32_t RxDispFreq;
volatile uint8_t Mode;            /* Frequency Or Channel Mode */
volatile uint8_t ChanNo;
volatile uint8_t Band;

boolean FreqChng = FALSE;
boolean ChnChng = FALSE;
boolean StrChn = FALSE;
boolean StrChnChng = FALSE;
boolean TuneBfoFlag = FALSE;

char *ModeFreChlStr[] = {"VFO","CH1","CH2","CH3","CH4","CH5","CH6","CH7","CH8","CH9","C10","BFO"};
char *SSBStr[] = {"LSB", "USB"};

/* Define Macros */
#define WriteMode()     EEPROM.write(MODE_ADD, Mode) 
#define ReadMode()      EEPROM.read(MODE_ADD)

#define RelayOn()       digitalWrite(RELAY_PIN, HIGH)
#define RelayOff()      digitalWrite(RELAY_PIN, LOW)

void setup() {
  lcd.begin(16, 2);                                                    
  lcd.clear();
  ReadEEPROM();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);                    /* Enable pin change interrupt for the encoder */
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  Wire.begin();
  si5351.set_correction(SI5351_CALIB);    /* Calibrate each Board */
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0); 
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  DispInit();
}

void loop() {
  /* Update the display & Clk  for Freq Change*/
  if (FreqChng) {
    FreqChang();
    FreqChng = FALSE;
  }
  /* Update the display & Clk  for Channel Change*/
  if(ChnChng) {
    ChanlChanged();
    ChnChng = FALSE;
  }
  /* Update for button Press */
  switch (get_button()) {
    case SHORT_PRESS:   if (StrChn) StrChnMEM();
                        else if (Mode >= 1 && Mode <= 10)   /* It Channel Mode */
                            ChngModeToFreq();
                        else
                          ChngStepSize();
                        break;
    case MEDIUM_PRESS:  ChngModeToChn(); break;
    case LONG_PRESS:    StoreChn(); break;
    case VLONG_PRESS:   TuneBfo();  break;
    default: break;
  }
  /* Store Channel to Memory */
  if (StrChn && StrChnChng) {
     DispMode();
     lcd.setCursor(0,0);
     StrChnChng = FALSE;
  }
}
                          
/**************************************/
/* Store VFO into EEPROM         */
/**************************************/
void StoreChn(void) {
  if (Mode == FREQ_MODE) {
    StrChn = TRUE;
    lcd.setCursor(0,1);
    lcd.print("Store Chn to MEM");
    Mode = ChanNo = 1;
    DispMode();
    lcd.setCursor(0,0);
    lcd.blink();
  }
}
/**************************************/
/* Store VFO to Channel in EEPROM         */
/**************************************/
void StrChnMEM(void) {
    StrChn = FALSE;
    Channels[ChanNo].RxFreq = RxDispFreq; /* store into Array */
    WriteChannel(ChanNo);               /* store in EEPROM */
    lcd.setCursor(0,0);
    lcd.noBlink();
    lcd.setCursor(0,1);
    lcd.print(" CHANNEL STORED ");
    delay(DLY_SEC);
    lcd.clear();
    DispInit();
}
/**************************************/
/* Display Mode/Channel Numer         */
/* and store in EEPROM                */
/**************************************/
void ChngModeToChn(void) {
    RxDispFreq = Channels[ChanNo].RxFreq;
    Mode = ChanNo;
    DispFreq();
    DispMode();
    WriteMode();
    Mode = CHAN_MODE;
}
/**************************************/
/* Display Mode to Frequency          */
/* and store in EEPROM                */
/**************************************/
void ChngModeToFreq(void) {
  RxDispFreq = Channels[FREQ_MODE].RxFreq;
  Mode = FREQ_MODE;
  DispFreq();
  WriteMode();
  DispMode();
}
/**************************************/
/*Display Step Size & Store in EEPROM */
/**************************************/
void ChngStepSize(void) {
  if ((StepSize *= 10) > 1000000)
    StepSize = 1;
   DispStepSize(); WriteStepSize();
}
/**************************************/
/* BFO Tune & Store in EEPROM*/
/**************************************/
void TuneBfo(void) {
  if (!TuneBfoFlag) {
    TuneBfoFlag = TRUE;
    Mode = BFO_MODE;
    DispMode();
    RxDispFreq =  Channels[BFO_MODE].RxFreq; 
    DispFreq();
  } else {
    lcd.setCursor(0,1);
    lcd.print("   BFO TUNNED   ");
    Mode = FREQ_MODE;
    delay(DLY_SEC);
    RxDispFreq = Channels[Mode].RxFreq;
    DispInit();
    TuneBfoFlag = FALSE;
  }
}
/**************************************/
/* Display Frequency & Store in EEPROM*/
/**************************************/
void FreqChang(void) {
  Channels[Mode].RxFreq = RxDispFreq;
  DispFreq();
  Channels[Mode].RxFreq = RxDispFreq;
  WriteChannel(Mode);
}
/**************************************/
/* Intialize the Display              */
/**************************************/
void DispInit(void) {
  lcd.clear();
  lcd.setCursor(13,0);
  lcd.print("KHz");
  lcd.setCursor(0, 1);
  lcd.print(CALL_SIGN);
  lcd.setCursor(14, 1);
  lcd.print("Hz");
  DispMode();
  DispFreq();  
  DispBand();
  DispStepSize();
}
/**************************************/
/* Display Mode or channel Number     */
/**************************************/
void DispMode(void) {
  lcd.setCursor(0, 0);
  lcd.print(ModeFreChlStr[Mode]);
}
/**************************************/
/* Display Band     */
/**************************************/
void DispBand(void) {
  lcd.setCursor(7, 1);
  lcd.print(SSBStr[Band]);
}
/**************************************/
/* Interrupt service routine for      */
/* encoder for frequency change       */
/**************************************/
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_CW)
    set_frequency(1);
  else if (result == DIR_CCW)
    set_frequency(-1);
}
/**************************************/
/* Change the frequency               */
/* dir = 1    Increment               */
/* dir = -1   Decrement               */
/**************************************/
void set_frequency(short dir){
  if (StrChn == TRUE) {
    switch(dir){
      case 1: if (++ChanNo >= (NO_CHANNELS - 1)) ChanNo = 1; break;
      case -1: if (--ChanNo <= 0) ChanNo = NO_CHANNELS - 2; break;
    }
    Mode = ChanNo;
    StrChnChng = TRUE;
  }
  else if (Mode == FREQ_MODE || Mode == BFO_MODE) {
    switch(dir) {
      case 1: RxDispFreq += StepSize; break;
      case -1: RxDispFreq -= StepSize; break;
    }
    if(RxDispFreq > F_MAX)      /* check overflow */
      RxDispFreq = F_MIN;
    if(RxDispFreq < F_MIN)
      RxDispFreq = F_MAX;
     FreqChng = TRUE;
 } else {
    switch(dir){
      case 1: if (++ChanNo >= (NO_CHANNELS - 1)) ChanNo = 1; break;
      case -1: if (--ChanNo <= 0) ChanNo = NO_CHANNELS - 2; break;
    }
    ChnChng = TRUE; 
 }
}
/**************************************/
/* Display Channel Change & Store      */
/**************************************/
void ChanlChanged() {
  RxDispFreq = Channels[ChanNo].RxFreq;
  DispFreq();
  Mode = ChanNo;
  DispMode();
  WriteMode();
}
/**************************************/
/* Read the button with debouncing    */
/**************************************/
uint8_t get_button() {
  if (!digitalRead(ENCODER_BTN)) {
    delay(20);
    if (!digitalRead(ENCODER_BTN)) {
      long strttime = millis();
      while (!digitalRead(ENCODER_BTN));
      long Duration = millis() -  strttime; 
      if (Duration  > L_PRESS_TIME)           
        return VLONG_PRESS;        
      else if (Duration  > M_PRESS_TIME)           
        return LONG_PRESS;        
      else if (Duration > S_PRESS_TIME)
        return MEDIUM_PRESS;
      else
        return SHORT_PRESS;          
    }
  }
  return ERROR_PRESS;
}
/**************************************/
/* Displays the frequency             */
/**************************************/
void DispFreq(){
  lcd.setCursor(12, 0);
  lcd.print(' ');         /* Clear last digit/segement */
  lcd.setCursor(4, 0);
  lcd.print(RxDispFreq / FREQ_DIVISION,3);                      /* three Digit precision */
  SetClk();                              /* Generate Clock */
}
/**************************************/
/* Displays the step size             */
/**************************************/
void DispStepSize(void) {
  uint8_t x;
  lcd.setCursor(10, 1);
  lcd.print("    ");
  lcd.setCursor(10, 1);
  if(StepSize >= 1000000) {
    x = (StepSize / 1000000);
    if (x < 10) lcd.print("  ");
    else if (x < 100) lcd.print(" ");
    lcd.print(x);
    lcd.print("M");
  } else if (StepSize >= 1000) {
    x = (StepSize / 1000);
    if (x < 10) lcd.print("  ");
    else if (x < 100) lcd.print(' ');
    lcd.print(x);
    lcd.print("K");
  } else {
    if (StepSize < 10) lcd.print("   ");
    else if (StepSize < 100) lcd.print("  ");
    else if (StepSize >= 100) lcd.print(' ');
    lcd.print(StepSize);
  }
}
/**********************************************/
/* Set Clock for Both Receiver & Transmitter */
/*********************************************/
void SetClk() {
  volatile uint32_t bfo = Channels[BFO_MODE].RxFreq;
  volatile uint32_t vfo;

  if (Mode >= 1 && Mode <= 10)
    vfo = Channels[ChanNo]. RxFreq;
  else
    vfo = Channels[FREQ_MODE].RxFreq;  

  if (vfo < _10MHZ) {
    si5351.set_freq((bfo - vfo), 0ULL, SI5351_CLK0);
    Band = LSB;
    DispBand();
    RelayOff();
  } else {
    si5351.set_freq((vfo - bfo), 0ULL, SI5351_CLK0);
    Band = USB; 
    DispBand();
    RelayOn();
  }
  si5351.set_freq(bfo, 0ULL, SI5351_CLK2);
}
/**************************************/
/* Read Frequency, ModeOP, Channel No */
/* If EEPROM is blank, Write Defaults */
/**************************************/
void ReadEEPROM(void){
  uint8_t i;
  if (EEPROM.read(MAGIC_ADD) != MAGIC_NO) { /* New EEPROM & Default Values*/
      EEPROM.write(MAGIC_ADD, MAGIC_NO);
      for (i = 0; i < NO_CHANNELS; i++) {
      RxDispFreq = Channels[i].RxFreq;
      WriteChannel(i);
     }
      StepSize = DEF_STEP_SIZE;
      WriteStepSize();
      Mode = FREQ_MODE;
      WriteMode();
      RxDispFreq = Channels[0].RxFreq;
      Mode = FREQ_MODE;
      ChanNo = 1;
      si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_2MA); /* you can set this to 2MA, 4MA, 6MA or 8MA */
      si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA); /* Need to be done once */
   } else {                                       /* Read Entire Array */
      for (i = 0; i < NO_CHANNELS; i++) {
        ReadChannel(i);
        Channels[i].RxFreq = RxDispFreq;
      }
      Mode = ReadMode();                               /* Restore power off data */
      StepSize = ReadStepSize();
      RxDispFreq = Channels[Mode].RxFreq;
      if (Mode != FREQ_MODE) ChanNo = Mode;
      else ChanNo = 1;
   }  
}
/**************************************/
/*Write Frequency and Mode of Operation*/
/* encoder for frequency change       */
/**************************************/
void WriteChannel(uint8_t MemLoc) {
   uint8_t Add;
   Add = CHAN_FREQ_BASE + (15 * MemLoc); 
   WriteDispFreq(Add);
}
/**************************************/
/*Read Frequency and Mode of Operation*/
/**************************************/
void ReadChannel(uint8_t MemLoc) {
   uint8_t Add;
   Add = CHAN_FREQ_BASE + (15 * MemLoc); 
   ReadDispFreq(Add);
}
/**************************************/
/*Read Frequency From EEPROM as string*/
/**************************************/
void WriteDispFreq(uint8_t FreqAdd) {
  byte i , j;
  String FreqStr;
  FreqStr = String(RxDispFreq);
  i = FreqAdd; j = 0;
  while (FreqStr[j] != '\0') { 
       EEPROM.write(i, FreqStr[j]);  
       i++, j++;
  }  
  EEPROM.write(i, '\0');
}
/**************************************/
/* Read Frequency From EEPROM         */
/* and store as integer               */
/**************************************/
void ReadDispFreq(uint8_t FreqAdd) {
  byte i, j;
  char StrName[10];
  String inString;
  i = FreqAdd; j = 0;
  while ((StrName[j] = EEPROM.read(i)) != '\0') {
           i++; j++;
  }
  StrName[j] = '\0';
  inString = StrName;
  RxDispFreq = inString.toInt();
}
void WriteStepSize() {
  byte i , j;
  String FreqStr;
  FreqStr = String(StepSize);
  i = STEP_SIZE_ADD; j = 0;
  while (FreqStr[j] != '\0') { 
       EEPROM.write(i, FreqStr[j]);  
       i++, j++;
  }  
  EEPROM.write(i, '\0');
}
uint32_t ReadStepSize() {
  byte i, j;
  char StrName[10];
  String inString;
  i = STEP_SIZE_ADD; j = 0;
  while ((StrName[j] = EEPROM.read(i)) != '\0') {
           i++; j++;
  }
  StrName[j] = '\0';
  inString = StrName;
  return (uint32_t) inString.toInt();
}


