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

#define CALL_SIGN       "VU3TXF"    /* Your Ham Call Sign */
#define SI5351_CALIB    144000UL 

#define F_MIN           500000UL    /* in Hz */    
#define F_MAX           30000000UL
#define FREQ_MUL        100UL         /* CLK O/P in 1Hz */
#define CLK1_FREQ       455000UL
#define _10MHZ          10000000UL
#define SSB_MAX         12002500UL
#define SSB_MIN         11997500UL   

#define FREQ_DIVISION   1000.0      /* Decimal Point Position */
#define DEF_STEP_SIZE   1000        /* in Hz */

#define FREQ_MODE       0
#define CHAN_MODE       1

#define ENCODER_A       3                      
#define ENCODER_B       2                      
#define ENCODER_BTN     4                     

#define LCD_RS          5
#define LCD_E           6
#define LCD_D4          7
#define LCD_D5          8
#define LCD_D6          9
#define LCD_D7          10

#define LSB_USB_SW      A1

#define RELAY_PIN       A0      /* For Frontend Filter  switching*/

#define MAGIC_NO        68      /* For EEPROM Detection */
#define MAGIC_ADD       0       /* EEPROM ADDRESS */
#define MODE_ADD        1
#define SSBMODE_ADDR    2
#define STEP_SIZE_ADD   3
#define CHAN_FREQ_BASE  15

#define NO_CHANNELS     10
#define STORE_CH        13

#define LSB_MODE        11    /* Must Not Change */
#define USB_MODE        12

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

typedef struct  { 
  uint32_t RxFreq;
}MemoryChan;

/* Default Repeaters Frequencies */
MemoryChan Channels[] = {
                        7050000,   /* VFO Mode Freq   */
                        7040000,
                        7050000,   /* Channel1 .. 10 */
                        7080000, 
                        7085000, 
                        7123456, 
                        7150000,
                        7060000,  
                        7070000,
                        7090000,
                        7100000,
                        11997500,     /* LSB */
                        12002500,     /* USB */
                       };
                              
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);       
Rotary r = Rotary(ENCODER_A, ENCODER_B);
Si5351 si5351;

/* Global Variables */
volatile uint32_t StepSize;        /* step size in KHz. It can vary 10Hz, 50, 100Hz, 1K.. */ 
volatile uint32_t RxDispFreq;
volatile uint8_t  Mode;            /* Frequency Or Channel Mode */
volatile int8_t   ChanNo;
volatile uint8_t  SSBMode;

boolean FreqChng          = false;
boolean ChnChng           = false;
boolean StrChn            = false;
boolean StrChnChng        = false;
boolean TuneBfoFlag       = false;
boolean ChangeSSBModeFlag = false;    /* Display Change SSB Flag */
boolean StoreSSBFlag      = false;



char *ModeFreChlStr[] = {"VFO","CH1","CH2","CH3","CH4","CH5","CH6","CH7","CH8","CH9","C10","LSB", "USB"};

/* Define Macros */
#define StoreSSBMode()  EEPROM.write(SSBMODE_ADDR, SSBMode)
#define WriteMode()     EEPROM.write(MODE_ADD, Mode) 
#define ReadMode()      EEPROM.read(MODE_ADD)

#define RelayOn()       digitalWrite(RELAY_PIN, HIGH)
#define RelayOff()      digitalWrite(RELAY_PIN, LOW)

void setup() {
  SwitchInit();
  RelayInit();
  EncoderInit();
  Wire.begin();
  Si5351Init();
  ReadEEPROM();
  DispInit();
  sei();
}

void loop() {
  /* Update the display & Clk  for Freq Change*/
  if (FreqChng) {
    FreqChang();
    FreqChng = false;
  }
  /* Update the display & Clk  for Channel Change*/
  if(ChnChng) {
    ChanlChanged();
    ChnChng = false;
  }
  /* Update the display & Clk  for SSB Change*/
  if (ChangeSSBModeFlag) {
    ChangeSSBBand();
    ChangeSSBModeFlag = false;
  }
  
  /* Update for button Press */
  switch (get_button()) {
    case SHORT_PRESS:   if (StrChn) StrChnMEM();
                        else if (Mode >= 1 && Mode <= 10)   /* It Channel Mode */
                            ChngModeToFreq();
                        else
                          ChngStepSize();
                        break;
    case MEDIUM_PRESS:  if (TuneBfoFlag) {
                          TuneBfoFlag = false;
                          DispSSBTuned ();
                        } else 
                          ChngModeToChn();
                        break;
    case LONG_PRESS:    StoreChn(); break;
    case VLONG_PRESS:   TuneSideBand();  break;
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
/* Store VFO into EEPROM              */
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
/* Store VFO to Channel in EEPROM     */
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
/*******************************************/
/* Store LSB/USB Value & Mode in EEPROM     */
/*******************************************/
void DispSSBTuned (void) {
  lcd.setCursor(0,1);
  if (SSBMode == LSB_MODE)
    lcd.print("   LSB TUNNED!  ");
  else
    lcd.print("   USB TUNNED!  ");
  Mode = FREQ_MODE;
  delay(DLY_SEC);
  RxDispFreq = Channels[Mode].RxFreq;
  lcd.noBlink();
  DispInit();
  TuneBfoFlag = false;
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
  if (StepSize == 50)       StepSize /= 5;
  else if (StepSize == 100) StepSize /= 2;
  else if ((StepSize /= 10) < 10)
    StepSize = 1000000;
  DispStepSize(); WriteStepSize();
}
/**************************************/
/* Tune Side Band & Store in EEPROM   */
/**************************************/
void TuneSideBand(void) {
  if (!TuneBfoFlag) {
    TuneBfoFlag = true;
    Mode = SSBMode;
    DispMode();
    RxDispFreq =  Channels[SSBMode].RxFreq; 
    DispFreq();
   } 
}
/**************************************/
/* Change Side Band & Store in EEPROM   */
/**************************************/
void ChangeSSBBand (void) {
if (digitalRead(LSB_USB_SW) == LOW) {
      if (SSBMode == LSB_MODE)
        SSBMode = USB_MODE;
      else
        SSBMode = LSB_MODE;
      DispBand();
      StoreSSBMode();
      SetClk();
      if (TuneBfoFlag) {
        TuneBfoFlag   = false;
        TuneSideBand();
      }
   }
}
/*****************************************************/
/* Display Frequency, OutPut Clock  & Store in EEPROM*/
/*****************************************************/
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
  lcd.begin(16, 2);                                                    
  lcd.clear();
  lcd.setCursor(14,0);
  lcd.print("Hz");
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
  lcd.print(ModeFreChlStr[SSBMode]);
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
/* Interrupt service routine for      */
/* LSB/USB Switch                     */
/**************************************/
ISR (PCINT1_vect) {
  ChangeSSBModeFlag = true;
}
/**************************************/
/* Change the frequency               */
/* dir = 1    Increment               */
/* dir = -1   Decrement               */
/**************************************/
void set_frequency(short dir){
  if (StrChn == TRUE) {
    switch(dir){
      case 1: if (++ChanNo > NO_CHANNELS) ChanNo = 1; break;
      case -1: if (--ChanNo <= 0) ChanNo = NO_CHANNELS; break;
    }
    Mode = ChanNo;
    StrChnChng = TRUE;
  }
  else if (Mode == FREQ_MODE) {
    switch(dir) {
      case 1: RxDispFreq += StepSize; break;
      case -1: RxDispFreq -= StepSize; break;
    }
    if(RxDispFreq > F_MAX)      /* check overflow */
      RxDispFreq = F_MIN;
    if(RxDispFreq < F_MIN)
      RxDispFreq = F_MAX;
     FreqChng = TRUE;
 } else if (Mode == LSB_MODE || Mode == USB_MODE) {
    switch(dir) {
      case 1: RxDispFreq += StepSize; break;
      case -1: RxDispFreq -= StepSize; break;
    }
    if(RxDispFreq > SSB_MAX)      /* check overflow */
      RxDispFreq = SSB_MIN;
    if(RxDispFreq < SSB_MIN)
      RxDispFreq = SSB_MAX;
     FreqChng = true;
 }  else {
    switch(dir){
      case 1: if (++ChanNo > NO_CHANNELS) ChanNo = 1; break;
      case -1: if (--ChanNo <= 0) ChanNo = NO_CHANNELS; break;
    }
    ChnChng = TRUE; 
 }
}
/**********************************************/
/* Display Channel Change, Store, Set Clk     */
/**********************************************/
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
    delay(10);
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
  uint8_t x;
  uint32_t y;
  lcd.setCursor(13, 0);
  lcd.print(' ');         /* Clear last digit/segement */
  lcd.setCursor(4, 0);
  x = RxDispFreq / 1000000;
  y = RxDispFreq - (x * 1000000);
  lcd.print(x,10);
  lcd.write('.');
  if (y < 100000)
    lcd.write('0');
  if (y < 10000)
    lcd.write('0');
  lcd.print(y / FREQ_DIVISION, 3); /* three Digit precision */
  SetClk();                        /* Generate Clock */
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

/***************************************************/
/* Initialize Clock Generator & Set 455KHz to CLK1 */
/***************************************************/
void Si5351Init (void) {
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); 
  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_2MA);      /* you can set this to 2MA, 4MA, 6MA or 8MA */
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_2MA);      /* you can set this to 2MA, 4MA, 6MA or 8MA */
  si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA);        /* Need to be done once */
  si5351.set_correction(SI5351_CALIB, SI5351_PLL_INPUT_XO);    /* Calibrate each Board */
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(CLK1_FREQ * FREQ_MUL, SI5351_CLK1);
}
void EncoderInit (void) {
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);                    /* Enable pin change interrupt for the encoder */
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
}
void SwitchInit (void) {
    pinMode(LSB_USB_SW, INPUT_PULLUP);
   *digitalPinToPCMSK(LSB_USB_SW) |= bit (digitalPinToPCMSKbit(LSB_USB_SW));  // enable pin
    
   // PCMSK1 |= _BV(LSB_USB_SW);
    PCIFR  |= bit (digitalPinToPCICRbit(LSB_USB_SW)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(LSB_USB_SW)); // enable interrupt for the group
}
void RelayInit (void) {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}
/**********************************************/
/* Set Clock for Both Receiver & Transmitter */
/*********************************************/
void SetClk() {
  volatile uint32_t bfo = Channels[SSBMode].RxFreq;
  volatile uint32_t vfo;

  if (Mode >= 1 && Mode <= 10)
    vfo = Channels[ChanNo]. RxFreq;
  else
    vfo = Channels[FREQ_MODE].RxFreq;  

  if (vfo < _10MHZ) {
    si5351.set_freq((bfo - vfo) * FREQ_MUL, SI5351_CLK0);
 //   SSBMode = LSB_MODE;
 //   DispBand();
    RelayOff();
  } else {
    si5351.set_freq((vfo - bfo) * FREQ_MUL, SI5351_CLK0);
 //   SSBMode = USB_MODE; 
 //   DispBand();
    RelayOn();
  }
  si5351.set_freq(bfo * FREQ_MUL, SI5351_CLK2);
}
/**************************************/
/* Read Frequency, ModeOP, Channel No */
/* If EEPROM is blank, Write Defaults */
/**************************************/
void ReadEEPROM(void){
  uint8_t i;
  if (EEPROM.read(MAGIC_ADD) != MAGIC_NO) { /* New EEPROM & Default Values*/
      EEPROM.write(MAGIC_ADD, MAGIC_NO);
      for (i = 0; i < STORE_CH; i++) {
        RxDispFreq = Channels[i].RxFreq;
        WriteChannel(i);
     }
      StepSize = DEF_STEP_SIZE;
      WriteStepSize();
      Mode = FREQ_MODE;
      WriteMode();
      SSBMode = LSB_MODE;
      StoreSSBMode();
      RxDispFreq = Channels[0].RxFreq;
      Mode = FREQ_MODE;
      ChanNo = 1;
      
   } else {                                       /* Read Entire Array */
      for (i = 0; i < STORE_CH; i++) {
        ReadChannel(i);
        Channels[i].RxFreq = RxDispFreq;
      }
      Mode = ReadMode();                               /* Restore power off data */
      SSBMode = EEPROM.read(SSBMODE_ADDR);
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
