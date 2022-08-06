#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "math.h" 
#include "EasyNextionLibrary.h" 


/* ======= MC PIN assignment ======== */
#define ROTARY_POS A7 
#define PWR_FWD A1
#define PWR_REF A2 
#define ID A0 
#define LED_PO 18 // A4
#define LED_I 19 // A5
#define VCC A6
#define LED_SWR 3
#define BIAS_OFF 13 // connected to pin 5 of LM2596 bias regulator
#define PTT 17 //A3
#define band1 11
#define band2 12
#define band3 8
#define band4 7
#define band5 6
#define band6 4
#define band7 5
#define antenna2 10 //Antenna 2 relay pin of arduino


/* ======= Nextion Display ======== */
EasyNex myNex(Serial);

/* ======= Tempareture ======== */
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
int  resolution = 9;
unsigned long lastTempRequest = 0;
int  delayInMillis = 0;
float temperature = 0.0;

/* ======= PWM FAN ======== */
const byte OC1A_PIN = 9;
//const byte OC1B_PIN = 10;
const word PWM_FREQ_HZ = 31000; //Set currently to 31kHZ for Dalas 12V 4 wire Fan
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);



/* ======== Other variable ======== */


bool touch_status = false;
bool PTT_status = false;
bool error_i_status = false;
bool error_swr_status = false;
bool error_po_status = false;
bool error_temp_status = false;
bool antenna2_status = false;

int graph_maxWatt = 600; //Scale from 0 to 600, set as per your SSPA
int graph_maxTemp = 55; //Scale from 0 to 55
int graph_maxSwr = 2; //
int current_band = 0;
int band_pos =0;
int temp_heatsink =0;
int VdelayInMillis = 1000; // refresh V display every N mili sec


unsigned long lastVRequest =0;

int IdelayInMillis = 100; // refresh I display every N mili sec
unsigned long lastIRequest =0;




float Res_1=150000.00; //Set R1 of voltage devider
float Res_2=10000.00 ; //Set R2 of voltage devider
float Vin=0.00;
float Vout=0.00;

int graph_Watt = 0;
int graph_Temp = 0;
int graph_Swr = 0;

/* ======== power and swr related variable ======== */

// Calibration factors

int calibrP = 58;                // Assume 3.3v = 1000w in 50R. CalibrP = (3.3 / 5.0 x 1024 + Vdiode)x(3.3 / 5.0 x 1024 + Vdiode) / 1000w
int Vdiode = 60;                  // if we assume FWD voltage diode = 0,3v : Vdiode = 0,3 v / 5.0 v  x 1024
int T_pepHOLD = 600;          // msec pep hold time before return
long lastTpep = 0;                                   // update PEP display


unsigned long   power_fwd = 0;              // power forward (watts)
unsigned long   power_ref = 0;              // power reflected (watts)
unsigned int   power_fwd_max = 0;           // power forward max (for peak hold)
unsigned int   power_ref_max = 0;           // power reflected max (for peak hold)

float SWR = 0;                            // SWR 
float power_ratio = 0;                         // Power ratio P forward / P refl
int swr_display = 0;                           // swr x 10 showing in display


/* === PWM Function === */
void setPwmDuty(byte duty){
  OCR1A = (word) (duty*TCNT1_TOP)/100;
}

/* === Volt meter Function === */
void read_volt(){
  float vcc;
  int v1_val=0;
  int V = 0;
  v1_val = analogRead(VCC);
  Vout = (v1_val * 5.00) / 1024.00; 
  Vin = Vout / (Res_2/(Res_1+Res_2)); 
  V = ((Vin * 10)+ 0.5);

  if (millis() - lastVRequest >= VdelayInMillis)
  {
  myNex.writeNum("Cv.val", V);
  lastVRequest = millis(); 
  }
}

/* === SWR/Po calculation Function === */
void read_power(){
  power_fwd = analogRead(PWR_FWD);
  power_ref = analogRead(PWR_REF);
 
  if (power_fwd > 5){         // only correct for diode voltage when more than zero
    power_fwd = (power_fwd + Vdiode)*(power_fwd + Vdiode) / calibrP; 
    }
  
  if (power_ref > 5){             // only correct for diode voltage when more than zero
    power_ref = (power_ref + Vdiode)*(power_ref + Vdiode) / calibrP;
    }   

   // detect SWR error / load mismatch 
  power_ratio = power_fwd / power_ref;           // calculate ratio with raw data
  SWR = abs ((1+sqrt(power_ratio)) / (1-sqrt(power_ratio))) ; 
  
  // hold peak
  
  if (power_fwd >= power_fwd_max){
    lastTpep = millis();
    power_fwd_max = power_fwd;
    }
         
  if(millis() > (lastTpep + T_pepHOLD)){ // clear the peak after hold time
    power_fwd_max = power_fwd; 
    }


  swr_display = (SWR * 10) + 0.5;   // Float x 10 and convert to int with rounding
 
  if (swr_display < 10){    // SWR cannot be lower than 1.0
    swr_display = 10 ;
    }

//power_fwd_max is power output
// swr_display: swr value x 10
}

/* === SWR/Po display Function === */
void display_power(){
 // power_fwd_max = 432; //Test value *********************************
  
  myNex.writeNum("Cw.val", power_fwd_max);
        
  float graph_limit_watt = (graph_maxWatt/100.00);
  graph_Watt = (power_fwd_max / graph_limit_watt);
 // graph_Watt = 80; //Test value ***************************
 // swr_display = 15; //Test value ***************************
  myNex.writeNum("Gw.val", graph_Watt);
  myNex.writeNum("Cs.val", swr_display);
 
  float graph_limit_swr = ((graph_maxSwr-1)/100.00);
  graph_Swr = ((swr_display/10)-1) / (graph_limit_swr); // calculate from 1 to max swr.
  //graph_Swr = 15; //Test value *******************************
  myNex.writeNum("Gs.val", graph_Swr);
   
}

/* =========== Send band info to LCD ====================== */

void display_band(int Nband){
 
  myNex.writeNum("Cb.val", Nband);
  switch (Nband) {
      case 1:
        myNex.writeNum("b1.val", 1);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
      break;
      case 2:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 1);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
      break;
      case 3:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 1);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
      break;
      case 4:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 1);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
      break;
      case 5:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 1);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 0);
      break;
      case 6:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 1);
        myNex.writeNum("b7.val", 0);
      break;
      case 7:
        myNex.writeNum("b1.val", 0);
        myNex.writeNum("b2.val", 0);
        myNex.writeNum("b3.val", 0);
        myNex.writeNum("b4.val", 0);
        myNex.writeNum("b5.val", 0);
        myNex.writeNum("b6.val", 0);
        myNex.writeNum("b7.val", 1);
      break;
    }
}



/* =========== LPF Relay control ====================== */

void lpf_relay(int Nrelay){
  EEPROM.put(0, Nrelay); //Update/write band info to Memory
  switch (Nrelay){
    case 0:
      digitalWrite(band1, LOW);
      digitalWrite(band2, LOW);
      digitalWrite(band3, LOW);
      digitalWrite(band4, LOW);
      digitalWrite(band5, LOW);
      digitalWrite(band6, LOW);
      digitalWrite(band7, LOW);
    break;
    case 1:
      digitalWrite(band1, HIGH);
      digitalWrite(band2, LOW);
      digitalWrite(band3, LOW);
      digitalWrite(band4, LOW);
      digitalWrite(band5, LOW);
      digitalWrite(band6, LOW);
      digitalWrite(band7, LOW);
      break;
    case 2:
      digitalWrite(band1, LOW);
      digitalWrite(band2, HIGH);
      digitalWrite(band3, LOW);
      digitalWrite(band4, LOW);
      digitalWrite(band5, LOW);
      digitalWrite(band6, LOW);
      digitalWrite(band7, LOW);
      break;
    case 3:
      digitalWrite(band1, LOW);
      digitalWrite(band2, LOW);
      digitalWrite(band3, HIGH);
      digitalWrite(band4, LOW);
      digitalWrite(band5, LOW);
      digitalWrite(band6, LOW);
      digitalWrite(band7, LOW);
      break;
    case 4:
      digitalWrite(band1, LOW);
      digitalWrite(band2, LOW);
      digitalWrite(band3, LOW);
      digitalWrite(band4, HIGH);
      digitalWrite(band5, LOW);
      digitalWrite(band6, LOW);
      digitalWrite(band7, LOW);
      break;
    case 5:
      digitalWrite(band1, LOW);
      digitalWrite(band2, LOW);
      digitalWrite(band3, LOW);
      digitalWrite(band4, LOW);
      digitalWrite(band5, HIGH);
      digitalWrite(band6, LOW);
      digitalWrite(band7, LOW);
      break;
    case 6:
      digitalWrite(band1, LOW);
      digitalWrite(band2, LOW);
      digitalWrite(band3, LOW);
      digitalWrite(band4, LOW);
      digitalWrite(band5, LOW);
      digitalWrite(band6, HIGH);
      digitalWrite(band7, LOW);
      break;
    case 7:
      digitalWrite(band1, LOW);
      digitalWrite(band2, LOW);
      digitalWrite(band3, LOW);
      digitalWrite(band4, LOW);
      digitalWrite(band5, LOW);
      digitalWrite(band6, LOW);
      digitalWrite(band7, HIGH);
    break;
  } 
  
  current_band=Nrelay;
  display_band(Nrelay);
}


/* === Monitor and show PTT status on display === */
void onair(){
  int PTT_en = digitalRead(PTT);
 
    if (PTT_en==0 && PTT_status != true){ 
      //hmiSend("e1.val=",1);
      myNex.writeNum("e1.val", 1);
      PTT_status = HIGH;
      LCDband_disable();
      }
    else if(PTT_en==1 && PTT_status!= false){
  
      myNex.writeNum("e1.val", 0);
      PTT_status = LOW;
      LCDband_enable();
    }
}

/* === Monitor and show over current error on display === */
void error_i(){
  int error_i_en = digitalRead(LED_I);
  //error_i_en = 0; //test 
  if (error_i_en == LOW && error_i_status == false){
   
    myNex.writeNum("e2.val", 1);
    error_i_status = true;
    }
     else if (error_i_en == HIGH && error_i_status == true){
   
      myNex.writeNum("e2.val", 0);
      error_i_status = false;
     }
}

/* === Monitor and show over SWR error on display === */
void error_swr(){
  int error_swr_en = digitalRead(LED_SWR);
  //error_swr_en = 0; //test 
  if (error_swr_en==LOW && error_swr_status == false){
    //hmiSend("e3.val=",1);
    myNex.writeNum("e3.val", 1);
    error_swr_status = true;
    }
  else if(error_swr_en==HIGH && error_swr_status == true){
    //hmiSend("e3.val=",0);
    myNex.writeNum("e3.val", 0);
    error_swr_status = false;
  }
}

/* === Monitor and show over Power error on display === */
void error_po(){
  int error_po_en = digitalRead(LED_PO);
 // error_po_en = 0; //test ****************************************
  if (error_po_en==LOW && error_po_status == false){
   
    myNex.writeNum("e5.val", 1);
    error_po_status = true;
   }
      else if (error_po_en==HIGH  && error_po_status == true){
      //hmiSend("e5.val=",0);
      myNex.writeNum("e5.val", 0);
      error_po_status = false;
      }
}

/* === Over temp protection and show temp error=== */
void error_temp(bool highTemp){
    if (error_temp_status == false && highTemp){ 
        digitalWrite(BIAS_OFF, HIGH); // LM2596HVS PIN5
        myNex.writeNum("e4.val", 1);
        error_temp_status = true;
        }
    
    if (error_temp_status == true && !highTemp){ 
        digitalWrite(BIAS_OFF, LOW);
        myNex.writeNum("e4.val", 0);
        error_temp_status = false;
        }
       
    }

/* === Monitor current from protection board  === */
void read_ID(){
  int I;
  float Is;
  float Idv;
  float ref_vdd = (5.00/1024);
  
  Idv=( analogRead(ID) * ref_vdd );
  Is = (Idv*13000);
  Is = (Is/4482);
  I = ((Is * 10)+ 0.5);
//  I = 123; //test value
  

 if (millis() - lastIRequest >= IdelayInMillis)
  {
  myNex.writeNum("Ca.val", I);
  lastIRequest = millis(); 
  }
  
}

/* === Set FAN speed according to temp === */
void fanspeed(){
  if(temperature>20.0 && temperature<40.0){
    setPwmDuty(75);
   myNex.writeStr("Fs.txt", "LOW");
  }
  else if(temperature>=40.0 && temperature<45.0){
    setPwmDuty(50);
    myNex.writeStr("Fs.txt", "MID");
  }
  else if(temperature>=45.0 && temperature<50.0){
    setPwmDuty(25);
    myNex.writeStr("Fs.txt", "HIGH");
  }
  else if(temperature>=50.0 && temperature<55.0 ){
    setPwmDuty(0);
    myNex.writeStr("Fs.txt", "*HOT");
  }
    
}



/* === Tempareturn Monitor  === */
void read_temp(){
  if (millis() - lastTempRequest >= delayInMillis)
  {
  temperature = sensors.getTempCByIndex(0);
  sensors.requestTemperatures(); 
  lastTempRequest = millis(); 
  }
  if(temperature>=(graph_maxTemp-1)){
  
   error_temp(1);
   setPwmDuty(0);
   myNex.writeStr("Fs.txt", "ALRM");
  }
  if(temperature<=(graph_maxTemp-5) && error_temp_status){
     error_temp(0);
   
  }
  
  myNex.writeNum("Ct.val", temperature);
  float graph_limit = (graph_maxTemp/100.00);
  graph_Temp = (temperature / graph_limit);
  myNex.writeNum("Gt.val", graph_Temp);
  fanspeed();
  }


/* === Enabling touch functions of Display === */
void LCDband_enable(){
  if(touch_status != true){
    myNex.writeNum("v1.val", 1);
    myNex.writeStr("tsw b1,1");
    myNex.writeStr("tsw b2,1");
    myNex.writeStr("tsw b3,1");
    myNex.writeStr("tsw b4,1");
    myNex.writeStr("tsw b5,1");
    myNex.writeStr("tsw b6,1");
    myNex.writeStr("tsw b7,1");
    touch_status = true;
    }
}

/* === Disabling touch functions of Display === */
void LCDband_disable(){
  if(touch_status != false){
    myNex.writeNum("v1.val", 0);
    myNex.writeStr("tsw b1,0");
    myNex.writeStr("tsw b2,0");
    myNex.writeStr("tsw b3,0");
    myNex.writeStr("tsw b4,0");
    myNex.writeStr("tsw b5,0");
    myNex.writeStr("tsw b6,0");
    myNex.writeStr("tsw b7,0");
    touch_status = false;
  }
}




/* === Rotary band switch === */

int band_switch(){
  int band_select;
  //read rotary value
  int band_value =  analogRead(ROTARY_POS);
    
  if (band_value <=100){
    myNex.writeStr("r1.val=0");
    LCDband_enable();
    band_select = current_band;
  } 
  else if (band_value>=101&& band_value <=200){
      myNex.writeStr("r1.val=1");
      LCDband_disable();
      band_select=1;
    }
    else if (band_value>=201 && band_value <=350){
      myNex.writeStr("r1.val=1");
      LCDband_disable();
      band_select=2;
    }
    else if (band_value>=351 && band_value <=500){
      myNex.writeStr("r1.val=1");
      LCDband_disable();
      band_select=3;
    }
    else if (band_value>=501 && band_value <=650){
      myNex.writeStr("r1.val=1");
      LCDband_disable();
      band_select=4;
    }
    else if (band_value>=651 && band_value <=800){
      myNex.writeStr("r1.val=1");
      LCDband_disable();
      band_select=5;
    }
    else if (band_value>=801 && band_value <=950){
      myNex.writeStr("r1.val=1");
      LCDband_disable();
      band_select=6;
    }
    else if (band_value>=951 && band_value <=1024){
      myNex.writeStr("r1.val=1");
      LCDband_disable();
      band_select=7;
    }
    else {}
  return band_select;
  }


void band_position(){
  band_pos = band_switch();
  if (current_band != band_pos && PTT_status != HIGH ){
    lpf_relay(band_pos);
   }
}

void band_position_touch(){
  if (current_band != band_pos && PTT_status != HIGH){
    lpf_relay(band_pos);
  }
}

/* === Trigger from Nextion Display  === */
void trigger1(){
  band_pos = 1;
  band_position_touch();
  }

void trigger2(){
  band_pos = 2;
  band_position_touch();
  }
  
void trigger3(){
  band_pos = 3;
  band_position_touch();
  }

void trigger4(){
  band_pos = 4;
  band_position_touch();
  }

void trigger5(){
  band_pos = 5;
  band_position_touch();
  }

void trigger6(){
  band_pos = 6;
  band_position_touch();
  }

void trigger7(){
  band_pos = 7;
   band_position_touch();
  }

//Antenna2 switch
void trigger8(){
   if (antenna2_status == LOW){
    digitalWrite(antenna2, HIGH);
    myNex.writeNum("a1.val", 1); 
    antenna2_status = HIGH;
    } 
   else {
    digitalWrite(antenna2, LOW);
    myNex.writeNum("a1.val", 0); 
    antenna2_status = LOW;
    }
   
  }

  
void setup() {
  //delay (1000); //wait for display initialization
  myNex.begin();  // start Nextion Display 
  myNex.writeStr("op.txt", "A12BC"); //Write your callsign here
 
  /* === FAN PWM setup === */
  pinMode(OC1A_PIN, OUTPUT); //fan pwm

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Set Timer1 configuration
  // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
  // COM1B(1:0) = 0b00   (Output B normal operation)
  // WGM(13:10) = 0b1010 (Phase correct PWM)
  // ICNC1      = 0b0    (Input capture noise canceler disabled)
  // ICES1      = 0b0    (Input capture edge select disabled)
  // CS(12:10)  = 0b001  (Input clock select = clock/1)
  
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
  
    
  /* ==== Temp sensor setup ==== */
  sensors.begin(); //Temp sensor
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - resolution)); 
  lastTempRequest = millis(); 

  
pinMode(band1, OUTPUT);
pinMode(band2, OUTPUT);
pinMode(band3, OUTPUT);
pinMode(band4, OUTPUT);
pinMode(band5, OUTPUT);
pinMode(band6, OUTPUT);
pinMode(band7, OUTPUT);
pinMode(BIAS_OFF, OUTPUT);

pinMode(PTT, INPUT);
pinMode(LED_PO, INPUT);
pinMode(LED_I, INPUT);
pinMode(LED_SWR, INPUT);
pinMode(VCC, INPUT);

EEPROM.get(0, current_band); // Load last band position to current_band variable from memory location 0
lpf_relay(current_band);
display_band(current_band);
}

void loop() {
  if (PTT_status != HIGH) myNex.NextionListen();
  band_position(); //LPF Band position
  onair(); //Check PTT status
  read_volt(); // Read V
  read_ID(); //Read I
  read_power(); //Read Po and SWR
  display_power(); // Display power
  read_temp(); // Read Temp
  error_i(); // Check Error: I
  error_swr(); // Check Error: SWR
  error_po(); // Check Error: Po
  }
