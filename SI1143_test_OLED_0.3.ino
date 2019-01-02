
#include <Wire.h>
#include <si1143.h>
#include "U8g2lib.h"
#define LED_G PB1
#define LED_R PB10
#define LED_B PB11
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,PB12);

#define avgsize 20
#define pulsetrestohigh 1500
#define pulsetreshigh .6
#define pulsetreslow .3
#define sleep  10

int PS1,PS2,PS3,rawHR,result,pulsemax;
int graphArray[128],minmaxArray[avgsize];
int maxgraph,mingraph;
byte LowB,HighB;
byte linePos = 0;
bool pulsehigh,pulselow;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
byte minmaxSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
int HRrange;

void setup() {
  Serial.begin(115200);
  pinMode(LED_G, OUTPUT); digitalWrite(LED_G, LOW);
  pinMode(LED_R, OUTPUT); digitalWrite(LED_R, LOW);
  pinMode(LED_B, OUTPUT); digitalWrite(LED_B, LOW);
  
  Wire.begin(); // join i2c bus (address optional for master)
  delay(25);
  write_reg(HW_KEY, 0x17); // Setting up LED Power to full
  write_reg(PS_LED21,0xFF);
  write_reg(PS_LED3, 0x0F);
  param_set(CHLIST,0b00000001);
  param_set(PSLED12_SELECT,0b00000011);
  param_set(PS_ADC_GAIN,0b00000000);
  param_set(PS_ADC_MISC,0b00000100);
  char parameter = read_reg(PARAM_RD,1);
  delay(1000);
  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setContrast(255);
//Clear Screen
 u8g2.firstPage();
 do {} while (u8g2.nextPage());
display();
}

void loop() {
  readHR();
  calcHR();
  display();
}

void readHR(){
  write_reg(COMMAND,0b00000101); // Get a reading
  delay(5);
  LowB = read_reg(PS1_DATA0,1); // Read the data for the first LED
  HighB = read_reg(PS1_DATA1,1);
  PS1 = ((HighB * 255) + LowB);       // - bias1;
//  LowB = read_reg(PS2_DATA0,1);  // Read the data for the second LED
//  HighB = read_reg(PS2_DATA1,1);
//  PS2 = ((HighB * 255) + LowB); 
  rawHR=(PS1);
}

void calcHR(){ 
  digitalWrite(LED_R, LOW); 
      minmaxArray[minmaxSpot++] = rawHR; //Store this reading in the array
      minmaxSpot %= avgsize; //Wrap variable
  maxgraph = 1;  
  mingraph = 100000;
 for(int ArrayPos=1; ArrayPos<(avgsize-1); ArrayPos++) { if (maxgraph<minmaxArray[ArrayPos]) { maxgraph = minmaxArray[ArrayPos]; } if ((mingraph>minmaxArray[ArrayPos]) && (minmaxArray[ArrayPos]>100)) { mingraph = minmaxArray[ArrayPos];} }
HRrange=(maxgraph-mingraph);
result=(PS1-(maxgraph-HRrange));

if (pulsemax<result){pulsemax=result;} if ((pulsemax>(HRrange*pulsetreshigh))&&(HRrange>100)){pulsehigh =1;} if (pulsemax>pulsetrestohigh){pulsehigh=0;} if (result<(HRrange*pulsetreslow)){pulsemax=0;}
if((pulselow==1)&&(result>((HRrange*pulsetreshigh)))){pulselow=0;}
if ((pulsehigh==1)&&(result<(HRrange*pulsetreslow)&&(pulselow==0))){pulselow=1; pulsehigh=0; pulsemax=0; 
  digitalWrite(LED_R, HIGH);
  
    long deltaHR = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (deltaHR / 1000.0);
    //Serial.println(beatsPerMinute);

    if (beatsPerMinute < 180 && beatsPerMinute > 50) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++){
        beatAvg += rates[x]; }
      beatAvg /= RATE_SIZE;
  Serial.println(beatAvg);
    }
  }
}


void display(){

 
 Serial.print(result);
 Serial.print(" : ");
 Serial.println(HRrange);
// Serial.print(" : ");
// Serial.println(mingraph);

//  digitalWrite(LED_G, HIGH);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_courB24_tf);     //u8g2_font_logisoso62_tn
    u8g2.setCursor(0,24);
    u8g2.print(beatAvg);
    u8g2.setFont(u8g2_font_t0_17b_tf);
    u8g2.print("Bpm");
    u8g2.setFont(u8g2_font_6x10_tf);    //u8g2_font_torussansbold8_8u
    u8g2.setCursor(80,8);
    u8g2.print("SNR:");
    u8g2.print(HRrange);

  graphArray[127] = result;   
 for(int i=1; i<128; i++) { graphArray[i-1] = graphArray[i]; }
  for(int linePos=1; linePos<127; linePos++) {
    int graphhight1=(63 - (graphArray[(linePos-1)]/20));
    int graphhight=(63 - (graphArray[linePos]/20));
    u8g2.drawLine((linePos-1),(constrain(graphhight1,30,60)), (linePos),(constrain(graphhight,30,60)) ); }

    
    int graphtreshight=(63 - ((HRrange*pulsetreshigh)/20));
    int graphtreslow=(63 - ((HRrange*pulsetreslow)/20));
    u8g2.drawLine(1,(constrain(graphtreshight,30,60)), 128,(constrain(graphtreshight,30,60)) );
    u8g2.drawLine(1,(constrain(graphtreslow,30,60)), 128,(constrain(graphtreslow,30,60)) );
    }while (u8g2.nextPage());
//    digitalWrite(LED_G, LOW); 
  
}


void param_set(byte address, byte val)  // Set Parameter
{
  write_reg(PARAM_WR, val);
  write_reg(COMMAND, 0xA0|address);
}

char read_reg(unsigned char address, int num_data) // Read a Register
{

  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(IR_ADDRESS, num_data);
 
  while(Wire.available() < num_data);
 
  return Wire.read();
}

void write_reg(byte address, byte val) {  // Write a resigter
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(address);     
  Wire.write(val);       
  Wire.endTransmission();     
}
