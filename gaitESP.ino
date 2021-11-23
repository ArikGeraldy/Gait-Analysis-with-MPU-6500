//setrate pengambilan data perdetik
int setrate = 500; // maksimum 500
int a=(1000/setrate)-1;
//int a;
int zz=10;
const float d2r = 3.14159265359f/180.0f;
//lib
#include <Wire.h> // lib i2c esp32 on 
#include <MPU9250_asukiaaa.h> //lib mpu9250;

#ifdef _ESP32_HAL_I2C_H_ //define i2c pin
#define SDA_PIN 21
#define SCL_PIN 22
#endif

//SD CARD
#include "FS.h"                                           
#include "SD.h"
#include "SPI.h"
String dataMessage;
String dataMessage1;
//unsigned long int milli_time;

#define ACC_FULL_SCALE_8_G;
#define GYRO_FULL_SCALE_2000_DPS;
#define maxSensors 8 // array untuk multi sensor
MPU9250_asukiaaa mpu[maxSensors];
String id[maxSensors] = {"imu1", "imu2", "imu3", "imu4", "imu5", "imu6", "imu7", "imu8"};

//define variable
uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float aax,aay,aaz,ggx,ggy,ggz;
float axx,ayx,azx,gxx,gyx,gzx;
float xax,xay,xaz,xgx,xgy,xgz;
int paket;
int b;


float dat,sum,rate,del;
unsigned long int milli;
unsigned long int milli_time;

// opsional raw picth yaw 
//float pitch, roll, yaw;
//float aa,ab,ac,ga,gb,gc,oaa,oab,oac,oga,ogb,ogc;

//LED
const int ledPin1 =  12; 
const int ledPin2 =  13; 
const int ledPin3 =  14;
int ledState1 = LOW;

//TIMER LED
unsigned long previousMillis = 0;
const long interval = 500;   

//PUSH BUTTON
const int buttonPin1 = 25;  
const int buttonPin2 = 27;  

int buttonState1 = 0;  
int buttonState2 = 0;  

//BUZZER
int freq = 2000;
int channel = 0;
int resolution = 8;


   // SD CARD
// SD CARD BACA
void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

//BUKA FILE
void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        //Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
       // Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

//SD CARD log file
void logSDCard() {
    
  dataMessage1 =  /*"," +String(paket) +*/ ","+String(milli_time)+"," + String(gX) + "," + 
                String(gY) + "," + String(gZ)+ "," + String(aX) + "," + 
                String(aY) + "," + String(aZ)+ "," + String(mX) + "," + 
                String(mY) + "," + String(mZ);
 // Serial.print("Save data ");
 // Serial.println(dataMessage);
  appendFile(SD, "/data3.txt", dataMessage1.c_str());
 // dataMessage = " ";
}

void jne(){
  dataMessage =  String(paket) ;

  appendFile(SD, "/data3.txt", dataMessage.c_str());
  //dataMessage = " ";
}

void enter(){     
  dataMessage =  String(paket)+ ","  + String(gX) + "," + 
                String(gY) + "," + String(gZ)+ "," + String(aX) + "," + 
                String(aY) + "," + String(aZ)+ "," + String(mX) + "," + 
                String(mY) + "," + String(mZ) + "\r\n";
 // Serial.print("Save data ");
 // Serial.println(dataMessage);
  appendFile(SD, "/data3.txt", dataMessage.c_str()); 
//  dataMessage = " ";
}

//SD CARD HAPUS
void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
      //  Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}
   
void setup() {
  // SERIAL KOMUNIKASI, MATIKAN BILA TIDAK SEDANG DEBUG
  Serial.begin(115200);
    SPI.begin();
    SPI.setFrequency(40000000);
  //SET PIN LED DAN PUSH BUTTON
  pinMode(2, OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(32, channel);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);

  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);


//MULAI I2C ESP32
  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
      Wire.setClock(400000); //SET I2C 400KHZ
  //  mpu.setWire(&Wire);
  #endif


  for (int i = 2; i < 3; i++) {
digitalWrite(ledPin1, HIGH);
 
  mpu[i].beginAccel();
  mpu[i].beginGyro();
  digitalWrite(ledPin1, LOW);
   
  digitalWrite(ledPin1, HIGH);
  
  digitalWrite(ledPin1, LOW);
  mpu[i].beginMag();
//MPU KALIBRASI

  }
  for (int i = 0; i < zz; i++) {
 mpu[2].accelUpdate();
    aX = (float)mpu[2].accelX();
    aY = (float)mpu[2].accelY();
    aZ = (float)mpu[2].accelZ();
    mpu[2].gyroUpdate();
    gX = (float)mpu[2].gyroX();
    gY = (float)mpu[2].gyroY();
    gZ = (float)mpu[2].gyroZ();
    aax+=aX;
    aay+=aY;
    aaz+=aZ;
    ggx+=gX;
    ggy+=gX;
    ggz+=gZ;
    Serial.println(i);
    }
axx=aax/zz;
ayx=aay/zz;
azx=1-(aaz/zz);
gxx=ggx/zz;
gyx=ggy/zz;
gzx=ggz/zz;
  
// SD CARD, MOUNT & BUAT FILE
      if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
        exit(1);
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    File file = SD.open("/data3.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data3.txt", "Packet number,Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g),Magnetometer X (G),Magnetometer Y (G),Magnetometer Z (G) \r\n");
  }
  else {
   // Serial.println("File already exists");  
   deleteFile(SD, "/data3.txt");
   writeFile(SD, "/data3.txt", "Packet number,Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g),Magnetometer X (G),Magnetometer Y (G),Magnetometer Z (G) \r\n");
  }
  file.close();

Serial.println("Mulai Ambil Data");

digitalWrite(ledPin2, HIGH);
digitalWrite(ledPin3, LOW);

//buzzer
  ledcWriteTone(channel, 2000);
  int dutyCycle = 255 ;
  ledcWrite(channel, dutyCycle);
  delay(500);
  digitalWrite(ledPin2, LOW);
  ledcWriteTone(channel, 0);
delay(500);
  ledcWriteTone(channel, 2000);
digitalWrite(ledPin2, HIGH);
  ledcWrite(channel, dutyCycle);
  delay(750);
  digitalWrite(ledPin2, LOW);
  ledcWriteTone(channel, 0);
delay(250);
  ledcWriteTone(channel, 2000);
digitalWrite(ledPin2, HIGH);
  ledcWrite(channel, dutyCycle);
  delay(1000);
  digitalWrite(ledPin2, LOW);
  ledcWriteTone(channel, 0);
  delay(1);
  digitalWrite(ledPin2, HIGH);

 


}


void loop() {
     Wire.begin();
     Wire.setClock(400000);
   //  SPI.begin();
   // SPI.setFrequency(40000000);
     
digitalWrite(ledPin3, LOW);      
digitalWrite(ledPin1, HIGH);
digitalWrite(ledPin2, HIGH);
//jalan();

// a=1;//setpoint pengambilan data (ms)
 static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) == a)
    {
  // put your main code here, to run repeatedly:
   //  for (int i = 2; i < 8; i++) {
     
  // milli_time = millis();
    //no paket
 //   jne();
//Serial.print(paket);
//Serial.print(","); 
 // Serial.print("sensor ke: ");
 // Serial.print(i-1);
 
    mpu[2].accelUpdate();
    aX = (float)mpu[2].accelX();//-axx;
    aY = (float)mpu[2].accelY();//-ayx;
    aZ = (float)mpu[2].accelZ();//+azx;
    mpu[2].gyroUpdate();
    xgx = (float)mpu[2].gyroX();//+0.01625f;
    xgy = (float)mpu[2].gyroY()-0.03;
    xgz = (float)mpu[2].gyroZ();//+-0.000357143f;
    mpu[2].magUpdate();
    mX = (float)mpu[2].magX()-mpu[2].magXOffset;
    mY = (float)mpu[2].magY()-mpu[2].magYOffset;
    mZ = (float)mpu[2].magZ()-mpu[2].magZOffset;
gX=(xgx-gxx);//*d2r;
gY=(xgy-gyx);//*d2r;
gZ=(xgz-gzx);//*d2r;
   enter();
/*
       // yaw from mag
       pitch = atan2 (aY ,( sqrt ((aX * aX) + (aZ * aZ))));
   roll = atan2(-aX ,( sqrt((aY * aY) + (aZ * aZ))));
   float Yh = (mY * cos(roll)) - (mZ * sin(roll));
   float Xh = (mX * cos(pitch))+(mY * sin(roll)*sin(pitch)) + (mZ * cos(roll) * sin(pitch));
   yaw =  atan2(Yh, Xh);
    roll = roll*57.3;
    pitch = pitch*57.3;
    yaw = yaw*57.3;
*/
    
del =millis() - prev_ms;  
         
       sum += del;
       dat=dat+1;
       rate=(dat/sum)*1000;

      //   debug();
    //   Serial.print("paket : ");
    //   Serial.println(paket);
    //   Serial.print("setpoint : ");
    //   Serial.println(a);
    //   Serial.print("rate(Hz) : ");
    //  Serial.println(rate);
    //   Serial.print("delay total  : ");
     //  Serial.println(del);
   //    Serial.println();
       

      /*
       Serial.print("rpy : ");
       Serial.print(roll);
       Serial.print(" ,  ");
       Serial.print(pitch);
       Serial.print(" ,  ");
       Serial.println(yaw);
       */

            //  b=2;
if (setrate==rate){
    digitalWrite(2, HIGH);
      ledcWriteTone(channel, 2000);
  int dutyCycle = 255 ;
  ledcWrite(channel, dutyCycle);
  delay(250);
  digitalWrite(ledPin2, LOW);
  ledcWriteTone(channel, 0);
delay(250);
  ledcWriteTone(channel, 2000);
digitalWrite(ledPin2, HIGH);
}
if (setrate!=rate){
    digitalWrite(2, LOW);
}
     paket++;
       prev_ms = millis();  
    }
    sum=0;
    dat=0;
   // paket++;
   // delay(1);
  }
  


void debug(){
/*
//Serial.print(",");
Serial.print(gX);
Serial.print(",");
Serial.print(gY);
Serial.print(",");
Serial.print(gZ);
Serial.print("/t/t");
*/
Serial.print(aX);
Serial.print(",");
Serial.print(aY);
Serial.print(",");
Serial.println(aZ);
/*
Serial.print(gxx);
Serial.print(",");
Serial.print(gyx);
Serial.print(",");
Serial.println(gzx);
Serial.print(",");
Serial.print(xgx);
Serial.print(",");
Serial.print(xgy);
Serial.print(",");
Serial.println(xgz);
Serial.print(",");

Serial.print(gX);
Serial.print(",");
Serial.print(xgx);
Serial.print(",");
Serial.print(gxx);
Serial.print(",   ");
Serial.print(gY);
Serial.print(",");
Serial.print(xgy);
Serial.print(",");
Serial.print(gyx);
Serial.print(",   ");
Serial.print(gY);
Serial.print(",");
Serial.print(xgz);
Serial.print(",");
Serial.println(gzx);
//enter
//Serial.println();
//*/
}
