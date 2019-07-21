#include <Wire.h>       //accelerometer
#include <SD.h> 

const int chipSelect = 10;  //accelerometer
float sine_angle_z;
float angle_z_degrees;

int sensorPin = 0;    //analog pin (check it), for the temperature sensor

int vib_pin = 7; // to be checked, vibration pin

#define ACC (0xA7>>1)         //for acclerometer
#define A_TO_READ (6) 

void initAcc() {
  writeTo(ACC, 0x2D, 1<<3);
  writeTo(ACC, 0x31, 0x0B);
  writeTo(ACC, 0x2C, 0x09);
}
void getAccelerometerData(int * result) {
  int regAddress = 0x32; 
  byte buff[A_TO_READ];
  readFrom(ACC, regAddress, A_TO_READ, buff); 
  
  result[0] = (((int)buff[1]) << 8) | buff[0];
  result[1] = (((int)buff[3])<< 8) | buff[2];   
  result[2] = (((int)buff[5]) << 8) | buff[4]; 
}

void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); //end transmission
}

void readFrom(int DEVICE, byte address, int num, byte buff[]){
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.write(address); //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num); // request 6 bytes from ACC
  int i = 0;
  while(Wire.available()){ //ACC may send less than requested (abnormal)
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); 
}

void setup(){
  Serial.begin(9600);

  pinMode(vib_pin,INPUT);
  
  Wire.begin();
  initAcc();

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("card failed, or not present.");
    return;
  }
  Serial.println("card initialized.");
}

void loop(){

  int val = digitalRead(vib_pin);           //vibration sensor
  
  int reading = analogRead(sensorPin);  //temperature sensor
  float voltage = reading * 5.0;
  voltage /= 1024.0;

  Serial.print(voltage); Serial.println(" volts");

  float temperatureC = (voltage - 0.5) * 100;
  Serial.print(temperatureC); Serial.println( "degrees C");
  
  int x,y,z;                  //accelerometer
  int acc[3];

  getAccelerometerData(acc);
  x = acc[0]; //not sure
  y = acc[1]; //not sure
  z = acc[2];

  Serial.print("x = ");
  Serial.print(x);
  Serial.print(" ");

  Serial.print("y = ");
  Serial.print(y);
  Serial.print(" ");
  
  Serial.print("z = ");
  Serial.print(z);
  Serial.print(" ");

  sine_angle_z = ((z + 10.00 )/240.00);

  Serial.print(" sine_angle_z = ");
  Serial.print(sine_angle_z);
  Serial.print(" ");

  delay(1000);
  
  if (sine_angle_z >1) { 
    sine_angle_z = 1; 
 }
  else if (sine_angle_z <-1){ 
    sine_angle_z = -1; 
 }

  angle_z_degrees = asin(sine_angle_z)* RAD_TO_DEG;
  Serial.print(" angle_z_degrees = ");
  Serial.println(angle_z_degrees);
  delay(250);
  
  File dataFile = SD.open("accandangle.csv", FILE_WRITE); 
  if (dataFile) { 
    dataFile.println(x);
    dataFile.println(y);
    dataFile.println(z);
    dataFile.println(angle_z_degrees);
    dataFile.println(temperatureC);
    dataFile.println(val); 
    dataFile.close(); 
 }
  else {
  Serial.println("Error opening datalog.csv"); 
 }
}
