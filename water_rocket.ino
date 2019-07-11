#include <math.h>  //Library required for various mathematical functions
#include <SPI.h>  //SPI library to communicate with the sensors
#include <SD.h>  //SD library to communicate with the SD Card

#include <MS5xxx.h> //Library used for the altimeter

#include <Wire.h> //Library used for allowing I2C communication

#include <Adafruit_MMA8451.h>//Library used for the accelerometer 
#include <Adafruit_Sensor.h> //Library used for the accelerometer

#define cardSelect 4      //SD card chip select for feather SD Card
#define ADC_ref 2.56      //ADC reference voltage
#define zero_x 1.569      //0g voltage for x axis 
#define zero_y 1.569      //0g voltage for y axis
#define zero_z 1.569      //0g voltage for z axis
#define sensitivity_x 0.3 //Sensitivity of x for further calculations
#define sensitivity_y 0.3 //Sensitivity of y for further calculations
#define sensitivity_z 0.3 //Sensitivity of z for further calculations

float p_sealevel = 1013.25; //Sea-level pressure in hPa                   
int   vib_pin = A0;         //Pin used for the vibration sensor

File logfile;               //The file needed for the recording data

Adafruit_MMA8451 mma = Adafruit_MMA8451(); //Creating the Adafruit_MMA8451 object

MS5xxx sensor(&Wire);  //Creating the MS5xxx sensor object       

void setup() {
  Serial.begin(9600);

  //Checking if the SD Card is initialized
  if(!SD.begin(cardSelect)){              
    Serial.println("Card init. failed"); 
  }

  //Checking if the altimeter is connected correctly
  if(sensor.connect()>0){
    Serial.println("Error connecting to the altimeter");
    delay(500);
    setup();
    } 
  
  //Checking if the altimeter is connected correctly
  if(! mma.begin()){
    Serial.println("Error connecting to the acclerometer");
    while(1);
  }

  mma.setRange(MMA8451_RANGE_2_G); //Setting the range for the acclerometer
  
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
}

void loop() {

  //Measuring time in milliseconds
  unsigned long time = millis();
  Serial.print("Time: "); Serial.println(time);

  //Measuring vibration
  int vibSensor = analogRead(vib_pin);
  Serial.print("Vibration [Hz]: "); Serial.println(vibSensor);
  
  delay(100);

  //Reading from the accelerometer 
  mma.read();
 
  Serial.print("X:\t"); Serial.print(mma.x); 
  Serial.print("\tY:\t"); Serial.print(mma.y); 
  Serial.print("\tZ:\t"); Serial.print(mma.z); 
  Serial.println();

  //Getting a new sensor event 
  sensors_event_t event; 
  mma.getEvent(&event);

  //Measuring acceleration in ms^-2 for x,y and z axes
  Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");

  //Calculating acceleration in 1 dimension using pythagorean theorem
  float acc_1d = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
  Serial.println("Acceleration in 1D: "); Serial.print(acc_1d); Serial.println("m/s^2 ");
  
  //Getting the orientation of the sensor
  uint8_t o = mma.getOrientation();
  
  switch (o) {
    case MMA8451_PL_PUF: 
      Serial.println("Portrait Up Front");
      break;
    case MMA8451_PL_PUB: 
      Serial.println("Portrait Up Back");
      break;    
    case MMA8451_PL_PDF: 
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB: 
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF: 
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB: 
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF: 
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB: 
      Serial.println("Landscape Left Back");
      break;
    }
  Serial.println("---");
  delay(500); 

  float xv = (mma.x/1024.0*ADC_ref-zero_x)/sensitivity_x;
  float yv = (mma.y/1024.0*ADC_ref-zero_y)/sensitivity_y;
  float zv = (mma.z/1024.0*ADC_ref-zero_z)/sensitivity_z;

  //Measuring the angle for x
  float angle_x =atan2(-yv,-zv)*57.2957795+180;
  Serial.print("Rotation for x: ");
  Serial.print(angle_x);
  Serial.print(" deg");
  Serial.print(" ");

  //Measuring the angle for y
  float angle_y =atan2(-xv,-zv)*57.2957795+180;
  Serial.print("Rotation for y: ");
  Serial.print(angle_y);
  Serial.print(" deg");
  Serial.print(" "); 
  
  //Measuring the angle for z
  float angle_z =atan2(-yv,-xv)*57.2957795+180;
  Serial.print("Rotation for z: ");
  Serial.print(angle_z);
  Serial.print(" deg");
  Serial.print("\n");

  Serial.println("---");
  delay(500); 

  //Readings from the altimeter
  sensor.ReadProm();
  sensor.Readout();
  Serial.print("Temperature [C]: ");      //Measuring temperature in 0.01 Celcius
  Serial.println(sensor.GetTemp()*0.01);
  Serial.print("Pressure [Pa]: ");        //Measuring pressure
  Serial.println(sensor.GetPres());       
  
  //Calculating the altitude using hypsometric formula
  float altitude = ((-1.0 + pow((p_sealevel/(sensor.GetPres()/100)), (1/5.257)))*(((sensor.GetTemp()*0.01)+273.15)/0.0065)); //preassure in hPa
  Serial.print("Altitude [m]: "); Serial.println(altitude*0.3048); //Converting Ft to m, the calcultion above gives the altitude in Ft
  
  test_crc();
  Serial.println("---");
  delay(500);

  //Writing data to SD Card
  File logfile = SD.open("wr.csv", FILE_WRITE); //Opening a file to write data to
  if(logfile){  //If data file is open
    logfile.print("Time: "); logfile.print(","); logfile.println(time);
    logfile.print("Acceleration in x = "); logfile.print(","); logfile.println(mma.x);
    logfile.print("Acceleration in y = "); logfile.print(","); logfile.println(mma.y);
    logfile.print("Acceleration in z = "); logfile.print(","); logfile.println(mma.z);
    logfile.println("Acceleration in 1D: "); logfile.print(","); logfile.print(acc_1d); logfile.println("m/s^2 ");

    logfile.print("Rotation for x: "); logfile.print(","); logfile.print(angle_x); logfile.println(" deg");
    logfile.print("Rotation for y: "); logfile.print(","); logfile.print(angle_y); logfile.println(" deg");
    logfile.print("Rotation for z: "); logfile.print(","); logfile.print(angle_z); logfile.println(" deg");
  
    
    logfile.print("TemperatureA [C]: "); logfile.print(","); logfile.println(sensor.GetTemp()*0.01);
    logfile.print("Pressure [Pa]: "); logfile.print(","); logfile.println(sensor.GetPres());
    logfile.print("Altitude [m]: "); logfile.print(","); logfile.println(altitude*0.3048);

    logfile.print("Vibration [Hz]: "); logfile.print(","); logfile.println(vibSensor);
    logfile.close();
}
}

//Testing the readings for altimeter
void test_crc() {
  sensor.ReadProm();
  sensor.Readout(); 
  Serial.print("CRC=0x");
  Serial.print(sensor.Calc_CRC4(), HEX);
  Serial.print(" (should be 0x");
  Serial.print(sensor.Read_CRC4(), HEX);
  Serial.print(")\n");
  Serial.print("Test Code CRC=0x");
  Serial.print(sensor.CRCcodeTest(), HEX);
  Serial.println(" (should be 0xB)");
}
