#include <Wire.h>

int i, CF=1; 
float radDeg=180/PI, angleX=0;

// Accelerometer Variables
long accel_rawX, accel_rawY, accel_rawZ;
int accelFS=0, accel_LPF=0;// accelFS may take values 0,1,2 and 3. accel_LPF=0 for No LPF 1 for LPF
float accelFSRange;
float accel_calX, accel_calY, accel_calZ;
double accel_X, accel_Y, accel_Z;
int accelCalSamples=2000, accelRollAvg=100;
float accel_angleX, accel_angleY;

long gyro_rawX, gyro_rawY, gyro_rawZ;
int gyroFS=1;// It may take values 0,1,2 and 3
float gyroFSRange;
float gyro_calX, gyro_calY, gyro_calZ;
double gyro_X, gyro_Y, gyro_Z;
int gyroCalSamples=2000;
float gyro_angleX, gyro_angleY, gyro_angleZ;
long elapsedTime=0, previous_time=0, current_time=0;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  accelCalibration(accelCalSamples); // Calibration is on the coverted Data ()
  gyroCalibration(gyroCalSamples); // Calibration is on the coverted Data ()
  delay(2000);
}
void loop() 
{
  current_time = millis();  // actual time read in ms
  elapsedTime = (current_time - previous_time); // time duration in milli Seconds

  accelRawData(); // The output is stored in accel_rawX, accel_rawY and accel_rawZ
  accelMechFilter(5); // Check after commenting this also for your system
  accelCalData();  // Apply the bias - calculated using Calibration
  accelData();  // convert into gForce. This function to be called at last after signal processing
  accelAngle(); // Calculate the Angle using rotation Matrix Concept 

  gyroRawData(); // The output is stored in gyro_rawX, gyro_rawY and gyro_rawZ
  gyroCalData();  // Apply the bias - calculated using Calibration
  gyroData();  // This function to be called at last after signal processing

  gyro_angleX+=gyro_X*0.001*elapsedTime;
  gyro_angleY+=gyro_Y*0.001*elapsedTime;
  gyro_angleZ+=gyro_Z*0.001*elapsedTime;
  if(CF==1)complementryFilter();
  else angleX=0;
    
  plotData(accel_angleX, gyro_angleX, angleX);

  while (millis()-current_time<4);
  previous_time = current_time;  // the previous time is stored before the actual time read
}

void setupMPU()
{
// Power management Tasks
  Wire.beginTransmission(0b1101000); //Slave Device Address Sequence - This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Internal Register Address Sequence - Accessing the register 6B - Power Management (Sec. 4.28) 
  Wire.write(0b00000000); //Data Transfer Bit Sequence - Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
// Configer Accelerometer for use
  Wire.beginTransmission(0b1101000); //Slave Device Address Sequence - I2C address of the MPU
  Wire.write(0x1C); //Internal Register Address Sequence - Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  
  if(accelFS==0)
  {
    Wire.write(0b00000000); 
    accelFSRange=16384;
  }
  else if(accelFS==1)
  {
    Wire.write(0b00001000); 
    accelFSRange=8192;  
  }
  else if(accelFS==2)
  {
    Wire.write(0b00010000); 
    accelFSRange=4096;
  }
  else 
  {
    Wire.write(0b00011000); 
    accelFSRange=2048;
  }
  Wire.endTransmission();
  // Configer gyrometer for use
  Wire.beginTransmission(0b1101000); //Slave Device Address Sequence - I2C address of the MPU
  Wire.write(0x1B); //Internal Register Address Sequence - Accessing the register 1N - gyro Configuration (Sec. 4.5) 
  //Data Transfer Bit Sequence - Setting the gyro to gyroFSRange - LSB Sensitivity
  if(gyroFS==0)
  {
    Wire.write(0b00000000); 
    gyroFSRange=131;
  }
  else if(gyroFS==1)
  {
    Wire.write(0b00001000); 
    gyroFSRange=65.5;  
  }
  else if(gyroFS==2)
  {
    Wire.write(0b00010000); 
    gyroFSRange=32.8;
  }
  else 
  {
    Wire.write(0b00011000); 
    gyroFSRange=16.4;
  }
  Wire.endTransmission();
}

void accelCalibration(int calSamples)
{
  for(int i=0; i<calSamples; i++)
  {
    accelRawData();
    accel_calX+=accel_rawX;
    accel_calY+=accel_rawY;
    accel_calZ+=accel_rawZ;
  }
  accel_calX=accel_calX/calSamples;
  accel_calY=accel_calY/calSamples;
  accel_calZ=accel_calZ/calSamples;
}

void accelRawData() 
{
  Wire.beginTransmission(0b1101000); //Slave Device Address Sequence - I2C address of the MPU
  Wire.write(0x3B); //Internal Register Address Sequence - Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accel_rawX = Wire.read()<<8|Wire.read(); //Store first two bytes into accel_rawX after converting into decimal
  accel_rawY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accel_rawY
  accel_rawZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accel_rawZ
}
void accelData()
{
  accel_X =float(accel_rawX)/float(accelFSRange);
  accel_Y =float(accel_rawY)/float(accelFSRange);
  accel_Z =1.0+float(accel_rawZ)/float(accelFSRange);
}

void plotData(float first, float second, float third) 
{
  Serial.print(first); // Blue on Plotter
  Serial.print(",");
  Serial.print(second); // Red on Plotter
  Serial.print(",");
  Serial.println(third); // Green on Plotter
}

void rollAvg(int ra)
{
  float sampleSumX=0, sampleSumY=0, sampleSumZ=0;
  i=0;
  while(i<ra)
  {
    accelRawData();
    sampleSumX=sampleSumX+accel_rawX;
    sampleSumY=sampleSumY+accel_rawY;
    sampleSumZ=sampleSumZ+accel_rawZ;
    i++;
    //Serial.print(sampleSumX);
  }
  accel_rawX=sampleSumX/ra;
  accel_rawY=sampleSumY/ra;
  accel_rawZ=sampleSumZ/ra;
}
void accelMechFilter(int amf)
{
  if(accel_rawX>-amf && accel_rawX<amf) accel_rawX=0.0;
  if(accel_rawY>-amf && accel_rawY<amf) accel_rawY=0.0;
  if(accel_rawZ>-amf && accel_rawZ<amf) accel_rawZ=0.0;
}
void accelCalData()
{
  accel_rawX=accel_rawX-accel_calX;
  accel_rawY=accel_rawY-accel_calY;
  accel_rawZ=accel_rawZ-accel_calZ;
}
void accelAngle()
{
  if(accel_LPF==0)
  {
    accel_angleX=atan(accel_Y/sqrt(pow(accel_X,2)+pow(accel_Z,2)))*radDeg;
    accel_angleY= atan(-accel_X/sqrt(pow(accel_Y,2)+pow(accel_Z,2)))*radDeg;
  }
  else
  {
    accel_angleX=0.95*accel_angleX+0.05*atan(accel_Y/sqrt(pow(accel_X,2)+pow(accel_Z,2)))*radDeg;
    accel_angleY=0.95*accel_angleX+0.05*atan(-accel_X/sqrt(pow(accel_Y,2)+pow(accel_Z,2)))*radDeg;
  }
  
}

void gyroCalibration(int calSamples)
{
  for(int i=0; i<calSamples; i++)
  {
    gyroRawData();
    gyro_calX+=gyro_rawX;
    gyro_calY+=gyro_rawY;
    gyro_calZ+=gyro_rawZ;
  }
  gyro_calX=gyro_calX/calSamples;
  gyro_calY=gyro_calY/calSamples;
  gyro_calZ=gyro_calZ/calSamples;
}

void gyroRawData() 
{
  Wire.beginTransmission(0b1101000); //Slave Device Address Sequence - I2C address of the MPU
  Wire.write(0x43); //Internal Register Address Sequence - Starting register for gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyro_rawX = Wire.read()<<8|Wire.read(); //Store first two bytes into gyro_rawX after converting into decimal
  gyro_rawY = Wire.read()<<8|Wire.read(); //Store middle two bytes into gyro_rawY
  gyro_rawZ = Wire.read()<<8|Wire.read(); //Store last two bytes into gyro_rawZ
}
void gyroData()
{
  gyro_X =float(gyro_rawX)/float(gyroFSRange);
  gyro_Y =float(gyro_rawY)/float(gyroFSRange);
  gyro_Z =float(gyro_rawZ)/float(gyroFSRange);
}

void gyroCalData()
{
  gyro_rawX=gyro_rawX-gyro_calX;
  gyro_rawY=gyro_rawY-gyro_calY;
  gyro_rawZ=gyro_rawZ-gyro_calZ;
}
void complementryFilter()
{
  angleX=(angleX+gyro_X*0.001*elapsedTime)*0.98+0.02*accel_angleX;
}
