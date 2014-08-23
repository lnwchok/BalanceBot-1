#include <Wire.h>
#include <ADXL345.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

double fXg = 0;
double fYg = 0;
double fZg = 0;

ADXL345 acc;
double Xg, Yg, Zg;
float accbiasX, accbiasY, accbiasZ;
double accPitch = 0;

int L3G4200D_Address = 105; //I2C address of the L3G4200D
int x;
int y;
int z;
float gyrobiasX, gyrobiasY, gyrobiasZ;
float gyroRateX, gyroRateY, gyroRateZ;
double gyroPitch = 0;

double PitchAngle = 0;
uint32_t timer;
float timeStep = 0.2; //(0.2 sec) 200ms for time step value

void setup(){
  float TotalgyrobiasX = 0;
  float TotalgyrobiasY = 0;
  float TotalgyrobiasZ = 0;
  float TotalaccbiasX = 0;
  float TotalaccbiasY = 0;
  float TotalaccbiasZ = 0;
  
  double initPitch = 0;
  Wire.begin();
  acc.begin();

  Serial.begin(9600);
  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1000); //wait for the sensor to be ready 

  for (int i=1; i<100; i++){
    getGyroValues();
    TotalgyrobiasX += (int)x;
    TotalgyrobiasY += (int)y;
    TotalgyrobiasZ += (int)z;
    acc.read(&Xg, &Yg, &Zg);
    TotalaccbiasX += Xg;
    TotalaccbiasY += Yg;
    TotalaccbiasZ += Zg;
    delay(1);
  }

 // Final bias values for every axis  
  gyrobiasX = TotalgyrobiasX / 100;
  gyrobiasY = TotalgyrobiasY / 100;
  gyrobiasZ = TotalgyrobiasZ / 100;
  accbiasX = TotalaccbiasX / 100;
  accbiasY = TotalaccbiasX / 100;
  accbiasZ = TotalaccbiasX / 100;
  
  
  getGyroValues();
  gyroRateX = ((int)x - gyrobiasX)*.07;
  initPitch = gyroRateX * timeStep;
  Serial.print("Initial Pitch angle >> ");
  Serial.println(initPitch);
  
  InitialValues();
  
  Serial.print("RateX\tRateY\tRateZ\n");

}

double InitialPitch;

void InitialValues(){
  
  //  Accelerometer   
//  sensors_event_t event; 
  double InitialAngle = 0;
  double dGyro = 0;
  for(int i=1; i < 100; i++){      // Takes 100 values to get more precision
    
//    accel.getEvent(&event);
//    accRoll = (atan2(event.acceleration.y-accBiasY,-(event.acceleration.z-accBiasZ))+PI)*RAD_TO_DEG;

//  acc.read(&Xg, &Yg, &Zg);
//  accPitch = (atan2(-Yg, Zg)*180.0)/M_PI;
//  if (accPitch < 0) {
//    accPitch = -(accPitch + 180);
//  } else {
//    accPitch = 180 - accPitch;
//  }
  
        
//  getGyroValues();
//  gyroRateX = ((int)x - gyrobiasX)*.07; 
//
//  dGyro = gyroRateX * ((double)(micros() - timer)/1000000);
//  
  InitialAngle = 0.98* (InitialAngle + get_gyroRateRoll(gyrobiasX)) + 0.02 * (getAccPitch());
  timer = micros();
  
  delay(1);
  
  }
 
  InitialPitch = InitialAngle;
  
  Serial.print("Pitch Initial: ");
  Serial.println(InitialPitch);
  
}

double getAccPitch() {
  double _accPitch;
  acc.read(&Xg, &Yg, &Zg);
  _accPitch = (atan2(-Yg, Zg)*180.0)/M_PI;
  if (_accPitch < 0) {
    _accPitch = -(_accPitch + 180);
  } else {
    _accPitch = 180 - _accPitch;
  }
  return _accPitch;
}

double get_gyroRateRoll(float rollbias){
  
  getGyroValues();     // Get values from gyro
  
  // read raw angular velocity measurements from device  
  float gyrorate = ((int)x - rollbias)*.07; 
  
//  double dgyroRoll = gyroRateX * ((double)(micros() - timer)/1000000);
  double dgyroRoll = gyroRateX * timeStep;
  
  return dgyroRoll;
  
}

void loop(){
//  getGyroValues();  // This will update x, y, and z with new values
//  Serial.print(millis());
//  Serial.print(", ");
//  Serial.print(x);
//  Serial.print(", ");
//  Serial.print(y);
//  Serial.print(", ");
//  Serial.println(z);
//  delay(200); //Just here to slow down the serial to make it more readable

  timer = micros(); //initial time for loop taken
  
  getGyroValues();
  gyroRateX = ((int)x - gyrobiasX)*.07;
//  gyroRateY = ((int)y - gyrobiasY)*.07; 
//  gyroRateZ = ((int)z - gyrobiasZ)*.07;
//  
//  gyroPitch += gyroRateX * ((double)(micros() - timer)/1000000);
  gyroPitch += gyroRateX * timeStep;
  
  acc.read(&Xg, &Yg, &Zg);
//  accPitch = (atan2(Xg - accbiasX, sqrt((Yg - accbiasY)*(Yg - accbiasY) + (Zg - accbiasZ)*(Zg - accbiasZ)))*180)/M_PI;
  accPitch = (atan2(-(Yg-accbiasY), (Zg-accbiasZ))*180.0)/M_PI;
  if (accPitch < 0) {
    accPitch = -(accPitch + 180);
  } else {
    accPitch = 180 - accPitch;
  }
  
  PitchAngle = (0.98 * gyroPitch) + (0.02 * accPitch);
  
//  Serial.print(millis());
//  Serial.print(", ");
//  Serial.print(gyroRateX);
//  Serial.print(", ");
//  Serial.print(gyroRateY);
//  Serial.print(", ");
//  Serial.print(gyroRateZ);  
//  Serial.println(gyroRateZ);  
//  Serial.print(" >> ");
//  Serial.println(gyroRoll);  


  Serial.print(gyroRateX);
  Serial.print("\t");
  Serial.print(gyroRateY);
  Serial.print("\t");
  Serial.print(gyroRateZ);
  Serial.print("\t");
  Serial.print(gyroPitch);
  Serial.print("\t");
  Serial.print(accPitch);
  Serial.print("\t");
  Serial.println(PitchAngle);
  
//  timer = micros() - timer;
//  timer = ((timeStep * 1000000) - timer)/1000;
  delay(timeStep*1000);
//  delay(400);

}

//////////////////////////////////////////////////////////
//   GYRO FUNCTION by Jim Lindblom of Sparkfun's code   //
//////////////////////////////////////////////////////////

void getGyroValues(){
  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);
  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}
int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code
  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);
  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);
  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);
  // CTRL_REG4 controls the full-scale range, among other things:
  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }
  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}
void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}
int readRegister(int deviceAddress, byte address){
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 1); // read a byte
    while(!Wire.available()) {
        // waiting
    }
    v = Wire.read();
    return v;
}
