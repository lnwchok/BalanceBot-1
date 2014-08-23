#include <Wire.h>
#include <ADXL345.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D
int x;
int y;
int z;
float gyrobiasX, gyrobiasY, gyrobiasZ;
float gyroRateX, gyroRateY, gyroRateZ;

const float alpha = 0.5;
//double fXg = 0;
//double fYg = 0;
//double fZg = 0;
double Xg, Yg, Zg;
float accbiasX, accbiasY, accbiasZ;

ADXL345 acc;


//float timeStep = 0.2; //(0.2 sec) 200ms for time step value

void InitSensors(){
  float TotalgyrobiasX = 0;
//  float TotalgyrobiasY = 0;
//  float TotalgyrobiasZ = 0;
  float TotalaccbiasX = 0;
  float TotalaccbiasY = 0;
  float TotalaccbiasZ = 0;
//  double initRoll = 0;
  
  Wire.begin();
  acc.begin();
  Serial.begin(9600);
  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1000); //wait for the sensor to be ready 

  for (int i=1; i<100; i++){
    getGyroValues();
    TotalgyrobiasX += (int)x;
//    TotalgyrobiasY += (int)y;
//    TotalgyrobiasZ += (int)z;
    acc.read(&Xg, &Yg, &Zg);
    TotalaccbiasX += (int)Xg;
    TotalaccbiasY += (int)Yg;
    TotalaccbiasZ += (int)Zg;
    delay(1);
  }

 // Final bias values for every axis  
  gyrobiasX = TotalgyrobiasX / 100;
//  gyrobiasY = TotalgyrobiasY / 100;
//  gyrobiasZ = TotalgyrobiasZ / 100;
  accbiasX = TotalaccbiasX / 100;
  accbiasY = TotalaccbiasY / 100;
  accbiasZ = TotalaccbiasZ / 100;

  //roll  = (atan2(-fYg, fZg)*180.0)/M_PI;

//  InitValues();

}

void InitValues() {
  double initRoll = 0;
  double _accRoll = 0;
  double _gRoll = 0;
  for (int i=1; i < 100; i++) {
    
    acc.read(&Xg, &Yg, &Zg);
    _accRoll  = (atan2(-(Yg - accbiasY), (Zg - accbiasZ))*180.0)/M_PI;

    getGyroValues();
    gyroRateX = ((int)x - gyrobiasX)*.07;
//    _Roll = gyroRateX * ((double)(micros() - timer)/1000000);
    initRoll += gyroRateX * ((double)(micros() - timer)/1000000);
    timer = micros();
    delay(1);
  }
  Serial.print("Initial Roll angle >> ");
  Serial.print(initRoll);
  Serial.print(" : ");
  Serial.println(_accRoll);
  
  RollAngle = initRoll;

}

double getGyroRoll() {
  getGyroValues();
  double _rateX = ((int)x - gyrobiasX)*.07;
  double _gyroRoll = _rateX * ((double)(micros() - timer)/1000000);
  return _gyroRoll;
}

double getAccRoll() {
  acc.read(&Xg, &Yg, &Zg);
  double _accRoll  = (atan2(-(Yg - accbiasY), (Zg - accbiasZ))*180.0)/M_PI;
  return _accRoll;
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

