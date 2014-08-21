
//Arduino 1.0+ only
#include <Wire.h>

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
double gyroRoll = 0;
uint32_t timer;
float timeStep = 0.2; //(0.2 sec) 200ms for time step value

void setup(){
  float totalbiasX = 0;
  float totalbiasY = 0;
  float totalbiasZ = 0;
  Wire.begin();
  Serial.begin(9600);
  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1000); //wait for the sensor to be ready 

  for (int i=1; i<100; i++){
    getGyroValues();
    totalbiasX += (int)x;
    totalbiasY += (int)y;
    totalbiasZ += (int)z;
    delay(1);
  }

 // Final bias values for every axis  
  gyrobiasX = totalbiasX / 100;
  gyrobiasY = totalbiasY / 100;
  gyrobiasZ = totalbiasZ / 100;
  Serial.print("RateX\tRateY\tRateZ\n");

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
  gyroRateY = ((int)y - gyrobiasY)*.07; 
  gyroRateZ = ((int)z - gyrobiasZ)*.07;
  
  gyroRoll += gyroRateX * timeStep;
  
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
  Serial.println(gyroRoll);
  
  timer = micros() - timer;
  timer = ((timeStep * 1000000) - timer)/1000;
  delay(timer);
//  delay(400);

}

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
