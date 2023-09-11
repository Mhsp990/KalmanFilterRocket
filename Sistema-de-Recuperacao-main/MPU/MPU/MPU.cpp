/*
  MPU CACTUS
  Autores: Alan Victor e Marina Melo
  Agosto de 2021
*/

#include "MPU.h"
#include "Wire.h"

#define lpCoef 0.80
#define DEG_RAD 0.0174533

double time, dt, xA, yA, zA;

MPU6050::MPU6050(int _ADR, float _CONST)
{
  CALIBRATED = false;

  ADR = _ADR;
  CONST = _CONST;
  
  GRAVITY = 0;

  gyroXOffset = gyroYOffset = gyroZOffset = 0;
  gyroAngleX = gyroAngleY = gyroAngleZ = 0;
}

void MPU6050::beginMPU(){
  Wire.begin();
  Wire.beginTransmission(ADR);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission();

  /*
  Configura o giroscópio

  Wire.write(0b00000000); //   +/- 250 °/s
  Wire.write(0b00001000); //   +/- 500 °/s
  Wire.write(0b00010000); //   +/- 1000 °/s
  Wire.write(0b00011000); //   +/- 2000 °/s
  */

  Wire.beginTransmission(ADR);
  Wire.write(0x1B);
  Wire.write(0b00010000);
  Wire.endTransmission();
 
  /*
  Configura o acelerômetro

  Wire.write(0b00000000); //   +/- 2g
  Wire.write(0b00001000); //   +/- 4g
  Wire.write(0b00010000); //   +/- 8g
  Wire.write(0b00011000); //   +/- 16g
  */

  Wire.beginTransmission(ADR);
  Wire.write(0x1C);
  Wire.write(0b00011000);
  Wire.endTransmission();


  // Outras
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x19);
  Wire.write(0x00);
  Wire.endTransmission();
}

void MPU6050::readData(){
  Wire.beginTransmission(ADR);  
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(ADR, 14, true);

  accelX = Wire.read() <<8 | Wire.read(); //0x3B (ACCEL_XOU_H) & 0x3C(ACCEL_XOUT_L)
  accelY = Wire.read() <<8 | Wire.read(); //0x3D (ACCEL_YOU_H) & 0x3E(ACCEL_YOUT_L)
  accelZ = Wire.read() <<8 | Wire.read(); //0x3F (ACCEL_ZOU_H) & 0x40(ACCEL_ZOUT_L)
  temp   = Wire.read() <<8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX  = Wire.read() <<8 | Wire.read(); //0x43 (GYRO_XOU_H) & 0x44(GYRO_XOUT_L)
  gyroY  = Wire.read() <<8 | Wire.read(); //0x45 (GYRO_YOU_H) & 0x46(GYRO_YOUT_L)
  gyroZ  = Wire.read() <<8 | Wire.read(); //0x47 (GYRO_ZOU_H) & 0x48(GYRO_ZOUT_L)

  /*

  Divide conforme o fundo de escala

  -> acelerômetro
  +/- 2g = 16384
  +/- 4g = 8192
  +/- 8g = 4096
  +/- 16 = 2048

  ->giroscópio
  +/- 250°/s = 131
  +/- 500°/s = 65.5
  +/- 1000°/s = 32.8
  +/- 2000°/s = 16.4

  */
  accelX = ((float)accelX)/2048;
  accelY = ((float)accelY)/2048;
  accelZ = ((float)accelZ)/2048;
  temp   = ((float)(temp) / 340.0) + 36.53;
  gyroX  = ((float)gyroX)/32.8;
  gyroY  = ((float)gyroY)/32.8;
  gyroZ  = ((float)gyroZ)/32.8;

  // Troca de eixos
  
  float r = accelZ;
  float s = gyroZ;
  accelZ = accelY;
  accelY = -r;
  gyroZ = gyroY;
  gyroY = -s;
  
  dt = (millis()-time)/1000.0;
  time = millis();
}

void MPU6050::update()
{
  readData();

  if(CALIBRATED){
    gyroX -= gyroXOffset;
    gyroY -= gyroYOffset;
    gyroZ -= gyroZOffset;

    float x = roll*0.0174533;
    float y = pitch*0.0174533;

    gyroAngleX += (gyroX + gyroY*sin(x)*tan(y) + gyroZ*cos(x)*tan(y))*dt;
    gyroAngleY += (gyroY*cos(x) - gyroZ*sin(x))*dt;
    gyroAngleZ += (gyroY*sin(x)*(1/cos(y)) + gyroZ*cos(x)*(1/cos(y)))*dt;

    AccelFiltro();

    accelAngleX = atan2(accelY, sqrt(accelZ * accelZ + accelX * accelX)) * 180 / PI;
    accelAngleY = atan2(accelX, sqrt(accelZ * accelZ + accelY * accelY)) * -180 / PI;
	
    roll = accelAngleX*CONST + gyroAngleX*(1-CONST);
    pitch = accelAngleY*CONST + gyroAngleY*(1-CONST);
	
    yaw = gyroAngleZ;
    updateAccInWorldFrame();
    updateVelocity();
  }
}

void MPU6050::calibrate(int ITERATIONS){
  for(int i=0 ; i<100 ; ++i){
    readData();
  }
  for(int i=0 ; i<ITERATIONS ; ++i){
    readData();
    delay(2);

    gyroXOffset += gyroX;
    gyroYOffset += gyroY;
    gyroZOffset += gyroZ;
	
    GRAVITY += sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
  }

  CALIBRATED = true;

  gyroXOffset /= (float)ITERATIONS;
  gyroYOffset /= (float)ITERATIONS;
  gyroZOffset /= (float)ITERATIONS;

  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;

  AccelFiltro();

  accelAngleX = atan2(accelY, sqrt(accelZ * accelZ + accelX * accelX)) * 180 / PI;
  accelAngleY = atan2(accelX, sqrt(accelZ * accelZ + accelY * accelY)) * -180 / PI;
  
  float x = accelAngleX*0.0174533;
  float y = accelAngleY*0.0174533;

  gyroAngleX += (gyroX + gyroY*sin(x)*tan(y) + gyroZ*cos(x)*tan(y))*dt;
  gyroAngleY += (gyroY*cos(x) - gyroZ*sin(x))*dt;
  gyroAngleZ += (gyroY*sin(x)*(1/cos(y)) + gyroZ*cos(x)*(1/cos(y)))*dt;

  roll = accelAngleX*CONST + gyroAngleX*(1-CONST);
  pitch = accelAngleY*CONST + gyroAngleY*(1-CONST);

  GRAVITY /= (float)ITERATIONS*(accelZ>0 ? +1 : -1);
}

void MPU6050::updateVelocity(){
  velX += accWorldFrameX*dt;
  velY += accWorldFrameY*dt;
  velZ += accWorldFrameZ*dt;
}

// Calcula a aceleração nas Coordenadas Globais
void MPU6050::updateAccInWorldFrame(){
  float x = roll*0.0174533;
  float y = pitch*0.0174533;
  float z = yaw*0.0174533;

  accWorldFrameX = ( accelX )*( cos(z)*cos(y) )     + ( accelY )*( cos(z)*sin(y)*sin(x) - sin(z)*cos(x) ) + ( accelZ )*( cos(z)*sin(y)*cos(x) + sin(z)*sin(x) );
  accWorldFrameY = ( accelX )*( sin(z)*cos(y) )     + ( accelY )*( sin(z)*sin(y)*sin(x) + cos(z)*cos(x) ) + ( accelZ )*( sin(z)*sin(y)*cos(x) - cos(z)*sin(x) ); 
  accWorldFrameZ = (-GRAVITY) + ( accelX )*( -sin(y) ) + ( accelY )*( cos(y)*sin(x) )                     + ( accelZ )*( cos(y)*cos(x) );
}

void MPU6050::AccelFiltro(){
  //lowPassFilter();
}

void MPU6050::lowPassFilter(){
  accelX = xA*lpCoef + accelX*(1-lpCoef);
  accelY = yA*lpCoef + accelY*(1-lpCoef);
  accelZ = zA*lpCoef + accelZ*(1-lpCoef);

  xA = accelX;
  yA = accelY;
  zA = accelZ;
}

