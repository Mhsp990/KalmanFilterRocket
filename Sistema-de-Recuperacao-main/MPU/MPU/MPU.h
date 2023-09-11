/*
  MPU CACTUS
  Autores: Alan Victor e Marina Melo
  Agosto de 2021
*/
#ifndef mpu_H
#define mpu_H

#include <Arduino.h>

class MPU6050
{
	public:
	  	MPU6050(int _ADR = 0x68, float _CONST = 0.02);
		void beginMPU();
		void calibrate(int ITERATIONS = 3000);
	    
		void readData();
		void update();
		void updateVelocity();
		void updateAccInWorldFrame();

		void AccelFiltro();
		void lowPassFilter();

	    float accelX, accelY, accelZ;
		float gyroX, gyroY, gyroZ, temp;
		float gyroXOffset, gyroYOffset, gyroZOffset;

		float gyroAngleX, gyroAngleY, gyroAngleZ;
		float accelAngleX, accelAngleY;

		float yaw, pitch, roll;
		double accWorldFrameX, accWorldFrameY, accWorldFrameZ;
		double velX, velY, velZ;
		
		double GRAVITY;
	private:
		bool CALIBRATED;
		float CONST;
		int ADR;
		
};

#endif

