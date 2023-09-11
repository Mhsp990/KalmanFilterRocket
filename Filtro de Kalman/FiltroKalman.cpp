
#include <arduino.h>



//FUNCTIONS TO BE USED ON KALMAN FILTER

float calculateVelocity()
{

}


//Calculating KG ;  ErrE= Estimated Error;    ErrM= MensuredError ; Bigger differences(ErrE and ErrM) causes bigger KG;
float KalmanGain(ErrE,ErrM)
{
    return ErrE/(ErrE+ErrM);
}


float ActualEstimate(float EstimateK0, float KG)


float UpdateEstimative(float Height,float Accel,)


//End of Functions