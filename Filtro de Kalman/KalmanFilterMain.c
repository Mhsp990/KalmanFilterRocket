#include <arduino.h>


//Declaring Head of Functions. Functions algorithms will be after "MAIN" block
void UpdatePredictValue(float Value1,float Value2,float CycleTime,float Accel);
void ProcessError(float Error1,float Error2);
float KalmanGain(float ErrorProcessK,float ErrorObsK);
float CalculateCurrentState(float VarPredict,float VarObserved,float KG);
void UpdateProcessError(float KG_Variable1,float KG_Variable2);


int main() 
{    

//Declaring Variables to be used
float ReadValue1=0,ReadValue2=0; //Will receive values of sensors.
float PredictValue1=0,PredictValue2=0; //Predicted(theorical) values. Example : V=Vo+at and ΔH=Vo*t + (a*t^2)/2. Use the 't' as the ammout of time between interactions/reads.

float ProcessErrorVector[2]= {0,0}; //Contains the ProcessError(Example : Height x Velocity(causes height variance))
float KG_Variable1=0,KG_Variable2=0; //Each variable(example : Height x Velocity has its own KalmanGain)
float ObsError1=0,ObsError2=0; //Observation Error(measurement Error) of measuring that respective value. Normally, got from datasheet.
float Qk=0; //Error in the process of calculating the ProcessCovariance

//End of Variables-----



} //END OF MAIN



//Author: Matheus Henrique de Souza Passos





//FUNCTIONS TO BE USED ON KALMAN FILTER


//Updates the new state matrix based on the previous result from kalman Filter. In the first interaction, use the initial condition
//Example : value1 = height and value2 = velocity; CycleTime= Time elapsed between sensor(values) reads
void UpdatePredictValue(float Value1,float Value2,float CycleTime,float Accel)
{
PredictValue2= Value2 + Accel*CycleTime; //Calculates Velocity : V= Vo+ A*T
PredictValue1= Value1 + Value2*CycleTime+ ((Accel*CycleTime*CycleTime)/2) ; //Calculates Height : H= Ho + Vo*T + 0.5*A*(T^2)


}


//Sets the ProcessErrorVector : Insert the values of previous ProcessError to get the Predict ProcessError;
void ProcessError(float Error1,float Error2)
{
Error2=Error2*Error2; //Power 2
Error1=Error1*Error1; //Power 2
ProcessErrorVector[0]=(Error1) + Error2 +Qk;
ProcessErrorVector[1]=(Error2) + Qk;
}



//Calculating KG ;  Uses ProcessError dividing for ObservationalError to get the KalmanGain for each variable(Example: KG for Height and Velocity)
//Returns a float, so to get both KG for each variable, use once for each passing the value refering to that variable.
//It calculates how much "trust" or weight will be used for that variable, comparing the expect Error with the MeasurementError.
float KalmanGain(float ErrorProcessK,float ErrorObsK) 
{
    ErrorObsK=ErrorObsK*ErrorObsK; //Gets the square. ErrorProcessK was calculated on ProcessErrorVector
    return ErrorProcessK/(ErrorProcessK+ErrorObsK);
}


//Calculate Current State , based on the predict state, the difference between the predict and observed state, together with the kalman gain
//VarPredict : Use the value calculated by the Kalman Filter on the previous interaction.
//Use once for each variable(example : Height and Velocity),each time with its respective value.
float CalculateCurrentState(float VarPredict,float VarObserved,float KG)
{
return VarPredict+(KG*(VarObserved-VarPredict)); //Returns the calculated value of that variable calculated by the kalman filter.
}



//Updates ProcessError Expectation before next interaction, so the next interaction uses these values next on 
void UpdateProcessError(float KG_Variable1,float KG_Variable2)
{
    ProcessErrorVector[0]=ProcessErrorVector[0]*(1-KG_Variable1);
    ProcessErrorVector[1]=ProcessErrorVector[1]*(1-KG_Variable2);
}



//End of Functions