//Author : Matheus Henrique de Souza Passos
/*
 * In this test, the system will be stationary( Acceleration = 0 and Velocity=0) to see KF's behaviour.
 */
#include <arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SPI.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

//Declaring Head of Functions. Functions algorithms will be after "MAIN" block

void UpdatePredictValue(float Value1,float Value2,float CycleTime,float Accel);
void ProcessError(float Error1,float Error2);
float KalmanGain(float ErrorProcessK,float ErrorObsK);
float CalculateCurrentState(float VarPredict,float VarObserved,float KG);
void UpdateProcessError(float KG_Variable1,float KG_Variable2);

//END Declaring Head of Functions--------------------------


//Declaring Variables(Global) to be used-------------------------------
Adafruit_BMP280 bmp; //Sensor bmp280 to get measured height
int counter=0;//Simply to count number of interactions

//Some of these variables will need to be adjust for their initial value by taking a reasonable guess.
float ReadValue1=0,ReadValue2=0; //Will receive values of sensors.
float PredictValue1=0,PredictValue2=0; //Predicted(theorical) values. Example : V=Vo+at and ΔH=Vo*t + (a*t^2)/2. Use the 't' as the ammout of time between interactions/reads.
float FK_Value1=0,FK_Value2=0;

float CycleTime=1; //Sets as the rate wich you receive information from sensor. In case of a rocket, it will greatly vary at the very beginning before becoming a "constant". In this case, it will be time between iteractions
float Accel=0; //Sets as the acceleration you predict the system will be operating.
float ProcessErrorVector[2]= {20,0}; //Contains the ProcessError(Example : Height x Velocity(causes height variance)) . We are considering Position process error = 20
float KG_Variable1=0,KG_Variable2=0; //Each variable(example : Height x Velocity has its own KalmanGain). 

//In this case, BMP280 has a relative accuracy of  +- 1 m at 25°C. ObsError 2 is 0 for stationary test without sensor. But we will use 10 based on experience
float ObsError1=10,ObsError2=0; //Observation Error(measurement Error) of measuring that respective value. Normally, got from datasheet.
float Qk=0; //Error in the process of calculating the ProcessCovariance

//END Declaring Variables-------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Sensors test and configuration
  if(!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  //Configurating sensor
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  //END OF SENSOR TEST AND CONFIGURATION

  //Setting initial measurements
  ReadValue1=bmp.readAltitude(1024) ; //Getting Height measured by BMP280. The parameter is the absolut pressure in your local
  //PredictValue1= ReadValue1; // Initial value, in case you dont know exactly your absolute height. -10 just to cause a small difference at the beginning
  PredictValue1= 350; 
  //ReadValue2=0 // Because we are doing a stationary test,velocity is 0. Also, we dont have a sensor measuring velocity and we are considering Acceleration = 0 without a sensor.
  Serial.print(F("INITIAL VALUE ="));
  Serial.println(ReadValue1);
  
}

//================================================================================================
void loop() {//----------------------------=========================================================
 
//Begin iterations
UpdatePredictValue(PredictValue1,PredictValue2,CycleTime,Accel); //Updates the predicted state : Current Becomes Previous after first interaction
ProcessError(ProcessErrorVector[0],ProcessErrorVector[1]);      //Updates the predict process Error

//Calculate KG for each variable error
KG_Variable1= KalmanGain(ProcessErrorVector[0],ObsError1);
KG_Variable2= KalmanGain(ProcessErrorVector[1],ObsError2);

//Calcutes CurrentState
FK_Value1=CalculateCurrentState(PredictValue1,ReadValue1,KG_Variable1); //Calculates new predict Height
FK_Value2=CalculateCurrentState(PredictValue2,ReadValue2,KG_Variable2); //Calculates new predict Velocity

//Adjust ProcessError for next Interaction
UpdateProcessError(KG_Variable1,KG_Variable2);
//End interations


//Print Values on Monitor for Analysis
Serial.print("Measured Height is : ");
Serial.println(ReadValue1);
Serial.print("Kalman Filter Height is :");
Serial.println(FK_Value1);
Serial.print("Mesaured Velocity is : ");
Serial.println(ReadValue2);
Serial.print("Kalman Filter Velocity is : ");
Serial.println(FK_Value2);

Serial.print("End of iteration : ");
Serial.println(counter);


Serial.println("-----STARTING NEXT ITERATION-------");


//Get sensors values and calcultates
delay(1000); //Just for test. Dont use delay on real application

ReadValue1=bmp.readAltitude(1024); //New measured value
//ReadValue2= 0; // Since we are not measuring velocity. Also, in this test, Velocity and Acceleration = 0. In application, you need to measure velocity.


counter++;
}//END of loop--------------------------------



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
    if(ErrorObsK== 0 && ErrorProcessK==0 )
    return 1; 
    
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
