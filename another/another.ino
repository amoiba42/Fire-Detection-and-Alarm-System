#include <ESP32Servo.h>
#define         MQ_PIN                       (12)    
#define         RL_VALUE                     (5)    
#define         RO_CLEAN_AIR_FACTOR          (9.83)

#define         CALIBARAION_SAMPLE_TIMES     (50)  
#define         CALIBRATION_SAMPLE_INTERVAL  (500)  
#define         READ_SAMPLE_INTERVAL         (50)  
#define         READ_SAMPLE_TIMES            (5) 

#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

float           LPGCurve[3]  =  {2.3,0.21,-0.47};    

float           COCurve[3]  =  {2.3,0.72,-0.34};     

float           SmokeCurve[3] ={2.3,0.53,-0.44};     
float           Ro           =  10;           

#define SERVO_PIN 27
Servo servo;
int angle=0; 

const int flameSensorPinA = 4;  
const int flameThreshold=1800; 

const int motionSensorPin = 5; 

const int buzzerPin = 14;  

const int thermistorPin = 15; 


//dont use pin 34 & 35
const int redPin = 26;   
const int greenPin = 18; 
const int bluePin = 25;


const float R0 = 10000.0; 
const float x0 = 28.0;    
const float beta = 3950.0; 

float calculateTemperature(int rawADC) {
  float R = R0 * (1023.0 / rawADC - 1.0);
  float T = 1.0 / ((log(R / R0) / beta) + (1.0 / (x0 + 273.15)));
  T -= 273.15;
  return T;
}




void setup() {
  Serial.begin(115200);
  pinMode(flameSensorPinA, INPUT);

  pinMode(motionSensorPin,INPUT);

  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);
  Serial.print("Calibration is done...\n"); 

  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

  pinMode(thermistorPin,INPUT);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin,HIGH);

  servo.attach(SERVO_PIN);  

  pinMode(redPin, OUTPUT);        
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

}


void loop() 
{
  int flameSensorValueA = analogRead(flameSensorPinA);
  Serial.println("--------------------------- --------------------------------");
  Serial.println("");

  Serial.print("Analogous Value of flame sensor is ");
  Serial.println(4095-flameSensorValueA);
  Serial.println("");

  Serial.print("LPG:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
   Serial.print( "ppm    " );  
   Serial.print("CO:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
   Serial.print( "ppm    " );   
   Serial.print("SMOKE:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
   Serial.println( "ppm     " );
   Serial.println("");

  float rawADC = analogRead(thermistorPin);
  float temperature = calculateTemperature(rawADC); 
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C ,");
  Serial.println("");


  int motionSensorValue = digitalRead(motionSensorPin);
  
  if(( flameSensorValueA<flameThreshold))  //flame and smoke
  {
    if(motionSensorValue == HIGH)   // flame,smoke and a person inside
    {
      Serial.println("Motion Detected");
      protocol1();
    }
    else                            // flame,smoke but no one inside
    {
      Serial.println("No Motion Detected");
      protocol2();
    }
  }

  else if(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO)>35)   // to indicate toxic levels of carbon monoxide
  {
    protocol3();
  }
  
  else                                        // no flame or smoke,normal conditions
  {
    protocol4();
  }

  delay(2000);

}

void protocol1()
{
  Serial.println("In protocol 1");
  // sprinkleWater(1);
  triggerBuzzer(1);
  // personStuck(1);
  controlLED(2);
  // runFan(0);
  window(1);
}

void protocol2()
{

  Serial.println("In protocol 2");
  // sprinkleWater(1);
  triggerBuzzer(1);
  // personStuck(0);
  controlLED(2);
  // runFan(0);
  window(1);
}

void protocol3()
{
    Serial.println("In protocol 3");

  // sprinkleWater(0);
   triggerBuzzer(0);
  // personStuck(0);
  controlLED(1);
  // runFan(1);
  window(1);
}

void protocol4()
{
    Serial.println("In protocol 4");

  // sprinkleWater(0);
   triggerBuzzer(0);
  // personStuck(0);
  controlLED(0);
  // runFan(0);
  window(0);
}


  

void triggerBuzzer(int value) {
  if(value==1)
  {
  digitalWrite(buzzerPin, LOW); 
  Serial.println("Alarm triggered!");
  delay(250); 
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  }
  else
  {
    digitalWrite(buzzerPin,HIGH);
  }
}

//  void runFan(int value)
// {
//   if(value==1){
//    digitalWrite(fanPin, HIGH);
//     Serial.println("Turned on Fan");
//   }
//   else{
//     digitalWrite(fanPin, LOW);
//   }
// }

//  void sprinkleWater(int value)
// {
//   if(value==1){
//    digitalWrite(waterPin, HIGH);
//     Serial.println("Sprinkling Water");
//   }
//   else{
//      digitalWrite(waterPin, LOW);
//   }
// }

//  void personStuck(int value)
// {
//   if(value==1){
//    digitalWrite(rescuePin,HIGH);
//     Serial.println("Help! SOS Signal");
//   }
//   else{
//     digitalWrite(rescuePin,LOW);
//   }
// }

void window(int value)
{
  if(value==1)
  {
    angle=90;
    servo.write(angle); 
    Serial.println("Opening window");
  }
  else
  {
    angle=0;
    servo.write(angle);
    Serial.println("Window closed");
  }
}




  void controlLED(int value) {
  switch (value) {
    case 0: // Blue light
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin,LOW);
      digitalWrite(bluePin, HIGH);
      Serial.println("Glowing blue light");
      break;
    case 1: // Green light
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);
      Serial.println("Glowing green light");
      break;
    case 2: // Red light
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      Serial.println("Glowing red light");
      break;
    default: // Turn off all LEDs for unknown value
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      Serial.println("Glowing no light");
      break;
  }
}

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}


float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   

  val = val/RO_CLEAN_AIR_FACTOR;                         

  return val; 
}

float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}