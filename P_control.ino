#include <Servo.h>

Servo myservo2; //index
Servo myservo4; //middle
Servo myservo6; //ring
Servo myservo8; //pinky
Servo myservo10; //thumb

const int OUT_PIN0 = A0;
const int IN_PIN0 = A2;
const int OUT_PIN1 = A4;
const int IN_PIN1 = A7;
const float IN_STRAY_CAP_TO_GND = 24.48;
const float IN_CAP_TO_GND  = IN_STRAY_CAP_TO_GND;
const float R_PULLUP = 34.8;  
const int MAX_ADC_VALUE = 1023;
float capacitance0;
float capacitance1;
const float capacitance0bound = 100;
const float lowerbound = 300;
 int i=20;
 int fj; //pick up state
 const float m = 0.02;//p control coefficient
 const float p=(capacitance1-400);//p control
 double Ku = 0.5;
 double Tu = 1; 
double kp = 0.2*Ku;
double ki = Tu/2;
double kd = Tu/3;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setpoint, present_ang, change_ang;
double cumError, rateError;


void setup() {
  myservo2.attach(2);  

  myservo10.attach(10); 
  pinMode(OUT_PIN0, OUTPUT);
  pinMode(IN_PIN0, OUTPUT);
  pinMode(OUT_PIN1, OUTPUT);
  pinMode(IN_PIN1, OUTPUT);
  Serial.begin(9600);
  myservo10.write(90); 
  myservo2.write(80); 
  fj=0;
}

void loop(){
    setpoint = 300;
    pinMode(IN_PIN0, INPUT);
    pinMode(IN_PIN1, INPUT);
    digitalWrite(OUT_PIN0, HIGH);
    digitalWrite(OUT_PIN1, HIGH);
    int val0 = analogRead(IN_PIN0);
    int val1 = analogRead(IN_PIN1);
    digitalWrite(OUT_PIN0, LOW);
    digitalWrite(OUT_PIN1, LOW);
      if (val0 < 1000){
     pinMode(IN_PIN0, OUTPUT);
      capacitance0 = (float)val0 * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val0);
      // Serial.print(F("Capacitance Value = "));
      // Serial.print(capacitance, 3);
      // Serial.print(F(" pF ("));
      // Serial.print(val);
      // Serial.println(F(") "));
     }
      else{
      pinMode(IN_PIN0, OUTPUT);
      delay(1);
      pinMode(OUT_PIN0, INPUT_PULLUP);
      unsigned long u1 = micros();
      unsigned long t;
      int digVal;
      do{
        digVal = digitalRead(OUT_PIN0);
        unsigned long u2 = micros();
        t = u2 > u1 ? u2 - u1 : u1 - u2;
      } while ((digVal < 1) && (t < 400000L));
        pinMode(OUT_PIN0, INPUT);  
        val0 = analogRead(OUT_PIN0);
        digitalWrite(IN_PIN0, HIGH);
        int dischargeTime = (int)(t / 1000L) * 5;
        delay(dischargeTime);   
        pinMode(OUT_PIN0, OUTPUT);  
        digitalWrite(OUT_PIN0, LOW);
        digitalWrite(IN_PIN0, LOW);
      }
        if (val1 < 1000){
        pinMode(IN_PIN1, OUTPUT);
        capacitance1 = (float)val1 * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val1);
        //Serial.print(F("Capacitance1 Value = "));
        //Serial.print(capacitance1, 3);
        //Serial.print(F(" pF ("));
        //Serial.print(val1);
        //Serial.println(F(") "));
      }
        else{
        pinMode(IN_PIN1, OUTPUT);
        delay(1);
        pinMode(OUT_PIN1, INPUT_PULLUP);
        unsigned long u1 = micros();
        unsigned long t;
        int digVal1;
        do{
        digVal1 = digitalRead(OUT_PIN1);
        unsigned long u2 = micros();
        t = u2 > u1 ? u2 - u1 : u1 - u2;
        } while ((digVal1 < 1) && (t < 400000L));
        pinMode(OUT_PIN1, INPUT);  
        val1 = analogRead(OUT_PIN1);
        digitalWrite(IN_PIN1, HIGH);
        int dischargeTime = (int)(t / 1000L) * 5;
        delay(dischargeTime);   
        pinMode(OUT_PIN1, OUTPUT);  
        digitalWrite(OUT_PIN1, LOW);
        digitalWrite(IN_PIN1, LOW);
        }
        Serial.print(F(" Capacitance1 "));
        Serial.print(capacitance1, 3);
        Serial.print(F(" Capacitance0 "));
        Serial.print(capacitance0, 3);
        Serial.println(F(" "));
          
        
        if (capacitance0 < capacitance0bound && capacitance1 < lowerbound)
        {
            myservo2.write(i);   //stay
            fj = 0;
        }
        else if (capacitance0 < capacitance0bound && capacitance1 >= lowerbound) 
        {
            Serial.println(F(" error "));
            myservo2.write(i);   //return
            fj = 0;
        }
        else if (capacitance0 > capacitance0bound && capacitance1 < lowerbound)
        {
            
            present_ang=myservo2.read();// present angle
            change_ang = computePID(capacitance1);// output is angle and input is capacitance
            myservo2.write(change_ang+present_ang);
            delay(100);
            analogWrite(3, change_ang+present_ang); 
        }
        }
          
        
      
      double computePID(double cap){   
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = abs(setpoint- cap);                                // determine error
        cumError = error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
