#include <Servo.h>
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int escPin1 = 3;
int escPin2 = 5;
int escPin3 = 6;
int escPin4 = 9;

int minPulseRate = 1000;
int maxPulseRate = 2000;
int throttleChangeDelay = 200;

void setup() 
{  Serial.begin(9600);
   jerk_motors();
   start_motors();  
}

void loop() 
{   if(Serial.available() > 0) //means Blutooth is connected
   {    
        int Blue_value = Serial.read(); //values are alsways in ASCII
        if (Blue_value == 48)  //stop motors
        {    stop_motors();   }
        else if (Blue_value == 49)  //run motors
        {     run_motors();    } 
   }     
}

void jerk_motors ()
{    Serial.setTimeout(500);
  
     // Attach the the servo to the correct pin and set the pulse range
     esc1.attach(escPin1, minPulseRate, maxPulseRate); 
     esc2.attach(escPin2, minPulseRate, maxPulseRate); 
     esc3.attach(escPin3, minPulseRate, maxPulseRate); 
     esc4.attach(escPin4, minPulseRate, maxPulseRate); 
  
  // Write a minimum value (most ESCs require this correct startup)
     esc1.write(0);
     esc2.write(0);
     esc3.write(0);
     esc4.write(0);
       delay(2000);
    
     esc1.write(35);
     esc2.write(35);
     esc3.write(35);
     esc4.write(35);
        delay(1000);
      
     esc1.write(40);
     esc2.write(40);
     esc3.write(40);
     esc4.write(40);
        delay(1000);  
}


    
void start_motors()
{
   esc1.write(35);
   esc2.write(35);
   esc3.write(35);
   esc4.write(35);   //min threshold is 30;
      delay(6000);
    
   esc1.write(0);
   esc2.write(0);
   esc3.write(0);
   esc4.write(0); 
     delay(1000);
    
   esc1.write(40);
   esc2.write(40);
   esc3.write(40);
   esc4.write(40); 
      delay(6000);  
}

void stop_motors()
{           esc1.write(0);
            esc2.write(0);
            esc3.write(0);
            esc4.write(0);       }

void run_motors () 
{           esc1.write(100);
            esc2.write(100);
            esc3.write(100);
            esc4.write(100);     }


                         
 
  
