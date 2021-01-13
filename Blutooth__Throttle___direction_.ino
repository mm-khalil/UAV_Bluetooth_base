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

int change_value   = 110;
int constant_value = 80;

void setup() 
{  Serial.begin(9600);
   jerk_motors();
   start_motors();  
}

void loop() 
{   if(Serial.available() > 0) //means Blutooth is connected
   {    
        int Blue_value = Serial.read(); //values are alsways in ASCII
        
        
        if (Blue_value == 48)  //48=0, stop motors
        {    stop_motors();   }
        else if (Blue_value == 49)  //49=1, forward move
        {     forward();    } 
        else if (Blue_value == 50)  //50=2, reverse move
        {     reverse();    }
        else if (Blue_value == 51)  //51=3, right move
        {     move_right();    }
        else if (Blue_value == 52)  //52=4, left move
        {     move_left();    }

       
       
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

void forward () 
{           esc1.write(change_value);
            esc2.write(change_value);
            esc3.write(constant_value);
            esc4.write(constant_value);
            //   Serial.println("forward");
            }

void reverse () 
{           esc1.write(constant_value);
            esc2.write(constant_value);
            esc3.write(change_value);
            esc4.write(change_value);
            //Serial.println("reverse");}

void move_left () 
{           esc1.write(change_value);
            esc2.write(constant_value);
            esc3.write(change_value);
            esc4.write(constant_value);
            //Serial.println("moving left");
            }

void move_right () 
{           esc1.write(constant_value);
            esc2.write(change_value);
            esc3.write(constant_value);
            esc4.write(change_value);
            //Serial.println("moving right ");
            }
                         
 
  
