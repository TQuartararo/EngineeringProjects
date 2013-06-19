/*The PWM frequency that the Jaguar motor controllers take is equivalent to servo control
for the Arduino*/
#include <Servo.h>//include the servo library

//initiate the two servos 
Servo servoRight; 
Servo servoLeft;
int constantSpeed = 130;
int correction = 99;
int counter = 0;

//Zero motion occurs at Servo.write(99)

void setup(){
  Serial.begin(115200);
  servoRight.attach(3);
  servoLeft.attach(5);
}

void loop(){
   while (Serial.available()){
     correction = Serial.read();
     if (correction == 1)//kill the motors
     {
	servoLeft.write(99);
	servoRight.write(99);
     }
     else if (correction == 2)//slow down the motors if a beacon is not visible
     {
        servoLeft.write(constantSpeed - 20);
        servoRight.write(correction - 20);
        counter = counter + 1;
     }
     else if (correction == 3)
     {
	servoLeft.write(130);//tank turn
	servoRight.write(60);
	delay(1000);
	Serial.write(10);//signal to python that the turn is over
     }
     else{
       
     	servoLeft.write(constantSpeed);//drive the left wheel at a constant speed
     	servoRight.write(correction);//vary the speed of the right wheel to make straight line
        counter = 0;//reset the counter if a beacon has been spotted  
     }
    }

}
