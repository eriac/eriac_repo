#include <Servo.h>

Servo servo0;

void setup(){
  pinMode(48, INPUT_PULLUP);
  pinMode(49, INPUT_PULLUP);
  servo0.write(90);
  servo0.attach(40);
}

void loop(){

  if(!digitalRead(48)){
    servo0.write(110);
  }
  else if(!digitalRead(49)){
    servo0.write(70);
  }
  delay(50);
}
