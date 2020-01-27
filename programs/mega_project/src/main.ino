#define MOT0_A 32
#define MOT0_B 33
#define MOT0_P 5
#define ENC0_A 2
#define ENC0_B 22

#define MOT1_A 34
#define MOT1_B 35
#define MOT1_P 6
#define ENC1_A 3
#define ENC1_B 23


#define MOT2_A 36
#define MOT2_B 37
#define MOT2_P 7
#define ENC2_A 18
#define ENC2_B 24

#define LED0 9
#define LED1 10
#define LEDL 13
#define SW0 48
#define SW1 49


void setup(){
  pinMode(MOT0_A, OUTPUT);
  pinMode(MOT0_B, OUTPUT);
  pinMode(MOT0_P, OUTPUT);
  pinMode(ENC0_A, INPUT_PULLUP);
  pinMode(ENC0_B, INPUT_PULLUP);

  pinMode(MOT1_A, OUTPUT);
  pinMode(MOT1_B, OUTPUT);
  pinMode(MOT1_P, OUTPUT);
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);

  pinMode(MOT2_A, OUTPUT);
  pinMode(MOT2_B, OUTPUT);
  pinMode(MOT2_P, OUTPUT);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  digitalWrite(MOT0_A, LOW);
  digitalWrite(MOT0_B, LOW);
  digitalWrite(MOT1_A, LOW);
  digitalWrite(MOT1_B, LOW);
  digitalWrite(MOT2_A, LOW);
  digitalWrite(MOT2_B, LOW);


  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LEDL, OUTPUT);
  pinMode(SW0, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);

}

void set_all(int value){
  if(value>=0){
    digitalWrite(MOT0_A, HIGH);
    digitalWrite(MOT0_B, LOW);
    analogWrite(MOT0_P,value);  

    digitalWrite(MOT1_A, HIGH);
    digitalWrite(MOT1_B, LOW);
    analogWrite(MOT1_P,value);  

    digitalWrite(MOT2_A, HIGH);
    digitalWrite(MOT2_B, LOW);
    analogWrite(MOT2_P,value);  
  }
  else{
    digitalWrite(MOT0_A, LOW);
    digitalWrite(MOT0_B, HIGH);
    analogWrite(MOT0_P,-value);  

    digitalWrite(MOT1_A, LOW);
    digitalWrite(MOT1_B, HIGH);
    analogWrite(MOT1_P,-value);  

    digitalWrite(MOT2_A, LOW);
    digitalWrite(MOT2_B, HIGH);
    analogWrite(MOT2_P,-value);  
  }

}

void loop(){
  if(digitalRead(ENC0_A))digitalWrite(LED0, HIGH);
  else digitalWrite(LED0, LOW);
  if(digitalRead(ENC0_B))digitalWrite(LED1, HIGH);
  else digitalWrite(LED1, LOW);

  if(!digitalRead(SW0)){
    set_all(200);
  }
  else if(!digitalRead(SW1)){
    set_all(-200);
  }
  delay(1);
}
