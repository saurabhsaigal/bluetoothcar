  int enA = 3;       // Pin  ENA of L298
  int motorA1  = 5;  // Pin  IN1 of L298
  int motorA2  = 6;  // Pin  IN2 of L298
  int enB = 11;      // Pin  ENB of L298
  int motorB1  = 9;  // Pin IN3 of L298
  int motorB2  = 10;  // Pin IN4 of L298
//Leds connected to Arduino UNO Pin 12
  int lights  = 13;
//Bluetooth (HC-05 JY-MCU) State pin on pin 2 of Arduino
  int BTState = 2;
  int state;
  int vSpeed=255;     // Default speed, from 0 to 255
void setup() {
    // Set pins as outputs:
    Serial.println("working");
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(lights, OUTPUT);
    pinMode(BTState, INPUT);  
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);
    // Initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
}
 
void loop() { 
  //Stop car when connectiont or bluetooth disconnected
     if(digitalRead(BTState)==LOW) { state='S'; }

  //Save income data to variable 'state'
    if(Serial.available() > 0){     
      state = Serial.read();   
    }
  
  //Change speed if state is equal from 0 to 4. Values must be from 0 to 255 (PWM)
 if (state == '0'){
      vSpeed=0;}
    else if (state == '1'){
      vSpeed=40;}
    else if (state == '2'){
      vSpeed=60;}
    else if (state == '3'){
      vSpeed=80;}
    else if (state == '4'){
      vSpeed=100;}
    else if (state == '5'){
      vSpeed=125;}
    else if (state == '6'){
      vSpeed=150;}
    else if (state == '7'){
      vSpeed=175;}
    else if (state == '8'){
      vSpeed=200;}
    else if (state == '9'){
      vSpeed=255;}
     
  /***********************Forward****************************/
  //If state is equal with letter 'F', car will go forward!
    if (state == 'F') {
      analogWrite(motorA1, vSpeed); analogWrite(motorB2, 0);
        analogWrite(motorB1, vSpeed);  analogWrite(motorA2, 0);
               }
  /**********************Forward Left************************/
  //If state is equal with letter 'G', car will go forward left
    else if (state == 'G') {
      analogWrite(motorA1, vSpeed); analogWrite(motorA2, 0);  
        analogWrite(motorB1, 0);    analogWrite(motorB2, 0); 
   }
  /**********************Forward Right************************/
  //If state is equal with letter 'I', car will go forward right
    else if (state == 'I') {
        analogWrite(motorA1, 0); analogWrite(motorA2, 0); 
        analogWrite(motorB1, vSpeed);      analogWrite(motorB2, 0); 
    }
  /***********************Backward****************************/
  //If state is equal with letter 'B', car will go backward
    else if (state == 'B') {
      analogWrite(motorA1, 0);   analogWrite(motorA2, vSpeed); 
        analogWrite(motorB1, 0);   analogWrite(motorB2, vSpeed); 
    }
  /**********************Backward Left************************/
  //If state is equal with letter 'H', car will go backward left
    else if (state == 'J') {  
        analogWrite(motorA1, 0);   analogWrite(motorA2, 0); 
        analogWrite(motorB1, 0);   analogWrite(motorB2, vSpeed); 
    }
  /**********************Backward Right************************/
  //If state is equal with letter 'J', car will go backward right
    else if (state == 'H') {
        analogWrite(motorA1, 0);   analogWrite(motorA2,vSpeed); 
        analogWrite(motorB1, 0); analogWrite(motorB2, 0); 
   }
  /***************************Left*****************************/
  //If state is equal with letter 'L', wheels will turn left
    else if (state == 'L') {
      analogWrite(motorA1, vSpeed);   analogWrite(motorA2, 0); 
        analogWrite(motorB1, 0); analogWrite(motorB2, vSpeed); 
 }
  /***************************Right*****************************/
  //If state is equal with letter 'R', wheels will turn right
    else if (state == 'R') {
      analogWrite(motorA1, 0);   analogWrite(motorA2, vSpeed); 
        analogWrite(motorB1, vSpeed);   analogWrite(motorB2, 0);     
   }
  /************************Lights*****************************/
  //If state is equal with letter 'W', turn leds on or of off
    else if (state == 'W') {
      digitalWrite(lights,HIGH);
    }
    else if(state=='w')
      { digitalWrite(lights,LOW); }
  /************************Stop*****************************/
  //If state is equal with letter 'S', stop the car
    else if (state == 'S'){
        analogWrite(motorA1, 0);  analogWrite(motorA2, 0); 
        analogWrite(motorB1, 0);  analogWrite(motorB2, 0);
    }  
}
