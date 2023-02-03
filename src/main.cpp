#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#include "math.h"
//-------------------------------------------------------------------------------------------------------//
#define _PWM_LOGLEVEL_        3
#include "RP2040_PWM.h"

//#define pin0    25    // PWM channel 4B, BUILTIN_LED
#define servo1    0   // PWM channel 0A
#define servo2    1     // PWM channel 1A
#define servo5    2     // PWM channel 2A
#define servo6    3     // PWM channel 3A
#define servo7    6     // PWM channel 0A
#define servo8    7    // PWM channel 5A
#define servo9    8    // PWM channel 6A
#define servo10   10    // PWM channel 7A
#define echoPin   16
#define trigPin   17
#define thresholdmin 7
#define thresholdmax 100


uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 };

#define NUM_OF_PINS       ( sizeof(PWM_Pins) / sizeof(uint32_t) )

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

/*
// Input variables
uint8_t S1, prevS1, RE_S1;
uint8_t S2, prevS2, RE_S2;*/

uint32_t duration, distanceInterrupt;
uint32_t dist;
int distanceReady;

/*
// Output variables

// Our finite state machines
fsm_t fsm1, fsm2, fsm3, fsm4, fsm5, fsm6, fsm7, fsm8, fsm9, fsm10, fsm11;
*/

fsm_t creep_fw, rotate_left,rotate_right, sonar_core1,main_controller;

unsigned long interval, last_cycle = 0;
unsigned long loop_micros;

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

float dtc[NUM_OF_PINS];

//creates pwm instance
RP2040_PWM* PWM_Instance[NUM_OF_PINS];
//creates accel instance

// calculates pwm necessary for each different servo
float calcDutyCycle(int servo, float angle){
  float dc;

  switch(servo){

    case servo1:
      dc = 9.4/180.0*angle + 3.2;
      return dc;
    case servo2:
      dc = 10/180.0*angle + 3;
      return dc;
    case servo5:
      dc = 9.6/180.0*angle + 2.7;
      return dc;
    case servo6:
      dc = 9.6/180.0*angle + 2.8;
      return dc;
    case servo7:
      dc = 10.0/180.0*angle + 3.2;
      return dc;
    case servo8:
      dc = 10/180.0*(180-angle) + 3.4;
      return dc;
    case servo9:
      dc=9.4/180.0*(180-angle) + 2.6;
      return dc;
    case servo10:
      dc = 9.4/180.0*angle + 2.9;
      return dc;

  }
  return 3.0;
}
//variables for interrupt(sonar)
uint32_t echo_end,trig_beg;
int event_trig = 0;
int random_int = 0;//to determine if robot turns left(0) or right(1) when obstacle is detected

void rot_l(){
  if(rotate_left.state == 0 && rotate_left.tis > 100){
   
    rotate_left.new_state = 1;
  }
  else if(rotate_left.state == 1 && rotate_left.tis > 100){
   
    rotate_left.new_state = 2;
  }
  else if(rotate_left.state == 2 && rotate_left.tis > 100){
   
    rotate_left.new_state = 3;
  }
  else if(rotate_left.state == 3 && rotate_left.tis > 100){

    rotate_left.new_state = 4;
  }
  else if(rotate_left.state == 4 && rotate_left.tis > 100){

    rotate_left.new_state = 0;
  }else if(rotate_left.state == 10 && creep_fw.state == 50 && ((main_controller.state == 2 && random_int == 0) ) ){
    rotate_left.new_state = 0;
  }else if(rotate_left.state >= 0 && (main_controller.state != 2)){
    rotate_left.new_state = 10;
  }
  
}

void rot_r(){
   if(rotate_right.state == 0 && rotate_right.tis > 100){
   
    rotate_right.new_state = 1;
  }
  else if(rotate_right.state == 1 && rotate_right.tis > 100){
   
    rotate_right.new_state = 2;
  }
  else if(rotate_right.state == 2 && rotate_right.tis > 100){
   
    rotate_right.new_state = 3;
  }
  else if(rotate_right.state == 3 && rotate_right.tis > 100){

    rotate_right.new_state = 4;
  }
  else if(rotate_right.state == 4 && rotate_right.tis > 100){

    rotate_right.new_state = 0;
  }else if(rotate_right.state == 10  && creep_fw.state == 50 && ( (main_controller.state == 2 && random_int == 1))){

    rotate_right.new_state = 0;
  }else if(rotate_right.state >= 0 && (main_controller.state != 2) ){

    rotate_right.new_state = 10;
  }
}

void creep_forward(){
  int temp = 100;
  /*unsigned long cur_time = millis();
  creep_fw.tis = cur_time - creep_fw.tes;*/

  //Serial.println(creep_fw.new_state);
  //Serial.println(creep_fw.tis);

  if(creep_fw.state == 0 && creep_fw.tis > temp && main_controller.state == 0){
    creep_fw.new_state = 1;
  }
  else if(creep_fw.state == 1 && creep_fw.tis > temp  && main_controller.state == 0){
    creep_fw.new_state = 2;
  }
  else if(creep_fw.state == 2 && creep_fw.tis > temp && main_controller.state == 0){
    creep_fw.new_state = 3;
  }
  else if(creep_fw.state == 3 && creep_fw.tis > temp && main_controller.state == 0){
    creep_fw.new_state = 4;
  }
  else if(creep_fw.state == 4 && creep_fw.tis > temp && main_controller.state == 0){
    creep_fw.new_state = 5;
  }
  else if(creep_fw.state == 5 && creep_fw.tis > temp && main_controller.state == 0){
    creep_fw.new_state = 6;
  }
  else if(creep_fw.state == 6 && creep_fw.tis > temp && main_controller.state == 0){
    creep_fw.new_state = 0;
  }
  else if(main_controller.state != 0){ // if the robot is balancing/turning/adjusting, it cant be moving
      creep_fw.new_state = 50;
  }else if(creep_fw.state == 50 && main_controller.state == 0){
      creep_fw.new_state = 0;
  }

  /*else if(creep_fw.state == 50 && control.state == 0){
      creep_fw.new_state = 0;
  }*/
 
  //set_state(creep_fw, creep_fw.new_state);

}

//comparations of distanceInterrupt w/ 0 are because sometimes the sensor bugs and reads 0
void m_cont(){

  if(main_controller.state == 0 && dist <= 16 && dist > 2){ // when state = 0, walks forward(creep_backward)
    main_controller.new_state = 5; //go back

    randomSeed(millis()); //decide if turns left or right
    random_int = random(2);
  }else if(main_controller.state == 5 && main_controller.tis > 200 && dist <= 16 && dist > 2){
    main_controller.new_state = 1;
  }
  else if(main_controller.state == 5 && main_controller.tis > 200 && dist > 30){
    main_controller.new_state = 0;
  }
  else if(main_controller.state == 1 && main_controller.tis > 500){
    main_controller.new_state = 2; // turn left or right

  }else if(main_controller.state == 2 && main_controller.tis > 1500 && dist > 30){ //when state = 1, turns left or right
    main_controller.new_state = 0; //after turning, if there's space in front, go back to walk
  }
  else if(main_controller.state == 50){
      main_controller.new_state = 50;
  }
  
  //in state 0 does nothing, in state 1 the robot should be moving backwards(creep_forward), in state 2 the robot is turning for a minimum of 600ms
}

int cont = 0;
int cont2 = 0;



float freq = 50;
int wiggleval;
double thetaP, thetaR;


float dt;
void setup()
{
  interval = 10;
  
  pinMode(27,OUTPUT);
 
  for (uint8_t index = 0; index < 8; index++)
  {
    PWM_Instance[index] = new RP2040_PWM(PWM_Pins[index], freq, dtc[index]);
  }

  set_state(creep_fw, 0); //0 to walk, 50 to turn off
  set_state(rotate_left,10);  
  set_state(sonar_core1,0);
  set_state(rotate_right,10);
  set_state(main_controller,0); //controls the obstacle avoidance, 0 to enable 50 to disable

  //attachInterrupt(echoPin,echo_rising_edge,CHANGE);  
 
}


void echoInterrupt() {
  
   if (digitalRead(echoPin) == LOW) {
    // Pulse is low, calculate the duration of the pulse
    duration = micros() - duration;

    // Calculate the distance based on the duration of the pulse and the speed of sound
    distanceInterrupt = duration * 0.034 / 2 ;
    
    if(distanceInterrupt >= thresholdmax){
      distanceInterrupt = 40;
    }
    else if (distanceInterrupt <= thresholdmin){
      distanceInterrupt = 20;
    }
    // Distance is updated
    distanceReady = 1;
  } else {
    // Pulse is high, save the current time
    duration = micros();
  }
  
}




//setup for second core

void setup1(){

  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  delay(800);

  attachInterrupt(digitalPinToInterrupt(echoPin),echoInterrupt,CHANGE);

}


int flag = 0;

//loop for second core
void loop1(){

  
  static unsigned long last_cycle1 = 0;

  unsigned long curr_time1 = millis();
  if (curr_time1 - last_cycle1 >= 50){

    
    if (distanceReady) {
      // Send the distance to Core0 using the FIFO when distance is updated
      rp2040.fifo.push_nb(distanceInterrupt);

      // Clear the distanceReady flag
      distanceReady = false;
    }
    
   
    // Send a pulse to the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    last_cycle1 = curr_time1;
  }

    
    
}



void loop()
{
  unsigned long now = millis();
  

  if(now - last_cycle > interval){

  
    //update inputs
    
    //this function does not change the value in distanceInterrupt if fifo is empty, and never blocks the execution of the program
    rp2040.fifo.pop_nb(&dist);

    dt = now - last_cycle;
  
    unsigned long cur_time = millis();
    creep_fw.tis = cur_time - creep_fw.tes;
    rotate_left.tis = cur_time - rotate_left.tes;
    sonar_core1.tis = cur_time - sonar_core1.tes;
    rotate_right.tis = cur_time - rotate_right.tes;
    main_controller.tis = cur_time - main_controller.tes;
    
    
    //uint32_t PWM_Pins[]       = { servo1, servo2, servo5, servo6, servo7, servo8, servo9, servo10 }; para ver quais servos em que pinos
    //servo 9 together w 10, servo6 w 8
    //servo testing



    m_cont();
    creep_forward();
    rot_r();
    rot_l();
    
    if(sonar_core1.state == 0 && sonar_core1.tis > 50){
      sonar_core1.new_state = 1;
    }else if(sonar_core1.state == 1 && sonar_core1.tis > 5){
      sonar_core1.new_state = 0;
    }

    set_state(creep_fw,creep_fw.new_state);
    set_state(rotate_left,rotate_left.new_state);
    set_state(sonar_core1,sonar_core1.new_state);
    set_state(rotate_right,rotate_right.new_state);
    set_state(main_controller,main_controller.new_state);

 
    if(sonar_core1.state == 0){
      flag = 0;
    }else if(sonar_core1.state == 1){
      flag = 1;
    }

    //creep_forward outputs
    if(creep_fw.state == 0){
    dtc[0] = calcDutyCycle(servo1, 50);
    dtc[5] = calcDutyCycle(servo8, 50);
    dtc[6] = calcDutyCycle(servo9, 90);
    dtc[1] = calcDutyCycle(servo2, 90);
    dtc[4] = calcDutyCycle(servo7, 0);
    
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);

  }
  else if(creep_fw.state == 1){
    dtc[7] = calcDutyCycle(servo10, 30);
    dtc[6] = calcDutyCycle(servo9, 10);
    dtc[0] = calcDutyCycle(servo1, 50);
    dtc[1] = calcDutyCycle(servo2, 90);
    dtc[5] = calcDutyCycle(servo8, 50);
    
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(creep_fw.state == 2){
    dtc[7] = calcDutyCycle(servo10, 0);
    dtc[6] = calcDutyCycle(servo9, 50);
    dtc[0] = calcDutyCycle(servo1, 90);
    dtc[5] = calcDutyCycle(servo8, 10);
    dtc[1] = calcDutyCycle(servo2, 50);

    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  
  }
  else if(creep_fw.state == 3){
    dtc[5] = calcDutyCycle(servo8, 90);
    dtc[6] = calcDutyCycle(servo9, 50);
    dtc[0] = calcDutyCycle(servo1, 90);
    dtc[1] = calcDutyCycle(servo2, 50);
    dtc[3] = calcDutyCycle(servo6, 30);

    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(creep_fw.state == 4){
    dtc[3] = calcDutyCycle(servo6, 0);
    dtc[0] = calcDutyCycle(servo1, 10);
    dtc[2] = calcDutyCycle(servo5, 30);
    dtc[5] = calcDutyCycle(servo8, 90);
    dtc[6] = calcDutyCycle(servo9, 50);
    dtc[1] = calcDutyCycle(servo2, 50);

    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    //delay(40);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(creep_fw.state == 5){
    dtc[2] = calcDutyCycle(servo5, 0);
    dtc[0] = calcDutyCycle(servo1, 50);
    dtc[6] = calcDutyCycle(servo9, 90);
    dtc[5] = calcDutyCycle(servo8, 50);
    dtc[1] = calcDutyCycle(servo2, 10);

    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(creep_fw.state == 6){
    dtc[4] = calcDutyCycle(servo7, 30);
    dtc[1] = calcDutyCycle(servo2, 90);
    dtc[0] = calcDutyCycle(servo1, 50);
    dtc[6] = calcDutyCycle(servo9, 90);
    dtc[5] = calcDutyCycle(servo8, 50);

    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
     
  //rotate left outputs

  if(rotate_left.state == 0){
    dtc[7] = calcDutyCycle(servo10,0);
    dtc[6] = calcDutyCycle(servo9,45);  
    dtc[3] = calcDutyCycle(servo6,0);  
    dtc[5] = calcDutyCycle(servo8,45);  
    dtc[0] = calcDutyCycle(servo1,45);
    dtc[2] = calcDutyCycle(servo5,0);
    dtc[1] = calcDutyCycle(servo2,45);
    dtc[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_left.state == 1){
    dtc[7] = calcDutyCycle(servo10,0);
    dtc[6] = calcDutyCycle(servo9,70);  //leg2 bw  
    dtc[3] = calcDutyCycle(servo6,0);  
    dtc[5] = calcDutyCycle(servo8,70);  //leg3 fw  
    dtc[0] = calcDutyCycle(servo1,70);  //leg1 bw
    dtc[2] = calcDutyCycle(servo5,20);  //leg1 up
    dtc[1] = calcDutyCycle(servo2,70);  //leg4 fw
    dtc[4] = calcDutyCycle(servo7,20);  //leg4 up
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_left.state == 2){
    dtc[7] = calcDutyCycle(servo10,0);
    dtc[6] = calcDutyCycle(servo9,70);    
    dtc[3] = calcDutyCycle(servo6,0);  
    dtc[5] = calcDutyCycle(servo8,70);  
    dtc[0] = calcDutyCycle(servo1,70);  
    dtc[2] = calcDutyCycle(servo5,0);   //leg1 dn
    dtc[1] = calcDutyCycle(servo2,70);  
    dtc[4] = calcDutyCycle(servo7,0);   //leg4 dn
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_left.state == 3){
    dtc[7] = calcDutyCycle(servo10,20); //leg2 up
    dtc[6] = calcDutyCycle(servo9,20);  //leg2 fw  
    dtc[3] = calcDutyCycle(servo6,20);  //leg3 up  
    dtc[5] = calcDutyCycle(servo8,20);  //leg3 bw
    dtc[0] = calcDutyCycle(servo1,20);  //leg1 fw
    dtc[2] = calcDutyCycle(servo5,0);
    dtc[1] = calcDutyCycle(servo2,20);  //leg4 bw
    dtc[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_left.state == 4){
    dtc[7] = calcDutyCycle(servo10,0); //leg2 dn
    dtc[6] = calcDutyCycle(servo9,20);  
    dtc[3] = calcDutyCycle(servo6,0);  //leg3 dn  
    dtc[5] = calcDutyCycle(servo8,20);  
    dtc[0] = calcDutyCycle(servo1,20);  
    dtc[2] = calcDutyCycle(servo5,0);
    dtc[1] = calcDutyCycle(servo2,20);  
    dtc[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }

  //rotate right
  if(rotate_right.state == 0){
    dtc[7] = calcDutyCycle(servo10,0);
    dtc[6] = calcDutyCycle(servo9,45);  
    dtc[3] = calcDutyCycle(servo6,0);  
    dtc[5] = calcDutyCycle(servo8,45);  
    dtc[0] = calcDutyCycle(servo1,45);
    dtc[2] = calcDutyCycle(servo5,0);
    dtc[1] = calcDutyCycle(servo2,45);
    dtc[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_right.state == 1){
    dtc[7] = calcDutyCycle(servo10,20); //leg2 up
    dtc[6] = calcDutyCycle(servo9,70);  //leg2 bw
    dtc[3] = calcDutyCycle(servo6,20);  //leg3 up  
    dtc[5] = calcDutyCycle(servo8,70);  //leg3 fw
    dtc[0] = calcDutyCycle(servo1,70);  //leg1 bw
    dtc[2] = calcDutyCycle(servo5,0);
    dtc[1] = calcDutyCycle(servo2,70);  //leg4 fw
    dtc[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_right.state == 2){
    dtc[7] = calcDutyCycle(servo10,0);  //leg2 dn
    dtc[6] = calcDutyCycle(servo9,70);  
    dtc[3] = calcDutyCycle(servo6,0);   //leg3 dn
    dtc[5] = calcDutyCycle(servo8,70);  
    dtc[0] = calcDutyCycle(servo1,70);  
    dtc[2] = calcDutyCycle(servo5,0);
    dtc[1] = calcDutyCycle(servo2,70);  
    dtc[4] = calcDutyCycle(servo7,0);
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_right.state == 3){
    dtc[7] = calcDutyCycle(servo10,0);  
    dtc[6] = calcDutyCycle(servo9,20);  //leg2 fw
    dtc[3] = calcDutyCycle(servo6,0);  
    dtc[5] = calcDutyCycle(servo8,20);  //leg3 bw
    dtc[0] = calcDutyCycle(servo1,20);  //leg1 fw
    dtc[2] = calcDutyCycle(servo5,20);  //leg1 up
    dtc[1] = calcDutyCycle(servo2,20);  //leg4 bw
    dtc[4] = calcDutyCycle(servo7,20);  //leg4 up
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
  else if(rotate_right.state == 4){
    dtc[7] = calcDutyCycle(servo10,0);  
    dtc[6] = calcDutyCycle(servo9,20);  
    dtc[3] = calcDutyCycle(servo6,0);  
    dtc[5] = calcDutyCycle(servo8,20);  
    dtc[0] = calcDutyCycle(servo1,20);  
    dtc[2] = calcDutyCycle(servo5,0);  //leg1 dn
    dtc[1] = calcDutyCycle(servo2,20);  
    dtc[4] = calcDutyCycle(servo7,0);  //leg4 dn
    PWM_Instance[4]->setPWM(servo7, freq, dtc[4]);
    PWM_Instance[6]->setPWM(servo9, freq, dtc[6]);
    PWM_Instance[7]->setPWM(servo10, freq, dtc[7]);
    PWM_Instance[5]->setPWM(servo8, freq, dtc[5]);
    PWM_Instance[3]->setPWM(servo6, freq, dtc[3]);
    PWM_Instance[0]->setPWM(servo1, freq, dtc[0]);
    PWM_Instance[2]->setPWM(servo5, freq, dtc[2]);
    PWM_Instance[1]->setPWM(servo2, freq, dtc[1]);
  }
   
  }

}



