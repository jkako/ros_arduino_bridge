/*
 * Testbed for speed controller of Robot
 * More Details and Credits see: Robo_base_controller
 * Step 1: measure speed + Set PWM + PID * 2
 * Done:Speed from Twist incl calc & FWD/Back discrim
 *      Motor Pin inits defined
 *      This version shall apply Quadrature frequency & position measurement (instead of quad pos only or freq only) 
 *      Besides int. PID and direct Twist msg 
 *      Rework msg handling (reporting)
 *      accel/decel setpoint modification
 *      ==>Evolution to integrate into ros-arduino-bridge instead of rosserial
 */
 
 // Frequency counter sketch, for measuring frequencies low enough to execute an interrupt for each cycle
// Connect the frequency source to the INT0 pin (digital pin 2 on an Arduino Uno)
// From https://forum.arduino.cc/index.php?topic=64219.msg2182522#msg2182522

#ifdef MONSTER_MOTO_SHIELD
/*
void roverCallBack(const geometry_msgs::Twist& cmd_vel)
{
  //Abstand 220mm D=50mm Getriebe 1:75
  
  SetpointVM0 = (cmd_vel.angular.z*WHEEL_DIST)/2 + cmd_vel.linear.x;
  SetpointVM1 = -1*(cmd_vel.angular.z*WHEEL_DIST)/2 + cmd_vel.linear.x;
  //SetpointVM1 = cmd_vel.linear.x*2-SetpointVM0;
  
  SetpointVM0 = SetpointVM0 * STEP_P_M; //  0.000218166156; //cmd_vel.linear.x * 127 ;
  SetpointVM1 = SetpointVM1 * STEP_P_M; //  0.000218166156; //cmd_vel.angular.z * 127 ;
  
  if(SetpointVM0 < 0)
  {
    SetpointVM0=SetpointVM0 * -1;
    MotorFWD(0,false);
  }
  else
  {
    MotorFWD(0,true);
  }
  if(SetpointVM1 < 0)
  {
    SetpointVM1=SetpointVM1 * -1;
    MotorFWD(1,false);
  }
  else
  {
    MotorFWD(1,true);
  }
  if(SetpointVM0 < 10)
  {
    MotorEn(0,false);
    SetpointVM0 = 0;
  }
    else
  {
    MotorEn(0,true);
  }
  if(SetpointVM1 < 10) //to0 small vel
  {
    MotorEn(1,false);
    SetpointVM1 = 0;
  }
  else
  {
    MotorEn(1,true);
  }
  //vel1_msg.data=SetpointVM1;
  //chatterV1.publish(&vel1_msg);
}
*/



void set_Headlight(uint8_t level)
  {
  
  if(level == 0)
  {
    digitalWrite(headlightPin,LOW);
  }
  else if (level == 255)
  {
    digitalWrite(headlightPin,HIGH);
  }
  else
  {
    analogWrite(headlightPin,level);
  }
  }
  

void EncoderInit()
{
    EncInit(encoderPin0A);
    EncInit(encoderPin1A);
    EncInit(encoderPin0B);
    EncInit(encoderPin1B);

  attachInterrupt(digitalPinToInterrupt(encoderPin0A),doEncoder0A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin0B),doEncoder0B,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1A),doEncoder1A,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1B),doEncoder1B,CHANGE);
}

// Interrupt on A changing state
void doEncoder0A(){
  // Test transition
  A0Set = digitalRead(encoderPin0A) == HIGH;
  // and adjust counter + if A leads B
  encoderPos0+= (A0Set != B0Set) ? +1 : -1;
  lastPulseTime0 = micros();//currently no timekeeping implemented
}

// Interrupt on B changing state
void doEncoder0B(){
  // Test transition
  B0Set = digitalRead(encoderPin0B) == HIGH;
  // and adjust counter + if B follows A
  encoderPos0 += (A0Set == B0Set) ? +1 : -1;
  lastPulseTime0 = micros();//currently no timekeeping implemented
}

// Interrupt on A changing state
void doEncoder1A(){
  // Test transition
  A1Set = digitalRead(encoderPin1A) == HIGH;
  // and adjust counter + if A leads B
  encoderPos1+= (A1Set != B1Set) ? +1 : -1;
  lastPulseTime1 = micros();//currently no timekeeping implemented
}

// Interrupt on B changing state
void doEncoder1B(){
  // Test transition
  B1Set = digitalRead(encoderPin1B) == HIGH;
  // and adjust counter + if B follows A
  encoderPos1 += (A1Set == B1Set) ? +1 : -1;
  lastPulseTime1 = micros(); //currently no timekeeping implemented
}

/*
 * 
 void setup()
{
    HeadlightInit();
    MotorInit(0);
    MotorInit(1);

}
*/
#undef NOT_DEFINED
#ifdef NOT_DEFINED 
void loop()
{
  currentMilis = millis();

  if ((currentMilis - lastMilis) > CNTRL_FREQ) // instead of using delay(1000) (Modify interval for lower freq e.g. 2000 instead of 1000) (reduced update intervall ... first try 500) ansonsten alternative interrupts um mehr pulse zu bekommen
    {
      lastMilis = currentMilis;
      /*
       * 
       * Scheme:
       * take stepcount
       * take timer
       * do calc with last values
       * 
       */
       
       unsigned long curPulseTime1 = lastPulseTime1;
       
       int curencoderPos1 = encoderPos1;//sequence to reduce issues with Iterrupt (if Interrupt fires in between first copy we still get last count and correct time unless higher bits are overflowing)
       
       if((curencoderPos1-prevencoderPos1) == 0)
        {
          Hz1=0;
        }
        else
       {
         Hz1 = 1000000.0 * (float)(curencoderPos1-prevencoderPos1) / (float)(prevPulseTime1 - curPulseTime1);
         prevencoderPos1 = curencoderPos1;
         prevPulseTime1 = curPulseTime1;
       }    
      //new routine for freq calculation w/o sei/cli ... with quadrature encoding no pin chang shall be lost instead live with the risk of small errors --> change to 8bit counters maybe later
 /*      unsigned long curPulseTime0 = lastPulseTime0;
       int curencoderPos0 = encoderPos0;//sequence to reduce issues with Iterrupt (if Interrupt fires in between first copy we still get last count and correct time unless higher bits are overflowing)
       if((curencoderPos0-prevencoderPos0) == 0)
        {
          Hz0=0;
        }
        else
       {
       Hz0 = 1000000.0 * (float)(curencoderPos0-prevencoderPos0) / (float)(prevPulseTime0 - curPulseTime0);
       prevencoderPos0 = curencoderPos0;
       prevPulseTime0 = curPulseTime0;
       } 
    if(Hz0<0)
    {
      InputVM0 = Hz0 * -1;
    }else
    {
    InputVM0 = Hz0;  
    }
    */
    
    if(Hz1<0)
    {
      InputVM1 = Hz1 * -1;
    }
     else
    {
      InputVM1 = Hz1;  
    }
    Hz1_msg.data=Hz1;
    chatterls.publish( &Hz1_msg);
    nh.spinOnce();
    myPID_Vel_M0.Compute();
    myPID_Vel_M1.Compute();
    SetMotorPWM(5,(int)OutputVM0);
    SetMotorPWM(6,(int)OutputVM1);
    }
}

#endif

int ReportLastDiff(uint8_t N)
{
  if(N==LEFT)
  {
   unsigned long prevPulseTime = lastPulseTime0;
   while(prevPulseTime==lastPulseTime0)
   {
     
   }
   return lastPulseTime0 - prevPulseTime;
  }
  else if (N==RIGHT)
  {
   unsigned long prevPulseTime = lastPulseTime1;
   while(prevPulseTime==lastPulseTime1)
   {
     
   }
   return lastPulseTime1 - prevPulseTime;    
  }
}

void HeadlightInit()
{
    pinMode(headlightPin, OUTPUT);
    digitalWrite(headlightPin,LOW);
}

void EncInit(int EncPin)
{
  pinMode(EncPin, INPUT_PULLUP); 
}

void MotorInit(int MotorN)
{
pinMode(enpin[MotorN],OUTPUT);
pinMode(inApin[MotorN],OUTPUT);
pinMode(inBpin[MotorN],OUTPUT);
InitMotorPWM(MotorN,PWMCNT);

}

void MotorFWD(int MotorN, bool FWD)
{
  if(FWD)
  {
    MotorEn(MotorN, false);
    digitalWrite(inApin[MotorN],LOW);
    digitalWrite(inBpin[MotorN],HIGH);
    MotorEn(MotorN, true);
  }
  else
  {
    MotorEn(MotorN, false);
    digitalWrite(inBpin[MotorN],LOW);
    digitalWrite(inApin[MotorN],HIGH);
    MotorEn(MotorN, true);
  }
  
}

void MotorEn(int MotorN, bool EN)
{
  if(EN)
  {
    digitalWrite(enpin[MotorN],HIGH);
  }
  else
  {
    digitalWrite(enpin[MotorN],LOW);
    SetMotorPWM(pwm_pin[MotorN],0);
  }
  
}

void InitMotorPWM(int MotorN, int PWM_max) 
{


if(MotorN==0)//AVR Pin 5,T/C3,OC3A
  {
    pinMode(5, OUTPUT);
    //CS32:0=1(System Clock)
    TCCR3A=(1<<COM3A1)|(0<<COM3A0)|(0<<COM3B1)|(0<<COM3B0)|(0<<COM3C1)|(0<<COM3C0)|(1<<WGM31)|(0<<WGM30); //WGM 3,2,1=1 WGM0=0 := Mode 14 Fast PWM 
    TCCR3B=(0<<ICNC3)|(0<<ICES3)|(1<<WGM33)|(1<<WGM32)|(0<<CS32)|(0<<CS31)|(1<<CS30); // prescaler = 1 (CS:001)
    ICR3=PWM_max;//8192; //define top ==> ~4khz PWM freq (16000000/(1*4096))
  //  TCCR3C Not for PWM Operation
  }

if(MotorN==1)//AVR Pin 6,T/C4,OC4A TODO Check Datasheet if this is correct or T/C3 is to be used --> Done! T/C4 for P6 is correct
  {
    pinMode(6, OUTPUT);
    //CS32:0=1(System Clock)
    TCCR4A=(1<<COM3A1)|(0<<COM3A0)|(0<<COM3B1)|(0<<COM3B0)|(0<<COM3C1)|(0<<COM3C0)|(1<<WGM31)|(0<<WGM30); //WGM 3,2,1=1 WGM0=0 := Mode 14 Fast PWM 
    TCCR4B=(0<<ICNC3)|(0<<ICES3)|(1<<WGM33)|(1<<WGM32)|(0<<CS32)|(0<<CS31)|(1<<CS30); // prescaler = 1 (CS:001)
    ICR4=PWM_max;//16000; //define top ==> ~4khz PWM freq (16000000/(1*4096))
  //  TCCR3C Not for PWM Operation
  }

}

void SetMotorPWM(int MotorN, int duty_cycle)
{
  if(MotorN==0)//AVR Pin 5,T/C3,OC3A
{
  OCR3A=duty_cycle;
}

  if(MotorN==1)//AVR Pin 5,T/C3,OC3A
{
  OCR4A=duty_cycle;
} 

}

#endif

