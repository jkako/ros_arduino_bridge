/* 
 *  This class shall incorporate a acceleration control including a time difference based speed regulation
 *  ==> create a set point profile based on different accelerations for accel and deceleration
 *  ==> Synchronity for LEFT and RIGHT 
 *  ==> somehow care must be taken that the lower of both is taken into account if
 *  
 *  
 *   ^
 * v |  _______
 *   | /       \
 *   |/         \
 *   ---------------->
 *                  t
 *                  
 *                  
 * ==> TODO: translate setpoints directly to motor power levels as feed forward term                 
 * ==> TODO: Take into account accel phase wit additional motor power demand (PID only for external disturbances)
 * ==> TODO: encapsule into class
 */



void Init_Acc() //currently unnecessary, so not called
{
 /* 
  acc[0] = 1000;
  acc[1] = 1000;
  dec[0] = 1000;
  dec[1] = 1000;
  currSpeed[0]=0;
  currSpeed[1]=0;
*/
#ifdef DBG_ON
Serial.print("Conversion is: ");
Serial.println(CONV_TTPF_FROM_MMS);
#endif
  
}


void CalcFF()
{
 double percPow[2];//Power level
 if(isGrad[0])
 {
  percPow[0]=currSpeed[0]/MotorMaxSpeed[0] + cAcc[0]/MotorMaxAcc[0]; 
 }
 else
 {
  percPow[0]=currSpeed[0]/MotorMaxSpeed[0];
 }
 if(isGrad[1])
 {
  percPow[1]=currSpeed[1]/MotorMaxSpeed[1] + cAcc[1]/MotorMaxAcc[1]; 
 }
 else
 {
  percPow[1]=currSpeed[1]/MotorMaxSpeed[1];
 }
feedForward[0] = MAX_PWM*percPow[0];
feedForward[1] = MAX_PWM*percPow[1];
  setMotorSpeeds(leftPID.output+feedForward[0], rightPID.output+feedForward[1]); //this is currently opposite to base definition
}

void UpdateSetpoints()
{
  /*
   * Calculate the current SetPoint
   * TODO Handling of Acc/Dec and FWD/AFT ==> to be defined in cAcc[N] --> FWWD/AFT is handled on Motor side
   * TODO is timebase required of is update done every cycle? --> Done Timebase according PWM freq for continouos feedForward adjustment (leaving 1200 clock cycles per turn at ~14kHz PWM freq)
   */
  
  cur_time = micros(); //TODO: we are talking about micros, so overrun should be considered somehow though potentially not very critical --> evtl. not critical at all
  if(cur_time >= (unsigned long)(l_upd_time+timebase))
  l_upd_time = cur_time;
    {
        if(isGrad[0])
        {
          #ifdef DBG_ON
          n[0]++;
          #endif
            currSpeed[0] = prevSpeed[0]+(cAcc[0]*(cur_time-l_com_time)/1000000); // /1000000 due to microsec
            if (((((isAcc[0])&&(FWD[0]))&&(currSpeed[0] >= finalSpeed[0]))
            ||(((isAcc[0]==false)&&(FWD[0]==false))&&(currSpeed[0] >= finalSpeed[0])))
            ||((((isAcc[0])&&(FWD[0]==false))&&(currSpeed[0] <= finalSpeed[0]))
            ||(((isAcc[0]==false)&&(FWD[0]))&&(currSpeed[0] <= finalSpeed[0]))))
            {

              isGrad[0] = false;
              currSpeed[0] = finalSpeed[0];
              #ifdef DBG_ON
              Serial.print("No. Steps : ");
              Serial.print(n[0]);
              Serial.print(" to finalSpeed[0]: ");
              Serial.print(finalSpeed[0]);
              Serial.print(" - total time: [µs] ");
              Serial.println(cur_time-l_com_time);
              n[0] = 0;
              #endif              
            }
          
          leftPID.TargetTicksPerFrame = currSpeed[0] * CONV_TTPF_FROM_MMS; //TODO: Do conversion here
        }
        
        if(isGrad[1])
        {
          #ifdef DBG_ON
          n[1]++;
          #endif
            currSpeed[1] = prevSpeed[1]+(cAcc[1]*(cur_time-l_com_time)/1000000); // /1000000 due to microsec 
            if (((((isAcc[1])&&(FWD[1]))&&(currSpeed[1] >= finalSpeed[1]))
            ||(((isAcc[1]==false)&&(FWD[1]==false))&&(currSpeed[1] >= finalSpeed[1])))
            ||((((isAcc[1])&&(FWD[1]==false))&&(currSpeed[1] <= finalSpeed[1]))
            ||(((isAcc[1]==false)&&(FWD[1]))&&(currSpeed[1] <= finalSpeed[1]))))
            {
        
              isGrad[1] = false;
              currSpeed[1] = finalSpeed[1];
              #ifdef DBG_ON
              Serial.print("No. Steps : ");
              Serial.print(n[1]);
              Serial.print(" to finalSpeed[1]: ");
              Serial.print(finalSpeed[1]);
              Serial.print(" - total time: [µs] ");
              Serial.println(cur_time-l_com_time);
              n[1] = 0;
              #endif      
            }
        rightPID.TargetTicksPerFrame = currSpeed[1] * CONV_TTPF_FROM_MMS;//TODO: Do conversion here --> Modified, Acc/Dec and Motor Parameters are converted ...
        }
     // CalcFF();  
    }

}


void newSetpoint(double L, double R)
{
    #ifdef DBG_ON
  Serial.print("L: ");
  Serial.print(L);
  Serial.print(" - R: ");
  Serial.println(R);
  #endif
  if(L == 0 && R == 0) // full stop ==> should be included in normal handling
{
  finalSpeed[0] = 0;
  currSpeed[0]=0;
  isGrad[0] = false;
  leftPID.TargetTicksPerFrame = 0;
  finalSpeed[1] = 0;
  currSpeed[1]=0;
  isGrad[1] = false;
  rightPID.TargetTicksPerFrame = 0;
}
else
{
L = L / (double)CONV_TTPF_FROM_MMS;
R = R / (double)CONV_TTPF_FROM_MMS;
    #ifdef DBG_ON
  Serial.print("L: ");
  Serial.print(L);
  Serial.print(" - R: ");
  Serial.println(R);
  #endif
if((finalSpeed[0] == L) && (finalSpeed[1] == R))
{

}
else 
{
  l_com_time = micros();
  //TODO:Obviously somehow a conversion is required to TargetTickperFrame
  //TODO:Case differentiation btw. accel and decel
  prevSpeed[0] = currSpeed[0];
  prevSpeed[1] = currSpeed[1];

  #ifdef DBG_ON
  Serial.print("prevSpeed[0]: ");
  Serial.print(prevSpeed[0]);
  Serial.print(" - prevSpeed[1]: ");
  Serial.println(prevSpeed[1]);
  #endif
  finalSpeed[0]= L;
  finalSpeed[1]= R;
  #ifdef DBG_ON
  Serial.print("finalSpeed[0]: ");
  Serial.print(finalSpeed[0]);
  Serial.print(" - finalSpeed[1]: ");
  Serial.println(finalSpeed[1]);
  #endif
  //Calculate the higher difference for RH/LH for synchronous accel
  double delta[2] = {finalSpeed[0] - prevSpeed[0],finalSpeed[1] - prevSpeed[1]};
  #ifdef DBG_ON
  Serial.print("delta[0]: ");
  Serial.print(delta[0]);
  Serial.print(" - delta[1]: ");
  Serial.println(delta[1]);
  #endif
  
  if((delta[0] < 0) && (finalSpeed[0] < 0))// either speed is increasing and positive or decreasing and negative ==> acceleration
    {
      //REW
      FWD[0]=false;
      isGrad[0] = true;
      isAcc[0] = true;
      delta[0] = -1*delta[0];
      #ifdef DBG_ON
      Serial.print("+");
      #endif
    }
  else if ((delta[0]>0)&&(finalSpeed[0] >= 0))
    {
      //FWD
      FWD[0]=true;
      isGrad[0] = true;
      isAcc[0] = true;
      #ifdef DBG_ON
      Serial.print("%");
      #endif      
    }
  else if ((delta[0] > 0) && (finalSpeed[0] < 0))// either speed is decreasing and positive or increasing and negative ==> deceleration
    {
      //REW
      FWD[0]=false;
      isGrad[0] = true;
      isAcc[0] = false; 
            #ifdef DBG_ON
      Serial.print("-");
      #endif

    }
  else if ((delta[0]<0) && (finalSpeed[0] >= 0))
    {
      //FWD
      FWD[0]=true;
      isGrad[0] = true;
      isAcc[0] = false;
      delta[0] = -1*delta[0];  
      #ifdef DBG_ON
      Serial.print("/");
      #endif    
    }
  else //somehow none of the above cases TODO: check if other cases not considered
    {
      isGrad[0] = false;    
            #ifdef DBG_ON
      Serial.print("*");
      #endif
    }
  #ifdef DBG_ON
  Serial.print("case[0] FWD Grad Acc: ");
  Serial.print(FWD[0]);
  Serial.print(isGrad[0]);
  Serial.println(isAcc[0]);
  #endif
    
  if ((delta[1] < 0) && (finalSpeed[1] < 0))// either speed is increasing and finally positive or decreasing and negative ==> acceleration
    {
      //REW
      FWD[1] = false;
      isGrad[1] = true;
      isAcc[1] = true;
      delta[1] = -1*delta[1];
    }
  else if ((delta[1] > 0 ) && (finalSpeed[1] >= 0))
    {
      //FWD
      FWD[1] = true;
      isGrad[1] = true;
      isAcc[1] = true; 
    }
  else if ((delta[1] > 0) && (finalSpeed[1] < 0))// either speed is decreasing and positive or increasing and negative ==> deceleration
    {
      //REW
      FWD[1] = false;
      isGrad[1] = true;
      isAcc[1] = false; 
    }
  else if ((delta[1] < 0) && (finalSpeed[1] >= 0))
    {
      //FWD
      FWD[1] = true;
      isGrad[1] = true;
      isAcc[1] = false;  
      delta[1] = -1*delta[1];   
    }
  else //somehow none of the above cases TODO: check if other cases not considered
    {
      isGrad[1] = false;    
    }

  #ifdef DBG_ON
  Serial.print("case[1]  FWD Grad Acc: ");
  Serial.print(FWD[1]);
  Serial.print(isGrad[1]);
  Serial.println(isAcc[1]);
  #endif

  if(isGrad[0] || isGrad[1])
  {
    //if(isAcc[0]) // As long as no case differentiation is performed assume acc[N] = dec[N]/Problem is diff is per drive 
//    if(delta[0] < 0) delta[0]=delta[0]*-1;
//    if(delta[1] < 0) delta[1]=delta[1]*-1; //Workaround to allow without case diff as long as acc[N] = dec[N]
    if((delta[0])>=(delta[1])) //only valid for accel(see above); R is higher accel use max here and ratio for R
    {
      cAcc[0] = acc[0];
      cAcc[1] = (delta[1]/delta[0])*acc[0]; //TODO: check needed is cAcc[1] is <= acc[1] --> Done
     if(cAcc[1] > acc[1])
     {
     cAcc[0] = (delta[0]/delta[1])*acc[1];
     cAcc[1] = acc[1];
     }
    }
    else
    {
      cAcc[1] = acc[1];
      cAcc[0] = (delta[0]/delta[1])*acc[1]; //TODO: check needed is cAcc[1] is <= acc[1] --> Done
     if(cAcc[0] > acc[0])
     {
     cAcc[1] = (delta[1]/delta[0])*acc[0];
     cAcc[0] = acc[0];
     }
    }
    if(((isAcc[0]) && (FWD[0]==false)) || ((isAcc[0]==false) && (FWD[0]))) cAcc[0] = -1*cAcc[0];
    if(((isAcc[1]) && (FWD[1]==false)) || ((isAcc[1]==false) && (FWD[1]))) cAcc[1] = -1*cAcc[1];
  }
}
  #ifdef DBG_ON
  Serial.print("cAcc[0]: ");
  Serial.print(cAcc[0]);
  Serial.print(" - cAcc[1]: ");
  Serial.println(cAcc[1]);
  #endif
}
}

