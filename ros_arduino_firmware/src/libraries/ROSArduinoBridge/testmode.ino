void testmode(int arg)
{
  //TODO Remove delays for more accurate timekeeping and measure speed equalization and acceleration at different PWM rates
  //TODO Shut down as well Interrupts maybe and do the measurement exclusively (maybe not good as not possible to generalize or 
  arg = 2;
  switch (arg)
  {
    case 1:
    Serial.println("Starting TestMode Step 1... ");
    resetEncoders();
    Serial.print("Reset Encoders, 1/3 Power left Motor: ");
    Serial.print(LEFT,400);
    Serial.println(" - for 1s ...");
    setMotorSpeed(LEFT,400);
    delay(1000);
    setMotorSpeed(LEFT,0);
    Serial.print("Left Encoder Value is: ");
    Serial.println(readEncoder(LEFT));
    Serial.print("Right Encoder Value is: ");
    Serial.println(readEncoder(RIGHT));
    delay(1000);
    resetEncoders();
    Serial.print("Reset Encoders, 1/3 Power right Motor: ");
    Serial.print(LEFT,400);
    Serial.println(" - for 1s ...");
    setMotorSpeed(RIGHT,400);
    delay(1000);
    setMotorSpeed(RIGHT,0);
    Serial.print("Left Encoder Value is: ");
    Serial.println(readEncoder(LEFT));
    Serial.print("Right Encoder Value is: ");
    Serial.println(readEncoder(RIGHT));
    arg=2;
    break;
    case 2:
    Serial.println("Starting TestMode Step 1... ");
    resetEncoders();
    Serial.println("Reset Encoders, 1/3 Power both Motors Left Encoder: ");
    unsigned long targetTime;
    setMotorSpeeds(400,400);
    targetTime = micros() + 1000000;
    int i = 0;
    int deltas[500];
    while(targetTime > micros()) // do for 1 sec
    {
    if(i<500)
    {
    deltas[i]=ReportLastDiff(LEFT);
    i++;
    }
    }
    setMotorSpeeds(400,400);
    for(i=0;i<50;i++)
    {
      for(int n = 0;n<10;n++) 
      {
        Serial.print(deltas[10*i+n]);
        Serial.print(" ");
      }
      Serial.println(" ");
    }
    
    break;
    
  }
}

