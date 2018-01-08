#ifdef ACC_CONT

#ifdef DBG_ON
int n[2] = {0,0};
#endif

#define ENCODER_COUNTS_PER_REV 300 //75*4
#define DIA 50.0
#define CONV_TTPF_FROM_MMS ((ENCODER_COUNTS_PER_REV / (DIA * PI )) / (double)PID_RATE)

double acc[2]={500,500}; //Values for 0: RIGHT 1:LEFT mm/s² so 1m/s²
double dec[2]={500,500}; 
double cAcc[2] = {0,0};
int timebase = 75; //(int)(1000000 / ((double)16000000 / MAX_PWM)); // timebase to update for new setpoint in µs so calling micros() ... optimum to be in line with PWM update cycle (for setting the feedForward)
unsigned long l_com_time = 0; //time of the last command issue
unsigned long cur_time;
unsigned long l_upd_time = 0;
double currSpeed[2] = {0,0};
double finalSpeed[2] = {0,0};
double prevSpeed[2] = {0,0};


bool isGrad[2] = {false,false}; //determine  if we are in gradient phase
bool isAcc[2] = {false,false}; //determine  if we are in acc or dec phase
bool FWD[2] = {false,false};
long feedForward[2] = {0,0};

double MotorMaxSpeed[2]={2000,2000};//mm/s currently not a real value
double MotorMaxAcc[2]={5000,5000}; // mm/s² max acc at max motor Power also fake value atm

#endif
