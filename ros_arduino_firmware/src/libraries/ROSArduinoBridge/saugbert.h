#ifdef MONSTER_MOTO_SHIELD

#define headlightPin 46
#define encoderPin0A 18 //18&19 are Uart1 so not possible with usb serial conn --> usb is Uart0 so no problem ...
#define encoderPin0B 19
#define encoderPin1A 20
#define encoderPin1B 21
#define WHEEL_DIST 0.22
#define CNTRL_FREQ 50
#define PWMCNT 1200
#define STEP_P_M 391.77 //64 * 1000 / (PI * 52)// 75 *2* 1000 / (PI * 50)

boolean A0Set = false; //set variables dont need to be volatile because they are only used inside ISR
boolean B0Set = false;
boolean A1Set = false;
boolean B1Set = false;


const int inApin[2] = {7, 4};  // INA: Clockwise input
const int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
const int pwm_pin[2] = {5, 6}; // PWM input
const int cs_pin[2] = {2, 3}; // CS: Current sense ANALOG input
const int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

int rprtstp=0; //this report scheme should be skipped
int serialtime = 0;


volatile long encoderPos0 = 0L, encoderPos1 = 0L; 

//unsigned long prevPulseTime0; //shall be done at the actual calculation item
volatile unsigned long lastPulseTime0;
int prevencoderPos0 = 0;
//unsigned long prevPulseTime1;
volatile unsigned long lastPulseTime1;
int prevencoderPos1 = 0;
unsigned long currentMilis;
unsigned long lastMilis = 0;
unsigned long lastMilisReport = 0;

float Hz1;
float Hz0;

double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1.5, consKi=0.0002, consKd=0.002;
//double consKp=1, consKi=0.05, consKd=0.05;
double SetpointVM0, InputVM0, OutputVM0;
double SetpointVM1, InputVM1, OutputVM1;
char debug_str[80] = "blank";
char text[10];

void EncoderInit();
void MotorInit(int MotorN);
void MotorFWD(int MotorN, bool FWD);
void MotorEn(int MotorN, bool EN);
void InitMotorPWM(int MotorN, int PWM_max);
void SetMotorPWM(int MotorN, int duty_cycle);
#endif
