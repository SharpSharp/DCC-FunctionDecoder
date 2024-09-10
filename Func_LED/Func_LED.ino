#include <NmraDcc.h>

// Define the Arduino input Pin number for the DCC Signal 
#define DCC_PIN     2
#define THIS_DECODER_ADDRESS 91

#define DAY_HEADLIGHT 9
#define NIGHT_HEADLIGHT 10
#define TAILLIGHT 7
#define CAB_LIGHT 3
#define VAN_LIGHT 5

bool DirectionState = 1;
bool F0State = 0;
bool F1State = 0;
bool F2State = 0;
bool F3State = 0;
bool F4State = 0;

byte brightness = 50;

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS,      THIS_DECODER_ADDRESS},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_MSB(THIS_DECODER_ADDRESS)},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_LSB(THIS_DECODER_ADDRESS)},
  {CV_29_CONFIG,                          CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
};

NmraDcc Dcc;

uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault()
{
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps)
{
  Serial.print("notifyDccFunc: Addr: ");
  Serial.println(Addr,DEC);
  
  DirectionState = Dir; // HIGH is forward, rev
};

void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  Serial.print("notifyDccFunc: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  Serial.print("  Function Group: ");
  Serial.print(FuncGrp,DEC);         
  Serial.print(" FN 0: ");
  Serial.println(FuncState,HEX);

  F0State = FuncState & FN_BIT_00;
  F1State = FuncState & FN_BIT_01;
  F2State = FuncState & FN_BIT_02;
  F3State = FuncState & FN_BIT_03;
  F4State = FuncState & FN_BIT_04;
}

void setup()
{
  Serial.begin(115200);
  uint8_t maxWaitLoops = 255;
  while(!Serial && maxWaitLoops--)
    delay(20);

  Serial.println(THIS_DECODER_ADDRESS);
  Serial.println("NMRA Dcc Multifunction Decoder Demo 1");

  pinMode( DAY_HEADLIGHT, OUTPUT );
  pinMode( NIGHT_HEADLIGHT, OUTPUT );
  pinMode( TAILLIGHT, OUTPUT );
  pinMode( CAB_LIGHT, OUTPUT );
  pinMode( VAN_LIGHT, OUTPUT );
  analogWrite ( DAY_HEADLIGHT, 0 );
  analogWrite ( NIGHT_HEADLIGHT, 0 );
  digitalWrite( TAILLIGHT, LOW );
  digitalWrite( CAB_LIGHT, LOW );
  digitalWrite( VAN_LIGHT, LOW );
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif

  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY, 0 );

  notifyCVResetFactoryDefault();

  if( FactoryDefaultCVIndex && Dcc.isSetCVReady()) 
  {
    for (int i=0; i < FactoryDefaultCVIndex; i++ ) 
    { 
      Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
    }
  }  
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // headlights set for GB practice. Taillights are used as parking lights. When headlight is off taillights are on.
  // When unit is a driving car at oposite end to the loco change to !DirectionState so headlighs are on in reverse.
  if (F0State && DirectionState) 
  {
    digitalWrite(TAILLIGHT, LOW);
    // function 4 used to toggle day or night headlight mode
    if (F4State)
    {
      analogWrite(DAY_HEADLIGHT, 255);
      analogWrite(NIGHT_HEADLIGHT, brightness);
    } else
    {
      analogWrite(DAY_HEADLIGHT, brightness);
      analogWrite(NIGHT_HEADLIGHT, 255);
    }
  } else 
  {
    analogWrite(DAY_HEADLIGHT, 0);
    analogWrite(NIGHT_HEADLIGHT, 0);
    digitalWrite(TAILLIGHT, HIGH);
  }
  F2State ? digitalWrite(CAB_LIGHT, HIGH) : digitalWrite(CAB_LIGHT, LOW); 
  F3State ? digitalWrite(VAN_LIGHT, HIGH) : digitalWrite(VAN_LIGHT, LOW);
  
}
